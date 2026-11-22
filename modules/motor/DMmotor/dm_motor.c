#include "dm_motor.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_board.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

static uint8_t idx = 0;
static DMMotorInstance *dm_motor_instance[DM_MOTOR_CNT] = {NULL};

/* 参数读写后的一小段屏蔽窗口(ms): 参数回复和反馈帧都落在 Master ID 上、长度也都是 8,
 * 光看帧头分不开, 所以读写参数后这段时间内不解反馈。 */
#define DM_PARAM_GUARD_MS 30.0f

/**
 * @brief 浮点数按量程映射为定点数
 */
static uint16_t DMFloatToUint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x - x_min;

    if (x < x_min)
        offset = 0.0f;
    else if (x > x_max)
        offset = span;

    return (uint16_t)(offset * (float)((1U << bits) - 1U) / span);
}

static float DMUintToFloat(uint16_t x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    return (float)x * span / (float)((1U << bits) - 1U) + x_min;
}

/**
 * @brief 用指定帧ID发送当前 tx_buff
 *
 * CAN 实例的 tx_id 和 txconf.Identifier 是在注册时定死的, 而达妙不同模式下控制帧
 * 的 ID 不同(0x100/0x200/0x300+ID), 所以这里临时改一下再发, 发完还原。
 */
static void DMSendOn(DMMotorInstance *motor, uint32_t frame_id, uint8_t len)
{
    CANInstance *ins = motor->motor_can_instance;
    uint32_t saved_id = ins->tx_id;

    /* ★ 发送 FIFO 满了就直接丢这一帧, 不要等。
     * 总线没接电机/没接 120Ω 时会一直是 bus-off, 每帧都要等 1ms 超时;
     * 7 个电机 × 1ms 会把 1kHz 的 MOTOR 任务拖死(实测 MOTOR 任务延迟到 8~10ms)。
     * 达妙的控制帧本来就是每 1ms 重发一次, 丢一帧无影响。 */
    if (HAL_FDCAN_GetTxFifoFreeLevel(ins->can_handle) == 0)
        return;

    ins->tx_id = frame_id;
    ins->txconf.Identifier = frame_id;
    CANSetDLC(ins, len);
    CANTransmit(ins, 1);

    ins->tx_id = saved_id;
    ins->txconf.Identifier = saved_id;
}

/**
 * @brief 发送一条命令帧(数据区固定 0xFF...0xFC 之类的格式)
 */
static void DMMotorSendCommandAt(DMMotorInstance *motor, uint32_t frame_id, uint8_t last_byte)
{
    for (uint8_t i = 0; i < 7; ++i)
        motor->motor_can_instance->tx_buff[i] = 0xFF;
    motor->motor_can_instance->tx_buff[7] = last_byte;

    DMSendOn(motor, frame_id, 8);
}

/**
 * @brief 按 MIT 协议打包并发送一帧控制
 */
static void DMMotorSendMITNow(DMMotorInstance *motor, float position, float velocity,
                              float kp, float kd, float torque_ff)
{
    DMMotor_Range_s *r = &motor->range;
    uint8_t *d = motor->motor_can_instance->tx_buff;

    uint16_t p_int = DMFloatToUint(position, r->p_min, r->p_max, 16);
    uint16_t v_int = DMFloatToUint(velocity, r->v_min, r->v_max, 12);
    uint16_t kp_int = DMFloatToUint(kp, r->kp_min, r->kp_max, 12);
    uint16_t kd_int = DMFloatToUint(kd, r->kd_min, r->kd_max, 12);
    uint16_t t_int = DMFloatToUint(torque_ff, r->t_min, r->t_max, 12);

    d[0] = (uint8_t)(p_int >> 8);
    d[1] = (uint8_t)(p_int & 0xFF);
    d[2] = (uint8_t)(v_int >> 4);
    d[3] = (uint8_t)(((v_int & 0x0F) << 4) | (kp_int >> 8));
    d[4] = (uint8_t)(kp_int & 0xFF);
    d[5] = (uint8_t)(kd_int >> 4);
    d[6] = (uint8_t)(((kd_int & 0x0F) << 4) | (t_int >> 8));
    d[7] = (uint8_t)(t_int & 0xFF);

    DMSendOn(motor, motor->motor_id, 8);
}

/**
 * @brief 速度模式控制帧: ID = 0x200 + ID, D[0..3] = float rad/s (小端)
 */
static void DMMotorSendVelNow(DMMotorInstance *motor, float rad_s)
{
    uint8_t *d = motor->motor_can_instance->tx_buff;
    float v = rad_s;

    memcpy(&d[0], &v, 4);
    d[4] = 0;
    d[5] = 0;
    d[6] = 0;
    d[7] = 0;

    DMSendOn(motor, 0x200u + motor->motor_id, 8);
}

/**
 * @brief 位置速度模式控制帧: ID = 0x100 + ID, D[0..3]=位置 D[4..7]=限速
 */
static void DMMotorSendPosVelNow(DMMotorInstance *motor, float pos_rad, float vel_limit)
{
    uint8_t *d = motor->motor_can_instance->tx_buff;
    float p = pos_rad;
    float v = vel_limit;

    memcpy(&d[0], &p, 4);
    memcpy(&d[4], &v, 4);

    DMSendOn(motor, 0x100u + motor->motor_id, 8);
}

/**
 * @brief 解析反馈帧
 */
static void DecodeDMMotor(CANInstance *_instance)
{
    DMMotorInstance *motor = (DMMotorInstance *)_instance->id;
    DMMotor_Measure_s *measure = &motor->measure;
    uint8_t *d = _instance->rx_buff;
    DMMotor_Range_s *r = &motor->range;

    DaemonReload(motor->daemon);
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    /* 参数读写的回复也落在这个 ID 上, 长度同样是 8, 会污染反馈, 直接丢 */
    if (motor->param_guard_tick &&
        (DWT_GetTimeline_ms() - motor->param_guard_tick) < DM_PARAM_GUARD_MS)
        return;

    /* 参数回复的特征: D[2] 是 0x33/0x55 且 D[0..1] 是 CAN ID */
    if ((d[2] == 0x33 || d[2] == 0x55) &&
        (((uint16_t)d[0] | ((uint16_t)d[1] << 8)) == motor->motor_id))
    {
        if (d[3] == DM_RID_CTRL_MODE)
            motor->mode_readback = (uint8_t)d[4];
        return;
    }

    uint16_t p_int = ((uint16_t)d[1] << 8) | d[2];
    uint16_t v_int = ((uint16_t)d[3] << 4) | (d[4] >> 4);
    uint16_t t_int = (((uint16_t)d[4] & 0x0F) << 8) | d[5];

    measure->state = (d[0] >> 4) & 0x0F;
    measure->position = DMUintToFloat(p_int, r->p_min, r->p_max, 16);
    measure->velocity = DMUintToFloat(v_int, r->v_min, r->v_max, 12);
    measure->torque = DMUintToFloat(t_int, r->t_min, r->t_max, 12);
    measure->temperature_mos = d[6];
    measure->temperature_rotor = d[7];

    /* 与 DJI 电机保持一致的"角度制"接口: 达妙反馈的是输出轴绝对角度(rad),
     * 这里换算成度, 使角度环/速度环的 PID 参数含义与 DJI 电机一致 */
    measure->position_deg = measure->position * RAD_2_DEGREE;
    measure->velocity_deg = measure->velocity * RAD_2_DEGREE;
}

static void DMMotorLostCallback(void *motor_ptr)
{
    DMMotorInstance *motor = (DMMotorInstance *)motor_ptr;

    /* 总线上一台电机都没接的时候, 这个回调会一直触发,
     * 不限速的话日志会把 RTT 缓冲冲爆, 把真正有用的启动日志挤掉 */
    static float last_log[DM_MOTOR_CNT];
    uint8_t i = (motor->motor_id < DM_MOTOR_CNT) ? (uint8_t)motor->motor_id : 0;
    float now = DWT_GetTimeline_ms();

    if ((now - last_log[i]) < 2000.0f)
        return;
    last_log[i] = now;
    LOGWARNING("[dm_motor] Motor lost, id [%d]", (int)motor->motor_id);
}

DMMotorInstance *DMMotorInit(DMMotor_Init_Config_s *config)
{
    DMMotorInstance *instance = (DMMotorInstance *)malloc(sizeof(DMMotorInstance));
    memset(instance, 0, sizeof(DMMotorInstance));

    instance->motor_type = MOTOR_TYPE_NONE;
    instance->motor_settings = config->controller_setting_init_config;
    instance->range = config->range;
    instance->motor_id = (uint16_t)config->motor_id;
    instance->kp = config->kp;
    instance->kd = config->kd;
    instance->mit_torque_ff = 0.0f;
    instance->ctrl_mode = (config->ctrl_mode == 0) ? DM_MODE_MIT : config->ctrl_mode;
    instance->cmd_velocity = 0.0f;
    instance->cmd_position = 0.0f;
    instance->cmd_vel_limit = 10.0f;

    /* 若没有填量程, 使用达妙 4310 的默认量程 */
    if (instance->range.p_max == 0.0f && instance->range.p_min == 0.0f)
    {
        instance->range.p_min = DM_DEFAULT_P_MIN;
        instance->range.p_max = DM_DEFAULT_P_MAX;
        instance->range.v_min = DM_DEFAULT_V_MIN;
        instance->range.v_max = DM_DEFAULT_V_MAX;
        instance->range.kp_min = DM_DEFAULT_KP_MIN;
        instance->range.kp_max = DM_DEFAULT_KP_MAX;
        instance->range.kd_min = DM_DEFAULT_KD_MIN;
        instance->range.kd_max = DM_DEFAULT_KD_MAX;
        instance->range.t_min = DM_DEFAULT_T_MIN;
        instance->range.t_max = DM_DEFAULT_T_MAX;
    }

    PIDInit(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&instance->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    instance->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
    instance->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;

    /* 达妙电机的控制帧与反馈帧使用同一个 ID(约定 Master ID = CAN ID) */
    /* tx_id = 电机 ID(控制帧); rx_id 由调用者给出, 对应电机里的 Master ID(出厂 0)
     * 之前这里强行写成 motor_id, 会让 Master ID=0 的电机永远收不到反馈 */
    config->can_init_config.tx_id = config->motor_id;
    /* rx_id = 反馈帧 ID = 电机里的 Master ID:
     *   DM_RX_ID_AUTO(0xFF) -> 自动取 motor_id (兼容原来不写 rx_id 的配置)
     *   其它值(含 0)        -> 原样使用 (出厂 Master ID 可能就是 0) */
    if (config->can_init_config.rx_id == DM_RX_ID_AUTO)
        config->can_init_config.rx_id = config->motor_id;
    config->can_init_config.is_fd = 0; /* 达妙用经典 CAN */
    config->can_init_config.brs = 0;
    config->can_init_config.can_module_callback = DecodeDMMotor;
    config->can_init_config.id = instance;

    instance->motor_can_instance = CANRegister(&config->can_init_config);

    Daemon_Init_Config_s daemon_config = {
        .callback = DMMotorLostCallback,
        .owner_id = instance,
        .reload_count = 5,
    };
    instance->daemon = DaemonRegister(&daemon_config);

    /* MIT 模式下这里直接使能; 其它模式由应用层走"上电时序"(先切模式再使能)，
     * 因为模式切换会把指令清零, 顺序反了就白使能一次 */
    if (instance->ctrl_mode == DM_MODE_MIT)
        DMMotorEnable(instance);

    dm_motor_instance[idx++] = instance;

    LOGINFO("[dm_motor] motor id [%d] mode [%d] registered", (int)instance->motor_id, (int)instance->ctrl_mode);
    return instance;
}

void DMMotorSetRef(DMMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

void DMMotorSetMIT(DMMotorInstance *motor, float position, float velocity,
                   float kp, float kd, float torque_ff)
{
    DMMotorSendMITNow(motor, position, velocity, kp, kd, torque_ff);
}

/**
 * @brief 设置"电机内部 MIT 位置环"目标(由 DMMotorControl 每 1ms 发送)
 */
void DMMotorSetMITCmd(DMMotorInstance *motor, float pos_deg, float kp, float kd, float torque_ff)
{
    if (motor == NULL)
        return;

    motor->motor_controller.pid_ref = pos_deg; /* 该模式下 pid_ref 当"目标角度(度)"用 */
    motor->kp = kp;
    motor->kd = kd;
    motor->mit_torque_ff = torque_ff;
}

void DMMotorSetVelocity(DMMotorInstance *motor, float rad_s)
{
    motor->cmd_velocity = rad_s;
}

void DMMotorSetPosVel(DMMotorInstance *motor, float pos_rad, float vel_limit)
{
    motor->cmd_position = pos_rad;
    motor->cmd_vel_limit = vel_limit;
}

void DMMotorEnable(DMMotorInstance *motor)
{
    DMMotorSendCommandAt(motor, motor->motor_id, DM_CMD_ENABLE);
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorEnableAt(DMMotorInstance *motor, uint32_t frame_id)
{
    DMMotorSendCommandAt(motor, frame_id, DM_CMD_ENABLE);
    motor->stop_flag = MOTOR_ENALBED;
}

void DMMotorDisable(DMMotorInstance *motor)
{
    /* 失能帧的 ID 也要跟当前模式的"控制帧 ID"一致(手册 4.3 "帧ID如前所述") */
    uint32_t id = motor->motor_id;
    if (motor->ctrl_mode == DM_MODE_VEL)
        id = 0x200u + motor->motor_id;
    else if (motor->ctrl_mode == DM_MODE_POS_VEL)
        id = 0x100u + motor->motor_id;
    else if (motor->ctrl_mode == DM_MODE_POS_FORCE)
        id = 0x300u + motor->motor_id;

    DMMotorSendCommandAt(motor, id, DM_CMD_DISABLE);
    motor->stop_flag = MOTOR_STOP;
}

void DMMotorSetZero(DMMotorInstance *motor)
{
    DMMotorSendCommandAt(motor, motor->motor_id, DM_CMD_SAVE_ZERO);
}

void DMMotorClearError(DMMotorInstance *motor)
{
    DMMotorSendCommandAt(motor, motor->motor_id, DM_CMD_CLEAR_ERR);
}

void DMMotorSetCtrlMode(DMMotorInstance *motor, uint8_t mode)
{
    DMMotorWriteParam(motor, DM_RID_CTRL_MODE, (uint32_t)mode);
    motor->ctrl_mode = mode;
}

void DMMotorWriteParam(DMMotorInstance *motor, uint8_t rid, uint32_t data)
{
    uint8_t *d = motor->motor_can_instance->tx_buff;

    d[0] = (uint8_t)(motor->motor_id & 0xFF);
    d[1] = (uint8_t)((motor->motor_id >> 8) & 0xFF);
    d[2] = 0x55; /* 写 */
    d[3] = rid;
    d[4] = (uint8_t)(data & 0xFF);
    d[5] = (uint8_t)((data >> 8) & 0xFF);
    d[6] = (uint8_t)((data >> 16) & 0xFF);
    d[7] = (uint8_t)((data >> 24) & 0xFF);

    DMSendOn(motor, 0x7FF, 8);
    motor->param_guard_tick = (uint32_t)DWT_GetTimeline_ms();
}

void DMMotorWriteParamFloat(DMMotorInstance *motor, uint8_t rid, float data)
{
    uint32_t raw;
    memcpy(&raw, &data, sizeof(raw));
    DMMotorWriteParam(motor, rid, raw);
}

void DMMotorReadParam(DMMotorInstance *motor, uint8_t rid)
{
    uint8_t *d = motor->motor_can_instance->tx_buff;

    d[0] = (uint8_t)(motor->motor_id & 0xFF);
    d[1] = (uint8_t)((motor->motor_id >> 8) & 0xFF);
    d[2] = 0x33; /* 读 */
    d[3] = rid;
    d[4] = 0;
    d[5] = 0;
    d[6] = 0;
    d[7] = 0;

    DMSendOn(motor, 0x7FF, 8);
    motor->param_guard_tick = (uint32_t)DWT_GetTimeline_ms();
}

void DMMotorSaveParams(DMMotorInstance *motor)
{
    uint8_t *d = motor->motor_can_instance->tx_buff;

    d[0] = (uint8_t)(motor->motor_id & 0xFF);
    d[1] = (uint8_t)((motor->motor_id >> 8) & 0xFF);
    d[2] = 0xAA;
    d[3] = 0x01;
    d[4] = 0;
    d[5] = 0;
    d[6] = 0;
    d[7] = 0;

    DMSendOn(motor, 0x7FF, 8);
    motor->param_guard_tick = (uint32_t)DWT_GetTimeline_ms();
}

/**
 * @brief 1kHz 调用: 按各自模式发控制帧
 *
 * MIT 模式(默认): 沿用框架的三环串级 PID, 输出作为 t_ff 下发(kp=kd=0 即纯力矩);
 *                  若实例的 kp>0, 则直接把 pid_ref 当目标位置交给电机内部位置环。
 * 速度模式:        每周期重发 cmd_velocity(达妙速度环在电机内部)。
 * 位置速度模式:    每周期重发 cmd_position/cmd_vel_limit。
 */
void DMMotorControl(void)
{
    DMMotorInstance *motor;
    Motor_Control_Setting_s *setting;
    Motor_Controller_s *controller;
    DMMotor_Measure_s *measure;
    float pid_measure, pid_ref;
    float torque_out;

    for (size_t i = 0; i < idx; ++i)
    {
        motor = dm_motor_instance[i];
        setting = &motor->motor_settings;
        controller = &motor->motor_controller;
        measure = &motor->measure;

        /* ---------- 速度模式: 电机内部闭环, 直接重发目标转速 ---------- */
        if (motor->ctrl_mode == DM_MODE_VEL)
        {
            DMMotorSendVelNow(motor, (motor->stop_flag == MOTOR_STOP) ? 0.0f : motor->cmd_velocity);
            continue;
        }

        /* ---------- 位置速度模式 ---------- */
        if (motor->ctrl_mode == DM_MODE_POS_VEL)
        {
            DMMotorSendPosVelNow(motor,
                                 (motor->stop_flag == MOTOR_STOP) ? motor->measure.position : motor->cmd_position,
                                 motor->cmd_vel_limit);
            continue;
        }

        /* ---------- MIT 模式 ---------- */
        if (motor->stop_flag == MOTOR_STOP)
        {
            DMMotorSendMITNow(motor, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            continue;
        }

        pid_ref = controller->pid_ref;
        if (setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1.0f;

        /* 电机内部位置环模式: 直接把目标位置下发, 由电机自己做位置伺服 */
        if (motor->kp > 0.0f)
        {
            /* 电机内部位置环: 目标(度→rad) + kp/kd + 力矩前馈(重力等) */
            DMMotorSendMITNow(motor, pid_ref * DEGREE_2_RAD, 0.0f, motor->kp, motor->kd,
                              motor->mit_torque_ff);
            continue;
        }

        /* 力矩模式: 框架三环 PID 算出目标力矩 */

        // 位置环
        if ((setting->close_loop_type & ANGLE_LOOP) && setting->outer_loop_type == ANGLE_LOOP)
        {
            if (setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->position_deg;
            pid_ref = PIDCalculate(&controller->angle_PID, pid_measure, pid_ref);
        }

        // 速度环
        if ((setting->close_loop_type & SPEED_LOOP) &&
            (setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *controller->speed_feedforward_ptr;

            if (setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *controller->other_speed_feedback_ptr;
            else
                pid_measure = measure->velocity_deg;
            pid_ref = PIDCalculate(&controller->speed_PID, pid_measure, pid_ref);
        }

        // 力矩(电流)环
        if (setting->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += *controller->current_feedforward_ptr;
        if (setting->close_loop_type & CURRENT_LOOP)
        {
            pid_ref = PIDCalculate(&controller->current_PID, measure->torque, pid_ref);
        }

        if (setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1.0f;

        torque_out = pid_ref;
        DMMotorSendMITNow(motor, 0.0f, 0.0f, 0.0f, 0.0f, torque_out);
    }
}

/**
 * @brief 再挂一个接收帧 ID 别名(与主实例共用同一份 motor 数据)
 */
uint8_t DMMotorAttachAlias(DMMotorInstance *motor, uint32_t alt_rx_id)
{
    CAN_Init_Config_s cfg;

    if (motor == NULL || motor->motor_can_instance == NULL)
        return 0;

    if (alt_rx_id == motor->motor_can_instance->rx_id)
        return 1; /* 已经是主 ID */

    memset(&cfg, 0, sizeof(cfg));
    cfg.can_handle = motor->motor_can_instance->can_handle;
    cfg.tx_id = motor->motor_id;
    cfg.rx_id = alt_rx_id;
    cfg.nominal_bitrate = 1000000;
    cfg.is_fd = 0;
    cfg.can_module_callback = DecodeDMMotor;
    cfg.id = motor;

    return (CANRegister(&cfg) != NULL) ? 1u : 0u;
}
