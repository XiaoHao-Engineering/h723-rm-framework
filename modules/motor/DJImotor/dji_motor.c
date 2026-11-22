#include "dji_motor.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_board.h"

static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
/* DJI电机的实例,此处仅保存指针,内存的分配将通过电机实例初始化时通过malloc()进行 */
static DJIMotorInstance *dji_motor_instance[DJI_MOTOR_CNT] = {NULL}; // 会在control任务中遍历该指针数组进行pid计算

/**
 * @brief DJI 电机以"一帧控制4个电机"的方式发送, 因此用 3 条总线 * 3 个分组 = 9 个
 *        can_instance 专门负责发送, 这些实例只在本地使用, 不注册到 bsp_can.
 *
 *        C610(m2006)/C620(m3508): 0x1FF, 0x200
 *        GM6020:                  0x1FF, 0x2FF
 *        反馈(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
 *
 *        下标: bus0: [0]0x1FF [1]0x200 [2]0x2FF
 *              bus1: [3]0x1FF [4]0x200 [5]0x2FF
 *              bus2: [6]0x1FF [7]0x200 [8]0x2FF
 */
static CANInstance sender_assignment[DEVICE_CAN_CNT * 3] = {
    [0] = {.can_handle = BOARD_CAN1, .txconf = {.Identifier = 0x1ff, .IdType = FDCAN_STANDARD_ID,
                                                .TxFrameType = FDCAN_DATA_FRAME, .DataLength = 8,
                                                .ErrorStateIndicator = FDCAN_ESI_ACTIVE, .BitRateSwitch = FDCAN_BRS_OFF,
                                                .FDFormat = FDCAN_CLASSIC_CAN, .TxEventFifoControl = FDCAN_NO_TX_EVENTS},
           .tx_len = 8, .tx_id = 0x1ff},
    [1] = {.can_handle = BOARD_CAN1, .txconf = {.Identifier = 0x200, .IdType = FDCAN_STANDARD_ID,
                                                .TxFrameType = FDCAN_DATA_FRAME, .DataLength = 8,
                                                .ErrorStateIndicator = FDCAN_ESI_ACTIVE, .BitRateSwitch = FDCAN_BRS_OFF,
                                                .FDFormat = FDCAN_CLASSIC_CAN, .TxEventFifoControl = FDCAN_NO_TX_EVENTS},
           .tx_len = 8, .tx_id = 0x200},
    [2] = {.can_handle = BOARD_CAN1, .txconf = {.Identifier = 0x2ff, .IdType = FDCAN_STANDARD_ID,
                                                .TxFrameType = FDCAN_DATA_FRAME, .DataLength = 8,
                                                .ErrorStateIndicator = FDCAN_ESI_ACTIVE, .BitRateSwitch = FDCAN_BRS_OFF,
                                                .FDFormat = FDCAN_CLASSIC_CAN, .TxEventFifoControl = FDCAN_NO_TX_EVENTS},
           .tx_len = 8, .tx_id = 0x2ff},
    [3] = {.can_handle = BOARD_CAN2, .txconf = {.Identifier = 0x1ff, .IdType = FDCAN_STANDARD_ID,
                                                .TxFrameType = FDCAN_DATA_FRAME, .DataLength = 8,
                                                .ErrorStateIndicator = FDCAN_ESI_ACTIVE, .BitRateSwitch = FDCAN_BRS_OFF,
                                                .FDFormat = FDCAN_CLASSIC_CAN, .TxEventFifoControl = FDCAN_NO_TX_EVENTS},
           .tx_len = 8, .tx_id = 0x1ff},
    [4] = {.can_handle = BOARD_CAN2, .txconf = {.Identifier = 0x200, .IdType = FDCAN_STANDARD_ID,
                                                .TxFrameType = FDCAN_DATA_FRAME, .DataLength = 8,
                                                .ErrorStateIndicator = FDCAN_ESI_ACTIVE, .BitRateSwitch = FDCAN_BRS_OFF,
                                                .FDFormat = FDCAN_CLASSIC_CAN, .TxEventFifoControl = FDCAN_NO_TX_EVENTS},
           .tx_len = 8, .tx_id = 0x200},
    [5] = {.can_handle = BOARD_CAN2, .txconf = {.Identifier = 0x2ff, .IdType = FDCAN_STANDARD_ID,
                                                .TxFrameType = FDCAN_DATA_FRAME, .DataLength = 8,
                                                .ErrorStateIndicator = FDCAN_ESI_ACTIVE, .BitRateSwitch = FDCAN_BRS_OFF,
                                                .FDFormat = FDCAN_CLASSIC_CAN, .TxEventFifoControl = FDCAN_NO_TX_EVENTS},
           .tx_len = 8, .tx_id = 0x2ff},
    [6] = {.can_handle = BOARD_CAN3, .txconf = {.Identifier = 0x1ff, .IdType = FDCAN_STANDARD_ID,
                                                .TxFrameType = FDCAN_DATA_FRAME, .DataLength = 8,
                                                .ErrorStateIndicator = FDCAN_ESI_ACTIVE, .BitRateSwitch = FDCAN_BRS_OFF,
                                                .FDFormat = FDCAN_CLASSIC_CAN, .TxEventFifoControl = FDCAN_NO_TX_EVENTS},
           .tx_len = 8, .tx_id = 0x1ff},
    [7] = {.can_handle = BOARD_CAN3, .txconf = {.Identifier = 0x200, .IdType = FDCAN_STANDARD_ID,
                                                .TxFrameType = FDCAN_DATA_FRAME, .DataLength = 8,
                                                .ErrorStateIndicator = FDCAN_ESI_ACTIVE, .BitRateSwitch = FDCAN_BRS_OFF,
                                                .FDFormat = FDCAN_CLASSIC_CAN, .TxEventFifoControl = FDCAN_NO_TX_EVENTS},
           .tx_len = 8, .tx_id = 0x200},
    [8] = {.can_handle = BOARD_CAN3, .txconf = {.Identifier = 0x2ff, .IdType = FDCAN_STANDARD_ID,
                                                .TxFrameType = FDCAN_DATA_FRAME, .DataLength = 8,
                                                .ErrorStateIndicator = FDCAN_ESI_ACTIVE, .BitRateSwitch = FDCAN_BRS_OFF,
                                                .FDFormat = FDCAN_CLASSIC_CAN, .TxEventFifoControl = FDCAN_NO_TX_EVENTS},
           .tx_len = 8, .tx_id = 0x2ff},
};

/**
 * @brief 6个用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧
 */
static uint8_t sender_enable_flag[DEVICE_CAN_CNT * 3] = {0};

/**
 * @brief 由 FDCAN 句柄得到总线序号(0/1/2)
 */
static uint8_t DJIMotorBusIndex(FDCAN_HandleTypeDef *handle)
{
    if (handle == BOARD_CAN1)
        return 0;
    if (handle == BOARD_CAN2)
        return 1;
    if (handle == BOARD_CAN3)
        return 2;
    return 0;
}

/**
 * @brief 根据电调/拨码开关上的ID,根据说明书的默认id分配方式计算发送ID和接收ID,
 *        并对电机进行分组以便处理多电机控制命令
 */
static void MotorSenderGrouping(DJIMotorInstance *motor, CAN_Init_Config_s *config)
{
    uint8_t motor_id = config->tx_id - 1; // 下标从零开始,先减一方便赋值
    uint8_t motor_send_num;
    uint8_t motor_grouping;
    uint8_t bus_base = DJIMotorBusIndex(config->can_handle) * 3; // 每条总线有 3 个发送分组

    /* 检查是否发生id冲突 */
    for (size_t i = 0; i < idx; ++i)
    {
        if (dji_motor_instance[i]->motor_can_instance->can_handle == config->can_handle &&
            dji_motor_instance[i]->motor_can_instance->rx_id == config->rx_id && config->rx_id != 0)
        {
            LOGERROR("[dji_motor] ID crash. Check in debug mode, add dji_motor_instance to watch to get more information.");
            while (1)
                LOGERROR("[dji_motor] id [%d], can bus [%d]", (int)config->rx_id, (int)(bus_base / 3 + 1));
        }
    }

    switch (motor->motor_type)
    {
    case M2006:
    case M3508:
        if (motor_id < 4) // 根据ID分组
        {
            motor_send_num = motor_id;
            motor_grouping = bus_base + 1; // 0x200
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = bus_base + 0; // 0x1FF
        }
        config->rx_id = 0x200 + motor_id + 1; // 把ID+1,进行分组设置
        break;

    case GM6020:
        if (motor_id < 4)
        {
            motor_send_num = motor_id;
            motor_grouping = bus_base + 0; // 0x1FF
        }
        else
        {
            motor_send_num = motor_id - 4;
            motor_grouping = bus_base + 2; // 0x2FF
        }
        config->rx_id = 0x204 + motor_id + 1; // 把ID+1,进行分组设置
        break;

    default: // other motors should not be registered here
        while (1)
            LOGERROR("[dji_motor]You must not register other motors using the API of DJI motor.");
    }

    sender_enable_flag[motor_grouping] = 1; // 设置发送标志位,防止发送空帧
    motor->message_num = motor_send_num;
    motor->sender_group = motor_grouping;

    /* 同一总线同一分组的所有电机共用一帧报文 */
    CANSetDLC(&sender_assignment[motor_grouping], 8);
}

/**
 * @brief 根据返回的can_instance对反馈报文进行解析
 */
static void DecodeDJIMotor(CANInstance *_instance)
{
    uint8_t *rxbuff = _instance->rx_buff;
    DJIMotorInstance *motor = (DJIMotorInstance *)_instance->id;
    DJI_Motor_Measure_s *measure = &motor->measure; // measure要多次使用,保存指针减小访存开销

    DaemonReload(motor->daemon);
    motor->dt = DWT_GetDeltaT(&motor->feed_cnt);

    // 解析数据并对电流和速度进行滤波,电机的反馈报文具体格式见电机说明手册
    measure->last_ecd = measure->ecd;
    measure->ecd = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
    measure->angle_single_round = ECD_ANGLE_COEF_DJI * (float)measure->ecd;
    measure->speed_aps = (1.0f - SPEED_SMOOTH_COEF) * measure->speed_aps +
                         RPM_2_ANGLE_PER_SEC * SPEED_SMOOTH_COEF * (float)((int16_t)(rxbuff[2] << 8 | rxbuff[3]));
    measure->real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure->real_current +
                            CURRENT_SMOOTH_COEF * (float)((int16_t)(rxbuff[4] << 8 | rxbuff[5]));
    measure->temperature = rxbuff[6];

    // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°
    if (measure->ecd - measure->last_ecd > 4096)
        measure->total_round--;
    else if (measure->ecd - measure->last_ecd < -4096)
        measure->total_round++;
    measure->total_angle = measure->total_round * 360 + measure->angle_single_round;
}

static void DJIMotorLostCallback(void *motor_ptr)
{
    /* 注意: daemon 的离线回调在"离线期间"每个 DaemonTask 周期(10ms)都会被调用一次,
     * 直接打日志会在串口上刷屏, 把 LOGINFO/CSV 一起淹掉(而且阻塞式串口发送很慢)。
     * 这里限速到 1Hz, 既能看见告警, 又不会堵住串口。 */
    static float last_log_time = 0.0f;
    float now = DWT_GetTimeline_s();
    DJIMotorInstance *motor = (DJIMotorInstance *)motor_ptr;

    if (now - last_log_time < 1.0f)
        return;
    last_log_time = now;

    LOGWARNING("[dji_motor] Motor lost, can bus [%d] , id [%d]",
               DJIMotorBusIndex(motor->motor_can_instance->can_handle) + 1, (int)motor->motor_can_instance->tx_id);
}

// 电机初始化,返回一个电机实例
DJIMotorInstance *DJIMotorInit(Motor_Init_Config_s *config)
{
    DJIMotorInstance *instance = (DJIMotorInstance *)malloc(sizeof(DJIMotorInstance));
    memset(instance, 0, sizeof(DJIMotorInstance));

    // motor basic setting 电机基本设置
    instance->motor_type = config->motor_type;                         // 6020 or 2006 or 3508
    instance->motor_settings = config->controller_setting_init_config; // 正反转,闭环类型等

    // motor controller init 电机控制器初始化
    PIDInit(&instance->motor_controller.current_PID, &config->controller_param_init_config.current_PID);
    PIDInit(&instance->motor_controller.speed_PID, &config->controller_param_init_config.speed_PID);
    PIDInit(&instance->motor_controller.angle_PID, &config->controller_param_init_config.angle_PID);
    instance->motor_controller.other_angle_feedback_ptr = config->controller_param_init_config.other_angle_feedback_ptr;
    instance->motor_controller.other_speed_feedback_ptr = config->controller_param_init_config.other_speed_feedback_ptr;
    instance->motor_controller.current_feedforward_ptr = config->controller_param_init_config.current_feedforward_ptr;
    instance->motor_controller.speed_feedforward_ptr = config->controller_param_init_config.speed_feedforward_ptr;

    // 电机分组,因为至多4个电机可以共用一帧CAN控制报文
    MotorSenderGrouping(instance, &config->can_init_config);

    // 注册电机到CAN总线
    config->can_init_config.can_module_callback = DecodeDJIMotor; // set callback
    config->can_init_config.id = instance;                        // set id,eq to address(it is identity)
    instance->motor_can_instance = CANRegister(&config->can_init_config);

    // 注册守护线程
    Daemon_Init_Config_s daemon_config = {
        .callback = DJIMotorLostCallback,
        .owner_id = instance,
        .reload_count = 2, // 20ms未收到数据则丢失
    };
    instance->daemon = DaemonRegister(&daemon_config);

    DJIMotorEnable(instance);
    dji_motor_instance[idx++] = instance;
    return instance;
}

/* 电流只能通过电机自带传感器监测,后续考虑加入力矩传感器应变片等 */
void DJIMotorChangeFeed(DJIMotorInstance *motor, Closeloop_Type_e loop, Feedback_Source_e type)
{
    if (loop == ANGLE_LOOP)
        motor->motor_settings.angle_feedback_source = type;
    else if (loop == SPEED_LOOP)
        motor->motor_settings.speed_feedback_source = type;
    else
        LOGERROR("[dji_motor] loop type error, check memory access and func param");
}

void DJIMotorStop(DJIMotorInstance *motor)
{
    motor->stop_flag = MOTOR_STOP;
    motor->runaway_latch = 0; /* 显式停止 = 人工复位失控锁存 */
}

void DJIMotorEnable(DJIMotorInstance *motor)
{
    if (motor->runaway_latch != 0)
        return; /* 失控锁存期间禁止使能, 必须先 DJIMotorStop */
    motor->stop_flag = MOTOR_ENALBED;
}

/* 修改电机的实际闭环对象 */
void DJIMotorOuterLoop(DJIMotorInstance *motor, Closeloop_Type_e outer_loop)
{
    motor->motor_settings.outer_loop_type = outer_loop;
}

// 设置参考值
void DJIMotorSetRef(DJIMotorInstance *motor, float ref)
{
    motor->motor_controller.pid_ref = ref;
}

static DJIMotorOutputHook s_output_hook = NULL;

void DJIMotorRegisterOutputHook(DJIMotorOutputHook hook)
{
    s_output_hook = hook;
}

// 为所有电机实例计算三环PID,发送控制报文
/**
 * @brief 失控保护(统一入口)
 *
 * 两类判据, 任一满足持续 300ms 就强制 STOP 并锁存(需显式 DJIMotorStop 复位):
 *   1) 指令≈0 但电流指令 ≥90% 上限   —— 环在自激/反馈异常
 *   2) 指令与转速符号相反且转速足够大 —— 反馈符号接反, 正反馈
 *
 * @param ref   速度环设定值(度/秒)
 * @param speed 实测转速(度/秒)
 */
static void DJI_RunawayGuard(DJIMotorInstance *motor, float ref, float speed)
{
    const float cmd_abs = (motor->current_cmd >= 0.0f) ? motor->current_cmd : -motor->current_cmd;
    const float out_lim = motor->motor_controller.current_PID.MaxOut;
    const float ref_abs = (ref >= 0.0f) ? ref : -ref;
    const float spd_abs = (speed >= 0.0f) ? speed : -speed;
    uint8_t bad = 0;
    const char *why = "";

    if ((ref_abs < 50.0f) && (cmd_abs >= (0.9f * out_lim)))
    {
        bad = 1;
        why = "ref=0 but current saturated";
    }
    else if ((motor->motor_settings.feedback_reverse_flag != FEEDBACK_DIRECTION_REVERSE) &&
             (ref_abs > 500.0f) && (spd_abs > 1000.0f) && ((ref * speed) < 0.0f))
    {
        /* 注意: 输出被反向的电机(等价对调相线)符号天然相反, 必须豁免本判据 */
        bad = 1;
        why = "cmd/feedback sign mismatch";
    }

    if (!bad)
    {
        motor->runaway_ms = 0.0f;
        return;
    }

    motor->runaway_ms += 1.0f; /* 控制周期 1ms */
    if (motor->runaway_ms < 300.0f)
        return;

    motor->stop_flag = MOTOR_STOP;
    motor->runaway_latch = 1;
    motor->current_cmd = 0.0f;
    motor->runaway_ms = 0.0f;
    motor->motor_controller.speed_PID.Iout = 0.0f;
    motor->motor_controller.current_PID.Iout = 0.0f;
    LOGERROR("[dji_motor] RUNAWAY: bus[%d] id[%d] %s -> FORCE STOP",
             DJIMotorBusIndex(motor->motor_can_instance->can_handle) + 1,
             (int)motor->motor_can_instance->tx_id, why);
}

void DJIMotorControl()
{
    uint8_t group, num; // 电机组号和组内编号
    int16_t set;        // 电机控制CAN发送设定值
    DJIMotorInstance *motor;
    Motor_Control_Setting_s *motor_setting; // 电机控制参数
    Motor_Controller_s *motor_controller;   // 电机控制器
    DJI_Motor_Measure_s *measure;           // 电机测量值
    float pid_measure, pid_ref;             // 电机PID测量值和设定值

    /* 第一遍: 串级PID计算, 结果先存进 current_cmd, 不立即写CAN */
    for (size_t i = 0; i < idx; ++i)
    {
        motor = dji_motor_instance[i];
        motor_setting = &motor->motor_settings;
        motor_controller = &motor->motor_controller;
        measure = &motor->measure;
        /* ★ 安全保护: 反馈丢失时绝不能让 PID 继续输出。
         * 原因: 丢反馈后 speed 环误差恒为设定值, PID 会饱和成满电流;
         *       而 C620 收不到指令时会【保持最后一次电流】, 电机会一直转, 关遥控也没用。
         * 这里强制清零积分并把 current_cmd 置 0, 下一段会把 0 显式发到总线上。 */
        if ((motor->daemon != NULL) && (DaemonIsOnline(motor->daemon) == 0))
        {
            motor_controller->angle_PID.Iout = 0.0f;
            motor_controller->speed_PID.Iout = 0.0f;
            motor_controller->current_PID.Iout = 0.0f;
            motor->current_cmd = 0.0f;
            continue;
        }

        pid_ref = motor_controller->pid_ref;
        if (motor_setting->motor_reverse_flag == MOTOR_DIRECTION_REVERSE)
            pid_ref *= -1;

        // 计算位置环
        if ((motor_setting->close_loop_type & ANGLE_LOOP) && motor_setting->outer_loop_type == ANGLE_LOOP)
        {
            if (motor_setting->angle_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_angle_feedback_ptr;
            else
                pid_measure = measure->total_angle;
            pid_ref = PIDCalculate(&motor_controller->angle_PID, pid_measure, pid_ref);
        }

        // 计算速度环
        if ((motor_setting->close_loop_type & SPEED_LOOP) && (motor_setting->outer_loop_type & (ANGLE_LOOP | SPEED_LOOP)))
        {
            if (motor_setting->feedforward_flag & SPEED_FEEDFORWARD)
                pid_ref += *motor_controller->speed_feedforward_ptr;

            if (motor_setting->speed_feedback_source == OTHER_FEED)
                pid_measure = *motor_controller->other_speed_feedback_ptr;
            else
                pid_measure = measure->speed_aps;
            /* 方向映射(参考 mc-02 output_axis.direction):
             * 只翻【测量】和【输出】, 目标保持应用层符号。
             * I = -Kp(r - (-s)) = -Kp(r + s), 极点 1+kKp>0 恒稳定, 且转向相反。 */
            if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
                pid_measure = -pid_measure;
            pid_ref = PIDCalculate(&motor_controller->speed_PID, pid_measure, pid_ref);
        }

        // 计算电流环
        if (motor_setting->feedforward_flag & CURRENT_FEEDFORWARD)
            pid_ref += *motor_controller->current_feedforward_ptr;
        if (motor_setting->close_loop_type & CURRENT_LOOP)
        {
            /* 电流是"设备侧"量, 不做方向映射(文档: 直接电流/力矩输出不由轴方向换算) */
            pid_ref = PIDCalculate(&motor_controller->current_PID, measure->real_current, pid_ref);
        }

        /* 把输出轴闭环结果映回驱动方向(与上面的"测量取反"配对) */
        if (motor_setting->feedback_reverse_flag == FEEDBACK_DIRECTION_REVERSE)
            pid_ref *= -1;

        motor->current_cmd = pid_ref;

        DJI_RunawayGuard(motor, pid_ref, measure->speed_aps);
    }

    /* 钩子: 功率限制在这里统一改写各电机的 current_cmd */
    if (s_output_hook != NULL)
        s_output_hook();

    /* 第二遍: 把(可能被改写过的)电流指令写进CAN发送缓冲 */
    for (size_t i = 0; i < idx; ++i)
    {
        motor = dji_motor_instance[i];
        float cmd = motor->current_cmd;

        if (cmd > 16384.0f)
            cmd = 16384.0f;
        if (cmd < -16384.0f)
            cmd = -16384.0f;
        set = (int16_t)cmd;

        group = motor->sender_group;
        num = motor->message_num;

        if (motor->stop_flag == MOTOR_STOP)
        {
            /* 停止时该电机对应的两个字节清零, 不影响同组其他电机 */
            sender_assignment[group].tx_buff[2 * num] = 0;
            sender_assignment[group].tx_buff[2 * num + 1] = 0;
        }
        else
        {
            sender_assignment[group].tx_buff[2 * num] = (uint8_t)(set >> 8);         // 高八位
            sender_assignment[group].tx_buff[2 * num + 1] = (uint8_t)(set & 0x00ff); // 低八位
        }
    }

    // 遍历flag,检查是否要发送这一帧报文
    for (size_t i = 0; i < DEVICE_CAN_CNT * 3; ++i)
    {
        if (sender_enable_flag[i])
        {
            CANTransmit(&sender_assignment[i], 1);
        }
    }
}
