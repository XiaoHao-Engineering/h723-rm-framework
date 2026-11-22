/**
 * @file gimbal.c
 * @brief 云台应用: 单 Yaw + 双 Pitch(大P/小P 串联), 三台达妙电机
 *
 * 控制结构(三台都是 MIT 力矩模式, kp=kd=0, 位置环在 MCU 里跑):
 *
 *   Yaw     : 角度环(IMU YawTotalAngle) -> 速度环(IMU Gyro[z]) -> t_ff
 *   大 Pitch: 角度环(电机编码器)        -> 速度环(编码器)      -> t_ff + 重力前馈
 *   小 Pitch: 角度环(IMU Pitch)         -> 速度环(IMU Gyro[x]) -> t_ff + 重力前馈
 *
 * 为什么大P 用编码器、小P 用 IMU:
 *   IMU 贴在发射机构上(小P 输出端), 所以 IMU 的 Pitch 就是【炮管真实绝对仰角】,
 *   弹道要的就是这个量 -> 小P 直接用 IMU 闭环, 精度最高。
 *   大P 没有绝对基准, 就用自己的编码器闭环, 负责"粗调/抬臂", 刚度给低一点,
 *   让小P 主导最后的仰角精度(粗精分离)。两者目标都是同一个 pitch 指令:
 *   大P 到位时 小P 自然回到中位。
 *
 * 为什么不给 kp>0(让电机内部闭环):
 *   位置反馈用的是 IMU, 只能由 MCU 闭环; 而且 dm_motor 的 kp>0 分支会把 t_ff 写死成 0,
 *   重力前馈就进不去了。所以三轴统一走"kp=kd=0 + 框架三环 + t_ff"。
 */

#include "remote_control.h"
#include "gimbal.h"
#include "robot_def.h"
#include "rm_config.h"

#include "dm_motor.h"
#include "ins_task.h"
#include "message_center.h"
#include "general_def.h"
#include "bsp_board.h"
#include "bsp_dwt.h"

#include <math.h>

static attitude_t *gimba_IMU_data; // 云台IMU数据

static DMMotorInstance *yaw_motor, *bigp_motor, *smallp_motor;

static Publisher_t *gimbal_pub;                   // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                  // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data; // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;         // 来自cmd的控制信息

/* 重力前馈的影子量, 通过 current_feedforward_ptr 挂到电机控制器上 */
static float ff_big_torque = 0.0f;
static float ff_small_torque = 0.0f;

/* 私有小工具 */
static float ClampF(float x, float lo, float hi)
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

/* 0~360 归一化 */
static float Wrap360(float deg)
{
    while (deg >= 360.0f)
        deg -= 360.0f;
    while (deg < 0.0f)
        deg += 360.0f;
    return deg;
}

/**
 * @brief 按"静态重力矩"算前馈: tau = m * g * L * cos(theta)
 *        theta 用 IMU 的绝对 pitch, 小P 的 ±15° 行程对 cos 影响很小, 忽略。
 */
static void UpdateGravityFeedforward(void)
{
#if GRAV_FF_ENABLE
    const float pitch_rad = gimba_IMU_data->Pitch * DEGREE_2_RAD;
    const float c = cosf(pitch_rad);

    ff_small_torque = GRAV_FF_SIGN_SMALL * SMALLP_LOAD_MASS * G_ACC * SMALLP_LOAD_ARM * c;
    ff_small_torque = ClampF(ff_small_torque, -GRAV_FF_MAX_SMALL, GRAV_FF_MAX_SMALL);

    ff_big_torque = GRAV_FF_SIGN_BIG * (SMALLP_LOAD_MASS + BIGP_EXTRA_MASS) * G_ACC * BIGP_LOAD_ARM * c;
    ff_big_torque = ClampF(ff_big_torque, -GRAV_FF_MAX_BIG, GRAV_FF_MAX_BIG);
#else
    ff_small_torque = 0.0f;
    ff_big_torque = 0.0f;
#endif
}

/**
 * @brief 填一份达妙云台电机的公共配置(三轴只差反馈来源与 PID 参数)
 */
static void FillGimbalMotorConfig(DMMotor_Init_Config_s *cfg, uint32_t id, uint8_t use_imu,
                                  float a_kp, float a_ki, float a_max, float a_db,
                                  float s_kp, float s_ki, float s_max, float s_ilim,
                                  float *ff_ptr)
{
    memset(cfg, 0, sizeof(*cfg));

    cfg->can_init_config.can_handle = RM_CAN_GIMBAL;
    cfg->can_init_config.rx_id = DM_RX_ID_AUTO; /* 反馈帧 ID 自动 = motor_id */
    cfg->can_init_config.nominal_bitrate = 1000000;

    cfg->motor_id = id;
    cfg->kp = 0.0f; /* 0 = 力矩模式, 位置环在 MCU 里跑 */
    cfg->kd = 0.0f;
    cfg->ctrl_mode = DM_MODE_MIT;

    cfg->range.p_min = -DM_PMAX;
    cfg->range.p_max = DM_PMAX;
    cfg->range.v_min = -DM_VMAX;
    cfg->range.v_max = DM_VMAX;
    cfg->range.kp_min = DM_DEFAULT_KP_MIN;
    cfg->range.kp_max = DM_DEFAULT_KP_MAX;
    cfg->range.kd_min = DM_DEFAULT_KD_MIN;
    cfg->range.kd_max = DM_DEFAULT_KD_MAX;
    cfg->range.t_min = -DM_TMAX;
    cfg->range.t_max = DM_TMAX;

    cfg->controller_param_init_config.angle_PID = (PID_Init_Config_s){
        .Kp = a_kp,
        .Ki = a_ki,
        .Kd = 0.0f,
        .DeadBand = a_db,
        .IntegralLimit = 100.0f,
        .MaxOut = a_max,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    };
    cfg->controller_param_init_config.speed_PID = (PID_Init_Config_s){
        .Kp = s_kp,
        .Ki = s_ki,
        .Kd = 0.0f,
        .IntegralLimit = s_ilim,
        .MaxOut = s_max, /* ★ 单位是 N·m */
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
    };

    cfg->controller_setting_init_config.outer_loop_type = ANGLE_LOOP;
    cfg->controller_setting_init_config.close_loop_type = ANGLE_LOOP | SPEED_LOOP;
    cfg->controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    cfg->controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL;

    if (use_imu)
    {
        cfg->controller_setting_init_config.angle_feedback_source = OTHER_FEED;
        cfg->controller_setting_init_config.speed_feedback_source = OTHER_FEED;
    }
    else
    {
        cfg->controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
        cfg->controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
    }

    /* 重力前馈: 速度环输出之后再叠加, 所以关闭电流环, 直接进 t_ff */
    if (ff_ptr != NULL)
    {
        cfg->controller_setting_init_config.feedforward_flag = CURRENT_FEEDFORWARD;
        cfg->controller_param_init_config.current_feedforward_ptr = ff_ptr;
    }
    else
    {
        cfg->controller_setting_init_config.feedforward_flag = FEEDFORWARD_NONE;
    }
}

void GimbalInit()
{
    DMMotor_Init_Config_s cfg;

    gimba_IMU_data = INS_Init(); // 已初始化过会直接返回同一个指针

    /* ---------------- Yaw ---------------- */
    FillGimbalMotorConfig(&cfg, YAW_CAN_ID, 1,
                          YAW_ANGLE_KP, YAW_ANGLE_KI, YAW_ANGLE_MAXOUT, YAW_ANGLE_DEADBAND,
                          YAW_SPEED_KP, YAW_SPEED_KI, YAW_SPEED_MAXOUT, YAW_SPEED_I_LIMIT,
                          NULL);
    cfg.controller_param_init_config.other_angle_feedback_ptr = &gimba_IMU_data->YawTotalAngle;
    cfg.controller_param_init_config.other_speed_feedback_ptr = &gimba_IMU_data->Gyro[2];
    yaw_motor = DMMotorInit(&cfg);

    /* ---------------- 大 Pitch(编码器闭环 + 重力前馈) ---------------- */
    FillGimbalMotorConfig(&cfg, BIGP_CAN_ID, 0,
                          BIGP_ANGLE_KP, BIGP_ANGLE_KI, BIGP_ANGLE_MAXOUT, BIGP_ANGLE_DEADBAND,
                          BIGP_SPEED_KP, BIGP_SPEED_KI, BIGP_SPEED_MAXOUT, BIGP_SPEED_I_LIMIT,
                          &ff_big_torque);
    bigp_motor = DMMotorInit(&cfg);

    /* ---------------- 小 Pitch(IMU 绝对仰角闭环 + 重力前馈) ---------------- */
    FillGimbalMotorConfig(&cfg, SMALLP_CAN_ID, 1,
                          SMALLP_ANGLE_KP, SMALLP_ANGLE_KI, SMALLP_ANGLE_MAXOUT, SMALLP_ANGLE_DEADBAND,
                          SMALLP_SPEED_KP, SMALLP_SPEED_KI, SMALLP_SPEED_MAXOUT, SMALLP_SPEED_I_LIMIT,
                          &ff_small_torque);
    cfg.controller_param_init_config.other_angle_feedback_ptr = &gimba_IMU_data->Pitch;
    cfg.controller_param_init_config.other_speed_feedback_ptr = &gimba_IMU_data->Gyro[0];
    smallp_motor = DMMotorInit(&cfg);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));

    /* 上电默认给个水平, 避免第一次收到指令时阶跃 */
    gimbal_cmd_recv.yaw = gimba_IMU_data->YawTotalAngle;
    gimbal_cmd_recv.pitch = 0.0f;

    LOGINFO("[gimbal] 3x DM4310 registered on CAN2 (id %d/%d/%d)",
            YAW_CAN_ID, BIGP_CAN_ID, SMALLP_CAN_ID);
}

/* 使能状态: 只在状态切换时发使能帧。
 * 如果每 10ms 都发一次 0xFC, 三台电机就是 300 帧/秒的无效流量, 会把 CAN 挤满。 */
static uint8_t gimbal_enabled = 0;

/* 机器人云台控制核心任务 */
void GimbalTask()
{
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);

    /* 不管什么模式, 前馈都按当前姿态算好(失能时值为 0 无所谓) */
    UpdateGravityFeedforward();

    switch (gimbal_cmd_recv.gimbal_mode)
    {
    case GIMBAL_ZERO_FORCE:
        if (gimbal_enabled)
        {
            DMMotorDisable(yaw_motor);
            DMMotorDisable(bigp_motor);
            DMMotorDisable(smallp_motor);
            gimbal_enabled = 0;
        }
        break;

    case GIMBAL_GYRO_MODE:
    case GIMBAL_FREE_MODE:
    default:
        if (!gimbal_enabled)
        {
            DMMotorEnable(yaw_motor);
            DMMotorEnable(bigp_motor);
            DMMotorEnable(smallp_motor);
            gimbal_enabled = 1;
        }

        /* Yaw: 指令就是 IMU 总角度(度), 上电已对齐 */
        DMMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);

        /* Pitch: 指令是"炮管绝对仰角(度)" */
        {
            float pitch_cmd = ClampF(gimbal_cmd_recv.pitch, PITCH_CMD_MIN_DEG, PITCH_CMD_MAX_DEG);

#if MINI_PITCH_MIT_ENABLE
            /* 上电/掉线保护: 遥控不在线时目标 = 当前角度, 避免 MIT 内部环把炮管拽到 0 度 */
            if (!RemoteControlIsOnline())
                pitch_cmd = smallp_motor->measure.position_deg;
#endif

            /* 小 P: IMU 域, 直接追绝对仰角 */
#if MINI_PITCH_MIT_ENABLE
            /* 小 P: 电机内部 MIT 位置环(kp/kd) + 重力前馈 -> 响应快、刚性好 */
            DMMotorSetMITCmd(smallp_motor, pitch_cmd, MINI_PITCH_KP, MINI_PITCH_KD,
                             ff_small_torque);
#else
            /* 小 P: 力矩模式, 框架三环 + 前馈 */
            DMMotorSetRef(smallp_motor, pitch_cmd);
#endif

            /* 大 P: 换到自己的编码器域; 到位时 小P 自然回中 */
            float bigp_cmd = ClampF(pitch_cmd - BIGP_ALIGN_ABS_DEG, BIGP_MIN_DEG, BIGP_MAX_DEG);
            DMMotorSetRef(bigp_motor, bigp_cmd);
        }
        break;
    }

    /* 软件限位回写: 让上层知道实际被夹到哪了, 免得指令一直往上堆 */
    {
        static float last_warn = 0.0f;
        const float now = DWT_GetTimeline_ms();

        float pitch_abs = gimba_IMU_data->Pitch;
        float bigp_now = bigp_motor->measure.position_deg;
        float smallp_now = smallp_motor->measure.position_deg;

        /* 限位告警限速 1Hz, 否则顶到限位时会 100Hz 刷屏把日志冲掉 */
        if ((now - last_warn) > 1000.0f)
        {
            last_warn = now;
            if (bigp_now > BIGP_MAX_DEG || bigp_now < BIGP_MIN_DEG)
                LOGWARNING("[gimbal] bigP out of range"); /* 日志模块不支持 %f */
            else if (smallp_now > SMALLP_MAX_DEG || smallp_now < SMALLP_MIN_DEG)
                LOGWARNING("[gimbal] smallP out of range");
        }

        /* 设置反馈数据 */
        gimbal_feedback_data.gimbal_imu_data = *gimba_IMU_data;
        gimbal_feedback_data.yaw_relative_deg = Wrap360(yaw_motor->measure.position_deg - YAW_MOTOR_ALIGN_DEG);
        gimbal_feedback_data.pitch_abs_deg = pitch_abs;
        gimbal_feedback_data.bigp_deg = bigp_now;
        gimbal_feedback_data.smallp_deg = smallp_now;
    }

    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}
