#include "chassis_omni.h"
#include "bsp_log.h"
#include "daemon.h"
#include "remote_control.h"

static DJIMotorInstance *omni_motor[OMNI_WHEEL_CNT];
static const uint8_t omni_id[OMNI_WHEEL_CNT] = {OMNI_ID_FRONT, OMNI_ID_REAR, OMNI_ID_LEFT, OMNI_ID_RIGHT};
static const int8_t omni_rev[OMNI_WHEEL_CNT] = {OMNI_REV_FRONT, OMNI_REV_REAR, OMNI_REV_LEFT, OMNI_REV_RIGHT};

static float omni_ref_rpm[OMNI_WHEEL_CNT];
static float omni_vx, omni_vy, omni_omega;

/* 底盘速度 -> 轮线速度(m/s) */
static void OmniInverse(float vx, float vy, float omega, float *wheel)
{
    const float a = 0.5f * OMNI_WHEELBASE;
    const float b = 0.5f * OMNI_TRACK;

    wheel[OMNI_FRONT] = vy + a * omega;
    wheel[OMNI_REAR] = -vy + a * omega;
    wheel[OMNI_LEFT] = -vx + b * omega;
    wheel[OMNI_RIGHT] = vx + b * omega;
}

/* 轮线速度(m/s) -> 底盘速度 */
static void OmniForward(const float *wheel, float *vx, float *vy, float *omega)
{
    const float a = 0.5f * OMNI_WHEELBASE;
    const float b = 0.5f * OMNI_TRACK;

    *vx = 0.5f * (wheel[OMNI_RIGHT] - wheel[OMNI_LEFT]);
    *vy = 0.5f * (wheel[OMNI_FRONT] - wheel[OMNI_REAR]);
    *omega = 0.25f * ((wheel[OMNI_FRONT] + wheel[OMNI_REAR]) / a + (wheel[OMNI_LEFT] + wheel[OMNI_RIGHT]) / b);
}

/* m/s -> 转子 rpm */
static float OmniLinearToRpm(float v)
{
    return v / (2.0f * 3.14159265f * OMNI_WHEEL_R) * 60.0f * OMNI_RATIO;
}

void ChassisOmniInit(void)
{
    Motor_Init_Config_s cfg = {
        .can_init_config.can_handle = OMNI_CAN_BUS,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = OMNI_SPEED_KP,
                .Ki = OMNI_SPEED_KI,
                .Kd = OMNI_SPEED_KD,
                .IntegralLimit = OMNI_SPEED_MAX_IOUT,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = OMNI_SPEED_MAX_OUT,
            },
            .current_PID = {
                .Kp = 0.5f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .IntegralLimit = 3000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 15000.0f,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
        },
        .motor_type = M3508,
    };

    for (uint8_t i = 0; i < OMNI_WHEEL_CNT; ++i)
    {
        /* DJIMotorInit() 会把算出的 rx_id 写回 config, 复用结构体时必须清零,
         * 否则第二个电机会带着上一个的 rx_id 注册 -> 触发 ID 冲突保护 */
        cfg.can_init_config.rx_id = 0;
        cfg.can_init_config.tx_id = omni_id[i];
        cfg.controller_setting_init_config.motor_reverse_flag =
            (omni_rev[i] < 0) ? MOTOR_DIRECTION_REVERSE : MOTOR_DIRECTION_NORMAL;
        cfg.controller_setting_init_config.feedback_reverse_flag =
            (omni_rev[i] < 0) ? FEEDBACK_DIRECTION_REVERSE : FEEDBACK_DIRECTION_NORMAL;

        omni_motor[i] = DJIMotorInit(&cfg);
        omni_ref_rpm[i] = 0.0f;
    }

    LOGINFO("[omni] 4-wheel chassis init done (CAN1, id %d-%d)", OMNI_ID_FRONT, OMNI_ID_RIGHT);
}

void ChassisOmniSetVelocity(float vx, float vy, float omega)
{
    float wheel[OMNI_WHEEL_CNT];
    float vmax = 0.0f;

    LIMIT_MIN_MAX(vx, -OMNI_MAX_VX, OMNI_MAX_VX);
    LIMIT_MIN_MAX(vy, -OMNI_MAX_VY, OMNI_MAX_VY);
    LIMIT_MIN_MAX(omega, -OMNI_MAX_W, OMNI_MAX_W);

    omni_vx = vx;
    omni_vy = vy;
    omni_omega = omega;

    OmniInverse(vx, vy, omega, wheel);

    for (uint8_t i = 0; i < OMNI_WHEEL_CNT; ++i)
    {
        float v = wheel[i] < 0.0f ? -wheel[i] : wheel[i];
        if (v > vmax)
            vmax = v;
    }

    /* 超出上限按比例缩放, 保持运动方向 */
    const float scale = (vmax > OMNI_MAX_VX) ? (OMNI_MAX_VX / vmax) : 1.0f;

    for (uint8_t i = 0; i < OMNI_WHEEL_CNT; ++i)
        omni_ref_rpm[i] = OmniLinearToRpm(wheel[i] * scale);
}

void ChassisOmniGetVelocity(float *vx, float *vy, float *omega)
{
    float wheel[OMNI_WHEEL_CNT];

    for (uint8_t i = 0; i < OMNI_WHEEL_CNT; ++i)
    {
        /* speed_aps: 转子 deg/s -> 轮线速度 m/s */
        const float rpm = omni_motor[i]->measure.speed_aps / 6.0f;
        wheel[i] = rpm / OMNI_RATIO / 60.0f * (2.0f * 3.14159265f * OMNI_WHEEL_R);
    }

    OmniForward(wheel, vx, vy, omega);
}

void ChassisOmniTask(void)
{
#if OMNI_RC_ENABLE
    extern RC_ctrl_t *rc_ctrl;

    if (rc_ctrl != NULL)
    {
        const int16_t dead = OMNI_RC_DEAD;
        float vx = 0.0f, vy = 0.0f, w = 0.0f;

        int16_t ch = (int16_t)rc_ctrl->rc.ch[OMNI_RC_VX_CH] - 1024;
        if (ch > dead || ch < -dead)
            vx = (float)ch / 660.0f * OMNI_MAX_VX;

        ch = (int16_t)rc_ctrl->rc.ch[OMNI_RC_VY_CH] - 1024;
        if (ch > dead || ch < -dead)
            vy = -(float)ch / 660.0f * OMNI_MAX_VY;

        ch = (int16_t)rc_ctrl->rc.ch[OMNI_RC_W_CH] - 1024;
        if (ch > dead || ch < -dead)
            w = (float)ch / 660.0f * OMNI_MAX_W;

        ChassisOmniSetVelocity(vx, vy, w);
    }
#endif

    /* 功率限幅: 按 24V 估算, 超限时整体降额 */
    if (OMNI_POWER_MAX > 0.0f)
    {
        float power = 0.0f;
        for (uint8_t i = 0; i < OMNI_WHEEL_CNT; ++i)
        {
            const float cur = omni_motor[i]->measure.real_current;
            power += (cur < 0.0f ? -cur : cur) * OMNI_VOLTAGE * 0.001f;
        }

        if (power > OMNI_POWER_MAX)
        {
            const float k = OMNI_POWER_MAX / power;
            for (uint8_t i = 0; i < OMNI_WHEEL_CNT; ++i)
                omni_ref_rpm[i] *= k;
        }
    }

    for (uint8_t i = 0; i < OMNI_WHEEL_CNT; ++i)
        DJIMotorSetRef(omni_motor[i], omni_ref_rpm[i] * (float)omni_rev[i]);
}

void ChassisOmniStop(void)
{
    for (uint8_t i = 0; i < OMNI_WHEEL_CNT; ++i)
    {
        omni_ref_rpm[i] = 0.0f;
        DJIMotorSetRef(omni_motor[i], 0.0f);
    }
}

uint8_t ChassisOmniIsOnline(void)
{
    for (uint8_t i = 0; i < OMNI_WHEEL_CNT; ++i)
    {
        if (omni_motor[i] == NULL || omni_motor[i]->stop_flag == MOTOR_STOP)
            return 0;
    }
    return 1;
}
