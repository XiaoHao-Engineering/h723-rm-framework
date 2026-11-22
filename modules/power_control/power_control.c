#include "power_control.h"
#include "controller.h"

#include <math.h>
#include <string.h>

#define PC_MAX_MOTORS 8
#define CMD_LIMIT 16000.0f
#define DEFAULT_CT 1.99688994e-6f

static PowerControl_Config_s s_cfg;
static PIDInstance s_buffer_pid;

static float s_budget = 0.0f;    // 电机侧总预算
static float s_total = 0.0f;     // 上一次的正功率之和
static float s_scale = 1.0f;     // 上一次的缩放系数
static float s_sim_buffer = 60;  // 台架模拟的缓冲能量
static uint8_t s_inited = 0;

/* ---------------------------------------------------------------- 工具 */
static float clampf(float v, float lo, float hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

/* aps(度/秒) → rpm */
static float aps_to_rpm(float aps)
{
    return aps / 6.0f;
}

/* 正向模型：给定转速与电流指令，算输入功率 */
static float pc_forward(const PowerModel_s *m, float w, float i)
{
    return m->ct * i * w + m->k1 * w * w + m->k2 * i * i + m->a;
}

/* 反向：给定目标功率与转速，按原电流方向解出电流指令
 * 返回 0 表示无解（此时外面会把电流压到 0） */
static int pc_inverse(const PowerModel_s *m, float w, float p, float i0, float *out)
{
    float b = m->ct * w;
    float c = m->k1 * w * w + m->a - p;
    float disc, sq, r;

    if (m->k2 <= 0.0f)
    {
        /* 参数异常时退化为线性近似，避免除零 */
        *out = (m->ct * w != 0.0f) ? (p - c) / b : 0.0f;
        return 1;
    }

    disc = b * b - 4.0f * m->k2 * c;
    if (disc < 0.0f)
    {
        *out = 0.0f;
        return 0;
    }

    sq = sqrtf(disc);
    if (i0 >= 0.0f)
        r = (-b + sq) / (2.0f * m->k2);
    else
        r = (-b - sq) / (2.0f * m->k2);

    *out = clampf(r, -CMD_LIMIT, CMD_LIMIT);
    return 1;
}

/* ---------------------------------------------------------------- 初始化 */
void PowerControlInit(PowerControl_Config_s *cfg)
{
    memset(&s_cfg, 0, sizeof(s_cfg));
    s_cfg = *cfg;

    if (s_cfg.motor_cnt > PC_MAX_MOTORS)
        s_cfg.motor_cnt = PC_MAX_MOTORS;
    if (s_cfg.motor_cnt == 0 || s_cfg.motors == NULL)
        return;

    if (s_cfg.model.ct <= 0.0f)
        s_cfg.model.ct = DEFAULT_CT;
    if (s_cfg.model.k1 <= 0.0f)
        s_cfg.model.k1 = 1.23e-7f;
    if (s_cfg.model.k2 <= 0.0f)
        s_cfg.model.k2 = 1.453e-7f;
    if (s_cfg.model.a <= 0.0f)
        s_cfg.model.a = 4.081f;
    if (s_cfg.default_limit_w <= 1.0f)
        s_cfg.default_limit_w = 45.0f;
    if (s_cfg.buffer_target_j <= 0.0f)
        s_cfg.buffer_target_j = 30.0f;
    if (s_cfg.buffer_kp <= 0.0f)
        s_cfg.buffer_kp = 3.0f;
    if (s_cfg.buffer_ki < 0.0f)
        s_cfg.buffer_ki = 0.05f;

    PID_Init_Config_s pid_cfg = {
        .Kp = s_cfg.buffer_kp,
        .Ki = s_cfg.buffer_ki,
        .Kd = 0.0f,
        .MaxOut = 40.0f,
        .DeadBand = 0.5f,
        .Improve = PID_Integral_Limit,
        .IntegralLimit = 40.0f,
    };
    PIDInit(&s_buffer_pid, &pid_cfg);

    s_budget = s_cfg.default_limit_w;
    s_total = 0.0f;
    s_scale = 1.0f;
    s_sim_buffer = 60.0f;
    s_inited = 1;

    /* 挂到电机输出钩子上：1 kHz，在所有电机 PID 之后、发 CAN 之前 */
    DJIMotorRegisterOutputHook(PowerControlApply);
}

/* ---------------------------------------------------------------- 预算（100 Hz） */
void PowerControlUpdate(float chassis_power, float buffer, float limit_w)
{
    float out, battery, e;

    if (!s_inited)
        return;

    if (limit_w <= 1.0f)
        limit_w = s_cfg.default_limit_w;

    if (s_cfg.use_referee)
    {
        e = buffer;
    }
    else
    {
        /* 台架模拟：缓冲能量 = ∫(上限 − 实测功率) dt，夹在 0~60 J */
        s_sim_buffer += (limit_w - s_total) * 0.01f;
        s_sim_buffer = clampf(s_sim_buffer, 0.0f, 60.0f);
        e = s_sim_buffer;
    }
    (void)chassis_power;

    /* 误差 = 目标 − 实测；缓冲低 → 输出正 → 预算收紧 */
    out = PIDCalculate(&s_buffer_pid, e, s_cfg.buffer_target_j);

    battery = limit_w - out;
    battery = clampf(battery, 0.0f, limit_w * 1.5f);

    s_budget = battery + s_cfg.cap_boost_w;
    if (s_budget < 0.0f)
        s_budget = 0.0f;
}

/* ---------------------------------------------------------------- 分配（1 kHz） */
void PowerControlApply(void)
{
    float p[PC_MAX_MOTORS];
    float sum = 0.0f;
    float k;
    uint8_t i;

    if (!s_inited)
        return;

    /* 1. 正向算出每台电机的功率 */
    for (i = 0; i < s_cfg.motor_cnt; i++)
    {
        DJIMotorInstance *m = s_cfg.motors[i];
        float w, cur;

        if (m == NULL)
        {
            p[i] = 0.0f;
            continue;
        }
        w = aps_to_rpm(m->measure.speed_aps);
        cur = m->current_cmd;
        p[i] = pc_forward(&s_cfg.model, w, cur);
        if (p[i] > 0.0f)
            sum += p[i];
    }

    s_total = sum;
    s_scale = 1.0f;

    /* 预算为 0 直接全停 */
    if (s_budget <= 0.0f)
    {
        for (i = 0; i < s_cfg.motor_cnt; i++)
            if (s_cfg.motors[i] != NULL)
                s_cfg.motors[i]->current_cmd = 0.0f;
        return;
    }

    /* 2. 没超预算就不动 */
    if (sum <= s_budget)
        return;

    /* 3. 等比例缩放，再逐台反解 */
    k = s_budget / sum;
    s_scale = k;

    for (i = 0; i < s_cfg.motor_cnt; i++)
    {
        DJIMotorInstance *m = s_cfg.motors[i];
        float w, cur, target, out;

        if (m == NULL)
            continue;
        if (p[i] <= 0.0f)
            continue; /* 负功率（回馈）不缩放 */

        w = aps_to_rpm(m->measure.speed_aps);
        cur = m->current_cmd;
        target = p[i] * k;

        if (!pc_inverse(&s_cfg.model, w, target, cur, &out))
        {
            /* 无解：目标功率低于该转速下的损耗地板，压到 0（原工程这里是 continue） */
            out = 0.0f;
        }
        m->current_cmd = out;
    }
}

/* ---------------------------------------------------------------- 状态查询 */
float PowerControlGetBudget(void)
{
    return s_budget;
}

float PowerControlGetTotalPower(void)
{
    return s_total;
}

float PowerControlGetScale(void)
{
    return s_scale;
}

void PowerControlSetCapBoost(float w)
{
    s_cfg.cap_boost_w = w;
}
