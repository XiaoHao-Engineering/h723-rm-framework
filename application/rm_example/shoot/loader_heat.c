/**
 * @file loader_heat.c
 * @brief 发射热量管理：状态机估计（数）+ 前馈规划（算）+ 硬限幅（兜底）
 *
 * 规则依据（2027 高校联盟赛 V1.0.0）：
 *   - 每发 17mm +10、42mm +100；裁判系统 10Hz(100ms) 结算冷却
 *   - 超过上限后必须降到 0 才解锁 -> 所以限幅要提前，不能等撞线
 *
 * 默认参数（冷却优先）：Q0=88、a=24/s、d=10 -> 稳态射频 a/d = 2.4 发/s
 * 切爆发优先：rm_config.h 里 SHOOT_HEAT_COOLING_FIRST 改 0（230 / 14）
 */

#include "loader_heat.h"

#include "rm_config.h"
#include "bsp_log.h"
#include <math.h>

/* ---------------- 内部状态 ---------------- */
static float lh_q;        /* 热量估计 */
static float lh_limit;    /* Q0 */
static float lh_cool;     /* a (/s) */
static float lh_per_shot; /* d */

static float lh_last_ms;
static uint8_t lh_inited;

/* 输入 */
static float lh_aps;      /* 摩擦轮转速(deg/s)，取两轮较大值 */
static float lh_amp;      /* 摩擦轮电流(A)，取两轮较大值 */
static float lh_cmd_rps;  /* 连发指令射频(发/s) */

/* 命令->确认 的账 */
static float lh_cmd_accum;   /* 连发指令累计到下一发 */
static float lh_pending;     /* 已下令未确认的发数 */
static float lh_pending_ms;  /* pending 开始计时 */

/* 状态机 */
static loader_heat_state_e lh_state;
static uint8_t lh_armed;     /* 电流已回落到阈值以下，允许下一次触发 */
static float lh_suspect_ms;

/* 输出 */
static float lh_allowed_rps;
static uint8_t lh_single_only;
static uint8_t lh_blocked;

/* 统计 */
static uint32_t lh_confirmed;
static uint32_t lh_missed;
static uint8_t lh_warn_print;

/* ---------------- 内部工具 ---------------- */
static float lh_abs(float x)
{
    return (x < 0.0f) ? -x : x;
}

static void lh_add_heat(const char *why)
{
    lh_q += lh_per_shot;
    if (lh_q > lh_limit + 50.0f)
        lh_q = lh_limit + 50.0f; /* 估计值封顶，别无限涨 */

    if (lh_q > lh_limit)
        LOGWARNING("[heat] Q=%.1f > limit %.0f (%s)", lh_q, lh_limit, why);
}

/* ---------------- 初始化 ---------------- */
void LoaderHeatInit(void)
{
#if SHOOT_HEAT_COOLING_FIRST
    lh_limit = SHOOT_HEAT_LIMIT_COOL;
    lh_cool = SHOOT_HEAT_COOL_RPS_COOL;
#else
    lh_limit = SHOOT_HEAT_LIMIT_BURST;
    lh_cool = SHOOT_HEAT_COOL_RPS_BURST;
#endif
    lh_per_shot = SHOOT_HEAT_PER_SHOT;

    lh_q = 0.0f;
    lh_last_ms = 0.0f;
    lh_inited = 0;

    lh_aps = 0.0f;
    lh_amp = 0.0f;
    lh_cmd_rps = 0.0f;

    lh_cmd_accum = 0.0f;
    lh_pending = 0.0f;
    lh_pending_ms = 0.0f;

    lh_state = LOADER_HEAT_IDLE;
    lh_armed = 0;
    lh_suspect_ms = 0.0f;

    lh_allowed_rps = 0.0f;
    lh_single_only = 0;
    lh_blocked = 0;
    lh_confirmed = 0;
    lh_missed = 0;
    lh_warn_print = 0;

    LOGINFO("[heat] init: Q0=%.0f a=%.0f/s d=%.0f (%s)", lh_limit, lh_cool,
            lh_per_shot, SHOOT_HEAT_COOLING_FIRST ? "cooling-first" : "burst-first");
}

/* ---------------- 输入 ---------------- */
void LoaderHeatFeedFriction(float aps_l, float aps_r, float current_a)
{
    float a_l = lh_abs(aps_l);
    float a_r = lh_abs(aps_r);

    lh_aps = (a_l > a_r) ? a_l : a_r;
    lh_amp = lh_abs(current_a);
}

void LoaderHeatFeedCommand(float cmd_rps)
{
    lh_cmd_rps = (cmd_rps > 0.0f) ? cmd_rps : 0.0f;
}

void LoaderHeatOnSingleShot(void)
{
    lh_pending += 1.0f;
    if (lh_pending_ms <= 0.0f)
        lh_pending_ms = 0.001f; /* 启动超时计时 */
}

/* ---------------- 状态机（底层：数） ---------------- */
static void lh_update_fsm(float dt)
{
    const uint8_t spun_up = (lh_aps > SHOOT_HEAT_FRICTION_READY_APS) ? 1u : 0u;

#if !SHOOT_HEAT_BENCH_BYPASS
    /* 连发：按指令射频逐发产生"待确认" */
    if (lh_cmd_rps > 0.0f)
    {
        lh_cmd_accum += lh_cmd_rps * dt;
        while (lh_cmd_accum >= 1.0f)
        {
            lh_cmd_accum -= 1.0f;
            lh_pending += 1.0f;
            if (lh_pending_ms <= 0.0f)
                lh_pending_ms = 0.001f;
        }
    }
    else
    {
        lh_cmd_accum = 0.0f;
    }
#endif /* !SHOOT_HEAT_BENCH_BYPASS */

    /* 电流回落到阈值以下 -> 重新武装，避免一个长脉冲被数成多几发 */
    if (lh_amp < SHOOT_HEAT_DETECT_AMP * SHOOT_HEAT_DETECT_REARM_RATIO)
        lh_armed = 1;

    switch (lh_state)
    {
    case LOADER_HEAT_IDLE:
        lh_suspect_ms = 0.0f;
        if (spun_up)
            lh_state = LOADER_HEAT_READY;
        break;

    case LOADER_HEAT_READY:
        lh_suspect_ms = 0.0f;
        if (!spun_up)
        {
            lh_state = LOADER_HEAT_IDLE;
            break;
        }
        if (lh_armed && (lh_amp >= SHOOT_HEAT_DETECT_AMP))
        {
            lh_armed = 0;
            lh_suspect_ms = 0.0f;
            lh_state = LOADER_HEAT_SUSPECT;
        }
        break;

    case LOADER_HEAT_SUSPECT:
        if (!spun_up)
        {
            lh_state = LOADER_HEAT_IDLE;
            break;
        }
        if (lh_amp < SHOOT_HEAT_DETECT_AMP)
        {
            lh_state = LOADER_HEAT_READY; /* 抖动，不算一发 */
            break;
        }
        lh_suspect_ms += dt * 1000.0f;
        if (lh_suspect_ms >= SHOOT_HEAT_DETECT_MS)
        {
            lh_confirmed++;
            if (lh_pending >= 1.0f)
                lh_pending -= 1.0f;
            lh_add_heat("confirm");
            lh_state = LOADER_HEAT_CONFIRMED;
        }
        break;

    case LOADER_HEAT_CONFIRMED:
    default:
        /* 瞬态：等电流回落就回 READY（armed 已在上面处理） */
        if (!spun_up)
            lh_state = LOADER_HEAT_IDLE;
        else if (lh_armed)
            lh_state = LOADER_HEAT_READY;
        break;
    }

#if !SHOOT_HEAT_BENCH_BYPASS
    /* 漏检兜底：下令后长时间没确认，按"打了"保守加热量（宁高勿低） */
    if (lh_pending >= 1.0f)
    {
        lh_pending_ms += dt * 1000.0f;
        if (lh_pending_ms >= (float)SHOOT_HEAT_CONFIRM_TIMEOUT_MS)
        {
            lh_pending_ms = 0.0f;
            lh_missed++;
            lh_add_heat("miss");
            lh_pending -= 1.0f;
            if (lh_pending < 0.0f)
                lh_pending = 0.0f;
        }
    }
    else
    {
        lh_pending_ms = 0.0f;
    }
#endif /* !SHOOT_HEAT_BENCH_BYPASS */
}

/* ---------------- 周期更新 ---------------- */
void LoaderHeatUpdate(float now_ms)
{
    float dt;

    if (!lh_inited)
    {
        lh_last_ms = now_ms;
        lh_inited = 1;
        return;
    }

    dt = (now_ms - lh_last_ms) / 1000.0f;
    lh_last_ms = now_ms;

    if (dt <= 0.0f || dt > 0.2f)
        dt = 0.01f; /* 防跳变（首帧/调试断点） */

    /* ---- 底层：冷却积分 ---- */
    lh_q -= lh_cool * dt;
    if (lh_q < 0.0f)
        lh_q = 0.0f;

    /* ---- 底层：发弹确认 ---- */
    lh_update_fsm(dt);

#if SHOOT_HEAT_BENCH_BYPASS
    /* 台架模式：不限幅, 连发直接用机械速率; 估算仍照常跑(接了摩擦轮就能看到真实曲线) */
    lh_allowed_rps = LOADER_FIRE_RATE_HZ;
    lh_single_only = 0;
    lh_blocked = 0;
    return;
#endif

    /* ---- 上层：前馈规划 ---- */
    {
        const float steady = lh_cool / lh_per_shot;                     /* a/d */
        const float dt_settle = SHOOT_HEAT_SETTLE_MS / 1000.0f;
        /* 一个结算窗口内不超上限的射速上限 */
        const float cap = ((lh_limit - lh_q) / dt_settle + lh_cool) / lh_per_shot;
        float allow = cap;

#if SHOOT_HEAT_POLICY_STEADY
        if (allow > steady)
            allow = steady; /* 稳态优先：不超过 a/d */
#endif

        if (allow < 0.0f)
            allow = 0.0f;
        if (allow > SHOOT_HEAT_MAX_RPS)
            allow = SHOOT_HEAT_MAX_RPS;

        lh_allowed_rps = allow;
    }

    /* ---- 兜底：硬限幅（按"再打几发会不会超"倒算，比拍脑袋的百分比更准） ---- */
    lh_single_only = 0;
    lh_blocked = 0;

    if (lh_q + lh_per_shot > lh_limit)
    {
        lh_blocked = 1; /* 再打一发就超限 -> 全禁 */
        lh_allowed_rps = 0.0f;
    }
    else if (lh_q + 2.0f * lh_per_shot > lh_limit)
    {
        lh_single_only = 1; /* 还能打一发 -> 只允许单发 */
        lh_allowed_rps = 0.0f;
    }

    /* 告警（软/硬阈值只用于提示，真正生效的是上面两条） */
    if (lh_q >= SHOOT_HEAT_HARD_RATIO * lh_limit && !lh_warn_print)
    {
        lh_warn_print = 1;
        LOGWARNING("[heat] Q=%.1f near limit (%.0f), burst off", lh_q, lh_limit);
    }
    else if (lh_q < SHOOT_HEAT_SOFT_RATIO * lh_limit)
    {
        lh_warn_print = 0;
    }
}

/* ---------------- 输出 ---------------- */
float LoaderHeatGetQ(void) { return lh_q; }
float LoaderHeatGetLimit(void) { return lh_limit; }
float LoaderHeatGetCoolRps(void) { return lh_cool; }
float LoaderHeatGetPerShot(void) { return lh_per_shot; }
float LoaderHeatGetSteadyRps(void) { return lh_cool / lh_per_shot; }
float LoaderHeatGetAllowedRps(void) { return lh_allowed_rps; }
float LoaderHeatGetCmdRps(void) { return lh_cmd_rps; }
uint8_t LoaderHeatIsSingleOnly(void) { return lh_single_only; }
uint8_t LoaderHeatIsBlocked(void) { return lh_blocked; }
uint8_t LoaderHeatGetState(void) { return (uint8_t)lh_state; }
uint32_t LoaderHeatGetConfirmed(void) { return lh_confirmed; }
uint32_t LoaderHeatGetMissed(void) { return lh_missed; }
