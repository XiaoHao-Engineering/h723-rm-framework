/**
 * @file power_bench.c
 * @brief 实验一: 空载扫转速(阶梯保持版)
 *
 * 目的: 在**轮子悬空、没有机械负载**的条件下, 让转速和电流各停在一系列已知值上,
 *       配合外部读到的母线电流, 拟合功率模型里的 k1(与 ω² 成正比)和 a(静态损耗)。
 *
 * 为什么先做实验一(而不是直接堵转测 k2):
 *   1. 它是唯一"错了也不会烧东西"的实验: 电流 ≤ 2500 counts、轮子悬空、无堵转;
 *   2. 它顺带验证整条链路: CAN 能不能发、0x201 能不能收、1kHz 帧有没有断、
 *      串口能不能稳定吐数据 —— 这些任何一条不对, 后面堵转实验就是白做甚至危险;
 *   3. 它给出 a, 后面拟合 k2 时 a 已知, k2 才不会被 a 带偏;
 *   4. 它让你摸清电机的脾气: 启动死区、最高转速、发热速度 —— 直接决定实验二的脉冲长度;
 *   5. 一句话: 实验一是"校准仪器", 实验二是"称重", 顺序反了, 称出来的数不可信。
 *
 * 接线(详见 docs/实验一_空载扫转速.md):
 *   24V+ ──[电表]── C620 电源+        C620 CAN_H/L ── MC02 CAN1(PD1/PD0)
 *   24V- ────────── C620 电源-        两端 120Ω, MC02 与电调共地
 *   MC02 的 XT30 也接同一路 24V, 这样板载 ADC(PC4) 读到的就是母线电压
 *   USB-TTL: PA9(TX) → PC 的 RX, GND 共地, 115200-8N1
 *
 * 操作:
 *   上电 → 白色待机 → 按 PA15 开始 → 蜂鸣器响一声 → 2s 后开始逐档给电流
 *   任何时刻再按一次 PA15 = 立即把电流打到 0 并中止
 */

#include "power_bench.h"

#include "robot_def.h"

#include <stdio.h>
#include <string.h>

#include "bsp_board.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_adc.h"
#include "bsp_ws2812.h"

#include "dji_motor.h"
#include "daemon.h"
#include "buzzer.h"

/**
 * @brief 开机音: 三音上行 G4-A4-E5, 约 0.78 秒
 *
 * 与 `application/demo/demo_songs.h` 里的 `demo_startup_music` **完全一致**,
 * 就是"有音乐那一版"的开机提示音。这里单独写一份而不是 include demo_songs.h,
 * 是为了不让实验台固件把 demo 的其它曲子(小星星/欢乐颂/30s 旋律)也拉进来。
 * 想换开机音就改这三行。
 */
static const BuzzerNote_t k_boot_chime[] = {
    {392, 240}, /* G4 */
    {440, 240}, /* A4 */
    {655, 300}, /* E5 */
};

/* ======================= 实验参数(只改这里) ======================= */

/**
 * @brief 默认电流阶梯, 单位 counts
 *
 * M3508 + C620 的经验值:
 *   - 500 以下经常只是"抖一下", 起不来(静摩擦 + 减速箱);
 *   - 2500 时转子一般会上到 6000~9000 rpm, 已经很吵了, 第一次跑别超过它;
 *   - 空载时"电流决定转速"只在低速段成立, 高速段会被反电动势顶住,
 *     此时 given(C620 实测电流)会明显小于 Icmd —— 这是正常的, 用 given 拟合就行。
 */
static const int32_t k_default_steps[PB_STEP_MAX] = {
    500, 800, 1100, 1400, 1700, 2000, 2250, 2500,
};

/* ======================= 内部状态 ======================= */

#define PB_KEY_DEBOUNCE_S 0.40f /* 按键消抖 / 防止一次按下既启动又急停 */
#define PB_STOP_HOLD_S 2.0f     /* 结束后保持 0 电流多久再回到待机 */

typedef enum
{
    PB_WAIT_KEY = 0, /* 待机: 电流 0, 等按键 */
    PB_IDLE,         /* 按键后空转 idle_ms, 让轮子停稳 */
    PB_RUN,          /* 逐档保持 */
    PB_STOP,         /* 电流已归零, 冷却一下 */
} PB_State_e;

static DJIMotorInstance *s_motor = NULL;
static PowerBench_Config_s s_cfg;

static PB_State_e s_state = PB_WAIT_KEY;

/* 展开后的序列: 上升段 + (可选)下降段 */
static int32_t s_seq[PB_STEP_MAX * 2];
static uint32_t s_hold[PB_STEP_MAX * 2];
static uint8_t s_seq_n = 0;

static int16_t s_idx = -1;       /* 当前在第几档, -1 表示还没开始 */
static float s_t_state = 0.0f;   /* 进入当前状态的时刻(s) */
static float s_t_step = 0.0f;    /* 进入当前档的时刻(s) */
static float s_t_log = 0.0f;     /* 上一次打 CSV 的时刻(s) */
static float s_t_hint = 0.0f;    /* 上一次打提示日志的时刻(s) */
static float s_t_key = 0.0f;     /* 上一次按键的时刻(s) */
static float s_t_led = 0.0f;     /* 心跳灯上次翻转的时刻(s) */
static uint8_t s_led_on = 0;
static int32_t s_cmd = 0;        /* 当前下发的电流指令 */
static uint8_t s_fault = 0;      /* 1: 本次是被保护中止的 */
static uint8_t s_boot_logged = 0;
static uint8_t s_boot_chime_done = 0;
static float s_t_boot = 0.0f;    /* 第一次进入任务的时间, 用来给开机音留一点间隔 */
static uint8_t s_led[3] = {0, 0, 0};

static int32_t (*s_pin_hook)(void) = NULL;

/* ======================= 小工具 ======================= */

#if defined(BSP_LOG_UART)
#define PB_UART BSP_LOG_UART
#else
#define PB_UART BOARD_USART1
#endif

static void pb_send(const char *s, uint16_t len)
{
    HAL_UART_Transmit(PB_UART, (uint8_t *)s, len, 20);
}

static void pb_led(uint8_t r, uint8_t g, uint8_t b)
{
    if (s_led[0] == r && s_led[1] == g && s_led[2] == b)
        return;
    s_led[0] = r;
    s_led[1] = g;
    s_led[2] = b;
    BSPWS2812SetRGB(r, g, b);
}

/* 下发电流: 开环模式下 current_cmd 就等于这里给的 counts */
static void pb_set(int32_t cmd)
{
    if (cmd > 16384)
        cmd = 16384;
    if (cmd < -16384)
        cmd = -16384;
    s_cmd = cmd;
    if (s_motor != NULL)
        DJIMotorSetRef(s_motor, (float)cmd);
}

/* 转子转速, rpm。DJI 模块里存的是 deg/s(已经过一阶低通) */
static float pb_rpm(void)
{
    if (s_motor == NULL)
        return 0.0f;
    return s_motor->measure.speed_aps / 6.0f;
}

/* ======================= 一行动作 ======================= */

static void pb_log_row(float now)
{
    char line[96];
    int n;
    uint32_t t_ms = (uint32_t)(now * 1000.0f);
    uint16_t mv = (uint16_t)(ADCGetVbatVoltage() * 1000.0f);
    int32_t pin_mw = (s_pin_hook != NULL) ? s_pin_hook() : -1;
    int32_t rpm = (int32_t)pb_rpm();

    n = snprintf(line, sizeof(line), "%lu,%ld,%ld,%d,%u,%u,%d,%ld\r\n",
                 (unsigned long)t_ms,
                 (long)s_cmd,
                 (long)rpm,
                 (int)s_motor->measure.real_current, /* C620 回传的实测电流 */
                 (unsigned)s_motor->measure.temperature,
                 (unsigned)mv,
                 (int)s_idx,
                 (long)pin_mw);
    if (n > 0)
        pb_send(line, (uint16_t)n);
}

/** 进入第 idx 档 */
static void pb_step_goto(int16_t idx, float now)
{
    s_idx = idx;
    s_t_step = now;
    s_t_log = now; /* 新的一档, 立刻打一行, 方便对齐 */
    pb_set(s_seq[idx]);
    pb_log_row(now);
    LOGINFO("[bench] step %d/%d  Icmd=%ld  hold=%lums  (now is the time to read the meter)",
            (int)idx + 1, (int)s_seq_n, (long)s_seq[idx], (unsigned long)s_hold[idx]);
}

/** 保护触发: 立刻归零并中止 */
static void pb_abort(float now, const char *why)
{
    pb_set(0);
    s_fault = 1;
    s_state = PB_STOP;
    s_t_state = now;
    pb_led(40, 0, 0);
    LOGERROR("[bench] ABORT: %s", why);
}

/** 当前档是否安全 */
static uint8_t pb_guard(float now, float rpm)
{
    if (!DaemonIsOnline(s_motor->daemon))
    {
        pb_abort(now, "no C620 feedback on CAN1 (check wiring / 24V / 120ohm)");
        return 0;
    }
    if (rpm > (float)s_cfg.rpm_limit || rpm < -(float)s_cfg.rpm_limit)
    {
        pb_abort(now, "over speed");
        return 0;
    }
    if ((int32_t)s_motor->measure.temperature > s_cfg.temp_limit)
    {
        pb_abort(now, "over temperature");
        return 0;
    }
    return 1;
}

static void pb_start(float now)
{
    s_idx = -1;
    s_fault = 0;
    s_state = PB_IDLE;
    s_t_state = now;
    s_t_hint = now;
    s_t_log = now;
    pb_set(0);
    pb_led(0, 0, 40);
    LOGINFO("[bench] START: %d steps, idle %lums then ramp. press key again to abort.",
            (int)s_seq_n, (unsigned long)s_cfg.idle_ms);
}

static void pb_finish(float now)
{
    pb_set(0);
    s_state = PB_STOP;
    s_t_state = now;
    s_fault = 0;
    pb_led(0, 40, 0);
    LOGINFO("[bench] DONE: %d segments finished, current = 0.", (int)s_seq_n);
    LOGINFO("[bench] now run: python tools/power_bench_fit.py <csv>");
}

/* ======================= 初始化 ======================= */

void PowerBenchSetPinHook(int32_t (*fn)(void))
{
    s_pin_hook = fn;
}

void PowerBenchInit(void)
{
    Motor_Init_Config_s cfg = {0};
    uint8_t i;

    /* ---------------- 1. 参数 ---------------- */
    memset(&s_cfg, 0, sizeof(s_cfg));
    s_cfg.step_cnt = PB_STEP_MAX;
    for (i = 0; i < PB_STEP_MAX; i++)
    {
        s_cfg.step[i].i_cmd = k_default_steps[i];
        s_cfg.step[i].hold_ms = 10000; /* 10s 一档: 前 4~5s 等速度稳, 后面留给读数 */
    }
    s_cfg.down_pass = 1;    /* 原路返回一遍: 数据点翻倍, 还能看出静摩擦迟滞 */
    s_cfg.idle_ms = 2000;
    s_cfg.log_period_ms = 20; /* 50Hz, 115200 的串口够用 */
    s_cfg.rpm_limit = 9000;
    s_cfg.temp_limit = 70;

    /* 把"上升 + 下降"展开成一条序列 */
    s_seq_n = 0;
    for (i = 0; i < s_cfg.step_cnt; i++)
    {
        s_seq[s_seq_n] = s_cfg.step[i].i_cmd;
        s_hold[s_seq_n] = s_cfg.step[i].hold_ms;
        s_seq_n++;
    }
    if (s_cfg.down_pass)
    {
        for (i = s_cfg.step_cnt; i > 0; i--)
        {
            s_seq[s_seq_n] = s_cfg.step[i - 1].i_cmd;
            s_hold[s_seq_n] = s_cfg.step[i - 1].hold_ms;
            s_seq_n++;
        }
    }

    /* ---------------- 2. 注册电机(开环电流模式) ----------------
     * close_loop_type = OPEN_LOOP: DJIMotorControl() 里三环全部跳过,
     * current_cmd 直接等于 DJIMotorSetRef() 给的 counts。
     */
    cfg.can_init_config.can_handle = BOARD_CAN1;
    cfg.can_init_config.tx_id = 1;      /* C620 拨码 ID = 1 */
    cfg.can_init_config.rx_id = 0x201;  /* 会被 MotorSenderGrouping 按 ID 重算 */
    cfg.can_init_config.nominal_bitrate = 1000000;
    cfg.can_init_config.is_fd = 0;

    cfg.controller_setting_init_config.outer_loop_type = OPEN_LOOP;
    cfg.controller_setting_init_config.close_loop_type = OPEN_LOOP;
    cfg.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    cfg.controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL;
    cfg.controller_setting_init_config.angle_feedback_source = MOTOR_FEED;
    cfg.controller_setting_init_config.speed_feedback_source = MOTOR_FEED;
    cfg.controller_setting_init_config.feedforward_flag = FEEDFORWARD_NONE;

    /* 三个 PID 全部给 0: 开环模式下根本不会被调用, 但 DJIMotorInit 会初始化它们 */
    cfg.controller_param_init_config.current_PID = (PID_Init_Config_s){0};
    cfg.controller_param_init_config.speed_PID = (PID_Init_Config_s){0};
    cfg.controller_param_init_config.angle_PID = (PID_Init_Config_s){0};

    cfg.motor_type = M3508;

    s_motor = DJIMotorInit(&cfg);
    pb_set(0);

    s_state = PB_WAIT_KEY;
    s_t_key = DWT_GetTimeline_s();
    pb_led(20, 20, 20);

    /* 注意: RobotInit() 是在 __disable_irq() 里跑的, 此时 bsp_log 只会走 RTT,
     * 串口上一行都看不到。所以"开机横幅"放到第一次 PowerBenchTask 里打(见下)。 */
}

/* ======================= 周期任务(100Hz) ======================= */

void PowerBenchTask(void)
{
    float now;
    float rpm;
    uint8_t key;

    if (s_motor == NULL)
        return;

    now = DWT_GetTimeline_s();

    /* 开机横幅: 必须等调度器跑起来(中断已开)才打, 否则 bsp_log 的串口输出会被跳过 */
    if (!s_boot_logged)
    {
        s_boot_logged = 1;
        s_t_boot = now;
        LOGINFO("[bench] power bench ready: M3508(id=1) on CAN1, open-loop current, %d segments",
                (int)s_seq_n);
        LOGINFO("[bench] CSV: t_ms,Icmd,rpm,given,temp,vbat_mv,step,pin_mw  @50Hz on USART1 115200");
        LOGINFO("[bench] press PA15 to start, press again to abort");
    }

    /* ---------- 开机音: 上电响一次三音上行 ----------
     * 作用: "听到这三个音"= 程序烧进去了、调度器起来了、蜂鸣器模块也正常,
     *       是最省事的一次 POST(自检)。听不到就说明启动流程某一步没跑到。
     *
     * 两个细节:
     *   1) 蜂鸣器模块是在 DAEMON 任务里 BuzzerInit() 的, 所以必须等 BuzzerIsReady();
     *   2) 故意晚 0.3s 再响, 和 BSPInit() 里那声 60ms 的硬件自检"滴"分开,
     *      听起来是"滴……嘀-嘀-嘀", 不会糊成一片。 */
    if (!s_boot_chime_done && BuzzerIsReady() && (now - s_t_boot) > 0.3f)
    {
        s_boot_chime_done = 1;
        BuzzerPlayMelody(k_boot_chime, (uint16_t)ARRAY_LEN(k_boot_chime), 0);
        LOGINFO("[bench] boot chime OK (G-A-E5 heard => firmware is running)");
    }

    rpm = pb_rpm();
    key = BoardKeyPressed();

    /* -------- 按键: 待机时启动, 其他任何时候都是急停 -------- */
    if (key && (now - s_t_key) > PB_KEY_DEBOUNCE_S)
    {
        s_t_key = now;
        if (s_state == PB_WAIT_KEY)
        {
            pb_start(now);
            return;
        }
        pb_abort(now, "key pressed (manual stop)");
        return;
    }

    switch (s_state)
    {
    case PB_WAIT_KEY:
        pb_set(0);
        /* 1Hz 心跳灯: 待机时"板子还活着"必须一眼看得出来,
         * 否则很容易被当成"又卡死了"(尤其手边没有串口线的时候)。 */
        if ((now - s_t_led) > 0.5f)
        {
            s_t_led = now;
            s_led_on = (uint8_t)!s_led_on;
            pb_led(s_led_on ? 30 : 0, s_led_on ? 30 : 0, s_led_on ? 30 : 0);
        }
        if ((now - s_t_hint) > 3.0f)
        {
            s_t_hint = now;
            LOGINFO("[bench] idle, Icmd=0, vbat=%dmV | press PA15 to start",
                    (int)(ADCGetVbatVoltage() * 1000.0f));
            if (!DaemonIsOnline(s_motor->daemon))
                LOGWARNING("[bench] no feedback from C620: CAN1 wiring / 120ohm / ESC power / ID=1 ?");
        }
        break;

    case PB_IDLE:
        pb_set(0);
        pb_led(0, 0, 40);
        if ((now - s_t_state) * 1000.0f >= (float)s_cfg.idle_ms)
        {
            s_state = PB_RUN; /* 别忘了切状态, 否则会一直卡在 IDLE 里重复发第一档 */
            pb_step_goto(0, now);
        }
        break;

    case PB_RUN:
        pb_led(0, 40, 0);
        if (!pb_guard(now, rpm))
            break;

        /* 到点换档 */
        if ((now - s_t_step) * 1000.0f >= (float)s_hold[s_idx])
        {
            if ((int16_t)(s_idx + 1) >= (int16_t)s_seq_n)
            {
                pb_finish(now);
                break;
            }
            pb_step_goto((int16_t)(s_idx + 1), now);
        }

        if ((now - s_t_log) * 1000.0f >= (float)s_cfg.log_period_ms)
        {
            s_t_log = now;
            pb_log_row(now);
        }
        break;

    case PB_STOP:
        pb_set(0);
        if ((now - s_t_state) > PB_STOP_HOLD_S)
        {
            s_state = PB_WAIT_KEY;
            s_t_key = now; /* 防止这次按键立刻又启动 */
            s_t_hint = now;
            pb_led(20, 20, 20);
            LOGINFO("[bench] back to idle (%s), press PA15 to run again", s_fault ? "aborted" : "done");
        }
        break;

    default:
        s_state = PB_WAIT_KEY;
        break;
    }
}
