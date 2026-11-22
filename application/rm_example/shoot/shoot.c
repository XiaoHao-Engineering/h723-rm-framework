/**
 * @file shoot.c
 * @brief 发射机构: 2 x M3508 摩擦轮(速度环) + DM4310 拨弹盘(速度模式)
 *
 * 拨弹盘为什么用速度模式而不是 MIT:
 *   拨弹盘是"连续转/一格一格转"的机构, 不是位置伺服。MIT 的 kp 是弹簧, 卡弹时
 *   弹簧会一直加力直到过流, 齿轮吃的是硬力矩; 速度模式的目标是转速, 卡住时表现
 *   为速度误差, 再叠加"电机内部电流上限(0x03) + 软件扭矩阈值"两道保护,
 *   还能自己退弹重试。
 *
 * 上电时序(关键, 顺序错了电机就"装死"):
 *   达妙用 CAN 改模式(0x0A)【不存 flash】, 掉电会回到上次保存的模式, 所以每次上电
 *   都要重新写。而且刚上电电机在自检(约 1s), 太早发帧会丢 -> 电机停在 MIT 模式,
 *   表现为"使能绿灯正常、反馈正常, 但 0x200+ID 的速度帧被忽略、有目标不转"。
 *   因此这里用状态机: 等自检 -> 失能 -> 写模式 -> 写电流上限 -> 使能(0x200+ID) -> 确认。
 */

#include "robot_cmd.h"
#include "loader_heat.h"
#include "vofa.h"
#include "shoot.h"
#include "robot_def.h"
#include "rm_config.h"

#include "dji_motor.h"
#include "dm_motor.h"
#include "message_center.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "general_def.h"
#include "bsp_board.h"

#include <math.h>

static DJIMotorInstance *friction_l, *friction_r; // 摩擦轮
static float shoot_friction_log_t; /* 摩擦轮日志节流 */
static float shoot_friction_cmd; /* 当前摩擦轮指令(度/秒, 含方向) */
static DMMotorInstance *loader;                   // 拨盘(达妙 4310)

static Publisher_t *shoot_pub;
static Subscriber_t *shoot_sub;
static Shoot_Ctrl_Cmd_s shoot_cmd_recv;
static Shoot_Upload_Data_s shoot_feedback_data;

/* ---------------- 拨弹盘状态机 ---------------- */
typedef enum
{
    LOADER_BOOT_WAIT = 0,   // 等电机上电自检
    LOADER_BOOT_MODE,       // 写控制模式 + 电流上限
    LOADER_BOOT_ENABLE,     // 使能(帧 ID = 0x200+ID)
    LOADER_READY,           // 正常工作
} loader_boot_e;

typedef enum
{
    JAM_IDLE = 0,
    JAM_STOP,
    JAM_REVERSE,
    JAM_SETTLE,
    JAM_BACKOFF, /* 连续卡弹后回拨大角度(代替失能) */
} jam_state_e;

static loader_boot_e boot_state = LOADER_BOOT_WAIT;
static float boot_tick = 0.0f;
static uint8_t boot_mode_done = 0;

static jam_state_e jam_state = JAM_IDLE;
static float jam_tick = 0.0f;
static uint8_t jam_retry = 0;          /* 连续异常次数, 达到 LOADER_RETRY_MAX 就回拨大角度 */
static uint8_t jam_next_backoff = 0;   /* 本次恢复是否走"回拨大角度" */
static float jam_dir = 1.0f;           /* 卡住时的旋转方向(反转退让要反着来) */
static float jam_backoff_start_deg = 0.0f;
static float jam_ok_ms = 0.0f;  /* 正常运行的累计时间, 用于清零"连续"计数 */

/* 电机位置反馈是 16 位映射到 ±PMAX(默认 ±12.5rad≈±716°), 会回绕;
 * 这里在软件里用"每 10ms 的最短路径增量"累计成一个不回绕的总角度,
 * 单发角度积分与回拨都基于它, 避免绕圈时算错。 */
static float loader_pos_total_deg = 0.0f;
static float loader_pos_prev_deg = 0.0f;
static uint8_t loader_pos_inited = 0;

static float loader_cmd_rpm = 0.0f;    // 斜坡后的实际指令(输出轴 rpm)
static float loader_target_rpm = 0.0f; // 本周期期望
static float loader_enable_tick = 0.0f;
static float single_fire_end = 0.0f;   // 单发结束时刻, 0 表示没有在单发
static float single_fire_next = 0.0f;  // 下一发最早能开始的时刻(冷却)
static uint8_t single_fire_active = 0;

#if RM_LOADER_ANGLE_ENABLE
/* ---- 右上拨杆单发(角度控制) ---- */
static uint8_t loader_shot_sw_prev = 0;   /* 上周期拨杆值, 做上升沿 */
static uint8_t loader_shot_busy = 0;
static float loader_shot_cool_until = 0.0f; /* 单发完成后冷却到什么时候 */      /* 正在推进一发 */
static uint8_t loader_shot_fault = 0;     /* 超时锁存, 拨回中位复位 */
static float loader_shot_start_deg = 0.0f;
static float loader_shot_prev_start_deg = 0.0f; /* 上一发的起点, 用来算它实际走了多少 */
static uint8_t loader_shot_have_prev = 0;
static float loader_shot_corr = 0.0f;           /* 本发补偿量(度): 上一发少了就补, 多了就减 */
static float loader_shot_t0 = 0.0f;
static uint32_t loader_shot_cnt = 0;

/**
 * @brief 单发推进: 返回本周期目标转速(输出轴 rpm)
 * @param edge 本周期是否出现"拨到上"的上升沿
 */
static float __attribute__((unused)) LoaderShotStep(float now, uint8_t sw, uint8_t edge)
{
    float pos_deg;
    float d;

    if (loader == NULL || !DMMotorIsEnabled(loader))
        return 0.0f;

    pos_deg = loader_pos_total_deg;

    /* 拨回中/下: 清锁存, 允许下一次单发 */
    if (sw != RC_SW_UP)
    {
        loader_shot_busy = 0;
        loader_shot_fault = 0;
        return 0.0f;
    }

    /* 上升沿: 开始一发 */
    if (edge && !loader_shot_busy && !loader_shot_fault && now >= loader_shot_cool_until)
    {
#if SHOOT_HEAT_ENABLE
        /* 硬限幅: 再打一发就要超限, 这发不放行 */
        if (LoaderHeatIsBlocked())
        {
            LOGWARNING("[heat] blocked, single shot refused (Q=%.1f)", LoaderHeatGetQ());
            return 0.0f;
        }
        LoaderHeatOnSingleShot();
#endif
        /* 用上一发的"真实终点"评估它实际走了多少, 更新补偿量(消除惯性与打滑的累积误差) */
        if (loader_shot_have_prev)
        {
            float travel = pos_deg - loader_shot_prev_start_deg;

            while (travel > 180.0f)
                travel -= 360.0f;
            while (travel < -180.0f)
                travel += 360.0f;
            travel *= (LOADER_DIR < 0.0f) ? -1.0f : 1.0f;

            loader_shot_corr += (LOADER_ANGLE_DEG - travel);
            if (loader_shot_corr > LOADER_ANGLE_CORR_MAX)
                loader_shot_corr = LOADER_ANGLE_CORR_MAX;
            else if (loader_shot_corr < -LOADER_ANGLE_CORR_MAX)
                loader_shot_corr = -LOADER_ANGLE_CORR_MAX;

            LOGINFO("[shoot] prev shot %.1f deg, corr %+.1f", travel, loader_shot_corr);
        }

        loader_shot_prev_start_deg = pos_deg;
        loader_shot_have_prev = 1;

        loader_shot_busy = 1;
        loader_shot_t0 = now;
        loader_shot_start_deg = pos_deg;
        loader_shot_cnt++;
        LOGINFO("[shoot] single shot #%d from %.1f deg", (int)loader_shot_cnt, pos_deg);
    }

    if (!loader_shot_busy)
        return 0.0f;

    /* 已经转过的角度(归一化到 ±180, 避免越界跳变) */
    d = pos_deg - loader_shot_start_deg;
    while (d > 180.0f)
        d -= 360.0f;
    while (d < -180.0f)
        d += 360.0f;
    if (LOADER_DIR < 0.0f)
        d = -d;

    if (d >= (LOADER_ANGLE_DEG - LOADER_ANGLE_STOP_MARGIN_DEG + loader_shot_corr))
    {
        loader_shot_busy = 0;
        loader_shot_cool_until = now + (float)LOADER_ANGLE_COOLDOWN_MS; /* 冷却, 防一次拨动两发 */
        /* 实测这一发实际转了多少度, 用来判断打滑/空发(与设定值差太多就要查机构) */
        LOGINFO("[shoot] shot done, %.1f deg (target %.0f)", d, LOADER_ANGLE_DEG);
        return 0.0f;
    }

    if ((now - loader_shot_t0) > (float)LOADER_ANGLE_TIMEOUT_MS)
    {
        loader_shot_busy = 0;
        loader_shot_cool_until = now + (float)LOADER_ANGLE_COOLDOWN_MS;
        loader_shot_fault = 1;
        LOGWARNING("[shoot] single shot timeout, %.1f deg", d);
        return 0.0f;
    }

    /* 两段速度: 远离目标时快, 接近目标时慢, 减小惯性超调(超调会让下一发累积偏移) */
    if ((LOADER_ANGLE_DEG - d) > LOADER_ANGLE_SLOW_ZONE_DEG)
        return LOADER_DIR * LOADER_ANGLE_RPM_FAST;

    return LOADER_DIR * LOADER_ANGLE_RPM_SLOW;
}
#if RM_LOADER_POS_MODE
/**
 * @brief 单发: 位置模式(0x100+ID), 目标 = 当前展开角 + 一发角度
 *
 * 参考 mc-02-framework 的做法:
 *   - 目标永远基于"当前角度", 不用绝对角 -> 不会一上电就猛转
 *   - 反馈无效/未使能时不发目标
 *   - 到位判据用位置误差(不是时间), 阻力大也能顶到位 -> 不漏发
 *   - 到位后继续保持目标(力矩被 OC 限流, 不会超过设定上限)
 *
 * @return 速度指令恒为 0(位置帧由 MOTORTASK 按 ctrl_mode 发送)
 */
static float loader_pos_target_deg;
static float loader_pos_shot_t0;
static uint8_t loader_pos_shot_active;

static float LoaderPosShotStep(float now, uint8_t sw, uint8_t edge)
{
    float err, rpm;

    if (loader == NULL || !DMMotorIsEnabled(loader))
        return 0.0f;

    /* 拨杆离开上档: 结束本次单发 */
    if (sw != RC_SW_UP)
    {
        loader_pos_shot_active = 0;
        return 0.0f;
    }

    /* 上升沿: 只触发一次, 目标 = 当前展开角 + 一发角度 */
    if (edge && !loader_pos_shot_active)
    {
        loader_pos_target_deg = loader_pos_total_deg + LOADER_DIR * LOADER_ANGLE_DEG;
        loader_pos_shot_t0 = now;
        loader_pos_shot_active = 1;
        LOGINFO("[shoot] pos shot: %.1f -> %.1f deg", loader_pos_total_deg, loader_pos_target_deg);
    }

    if (!loader_pos_shot_active)
        return 0.0f;

    err = loader_pos_target_deg - loader_pos_total_deg;

    /* 到位: 位置误差足够小(不是靠时间, 阻力大也能顶到位) */
    if (fabsf(err) <= LOADER_POS_DONE_DEG)
    {
        loader_pos_shot_active = 0;
        loader_shot_cnt++;
        LOGINFO("[shoot] pos shot done #%d, err %.1f deg", (int)loader_shot_cnt, err);
        return 0.0f;
    }

    /* 超时: 判定卡弹(阻力太大顶不动) */
    if ((now - loader_pos_shot_t0) > (float)LOADER_ANGLE_TIMEOUT_MS)
    {
        loader_pos_shot_active = 0;
        loader_shot_fault = 1;
        LOGWARNING("[shoot] pos shot timeout, err %.1f deg", err);
        return 0.0f;
    }

    /* 软件位置环(P 控制): 误差大就快, 接近就慢; 低于最小爬行速度时给最小速度顶住阻力 */
    rpm = LOADER_POS_KP * err;
    if (fabsf(rpm) < LOADER_POS_MIN_RPM)
        rpm = (err > 0.0f) ? LOADER_POS_MIN_RPM : -LOADER_POS_MIN_RPM;
    if (rpm > LOADER_ANGLE_RPM_FAST)
        rpm = LOADER_ANGLE_RPM_FAST;
    else if (rpm < -LOADER_ANGLE_RPM_FAST)
        rpm = -LOADER_ANGLE_RPM_FAST;

    return LOADER_DIR * rpm;
}
#endif /* RM_LOADER_POS_MODE */

#endif /* RM_LOADER_ANGLE_ENABLE */

static float FrictionCurrentA(const DJIMotorInstance *m)
{
    /* C620: 电流反馈 raw int16, ±16384 对应 ±20A */
    if (m == NULL)
        return 0.0f;
    return (float)m->measure.real_current * (20.0f / 16384.0f);
}

/**
 * @brief 两个摩擦轮电调是否都在线
 *
 * 丢反馈时不允许使能, 避免 PID 饱和导致 C620 收到满电流而一直转。
 */
static uint8_t FrictionPairOnline(void)
{
    if (friction_l == NULL || friction_r == NULL)
        return 0;
    if (friction_l->daemon == NULL || friction_r->daemon == NULL)
        return 0;

    return (uint8_t)((DaemonIsOnline(friction_l->daemon) != 0) &&
                     (DaemonIsOnline(friction_r->daemon) != 0));
}

/**
 * @brief 发使能帧(多帧 ID 覆盖)
 *
 * 达妙使能帧的 ID 随模式变(MIT=CAN ID, 位置=0x100+ID, 速度=0x200+ID),
 * 出厂默认是 MIT, 只发 0x200+ID 它不认。参考例程做法: 三种都发一遍,
 * 反正 enable 不会让电机转动, 多余的帧被忽略。
 */
static void LoaderEnableFrames(void)
{
    DMMotorEnableAt(loader, LOADER_CAN_ID);              /* MIT 模式: 帧 ID = CAN ID */
    DMMotorEnableAt(loader, 0x100u + LOADER_CAN_ID);     /* 位置速度模式 */
    DMMotorEnableAt(loader, 0x200u + LOADER_CAN_ID);     /* 速度模式 */
}

static float NowMs(void) { return DWT_GetTimeline_ms(); }

static float AbsF(float x) { return (x < 0.0f) ? -x : x; }

static float LoaderRpmToRad(float rpm) { return rpm * 6.2831853f / 60.0f; }
static float LoaderRadToRpm(float rad) { return rad * 60.0f / 6.2831853f; }

#define LOADER_SINGLE_COOLDOWN_MS (250.0f) /* 逐发模式两发之间的最小间隔 */

void ShootInit()
{
    /* ---------------- 摩擦轮: 2 x M3508, 速度环 ---------------- */
    Motor_Init_Config_s friction_config = {
        .can_init_config = {
            .can_handle = RM_CAN_SHOOT,
            .nominal_bitrate = 1000000,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = FRICTION_SPEED_KP,
                .Ki = FRICTION_SPEED_KI,
                .Kd = 0.0f,
                .IntegralLimit = 3000.0f,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = FRICTION_SPEED_MAXOUT,
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
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
        },
        .motor_type = M3508,
    };

    friction_config.can_init_config.rx_id = 0;
    friction_config.can_init_config.tx_id = FRICTION_L_ID;
    friction_l = DJIMotorInit(&friction_config);

    friction_config.can_init_config.rx_id = 0;
    friction_config.can_init_config.tx_id = FRICTION_R_ID;
    /* ★ 右轮不再翻转方向/反馈标志: 两个标志同时反向会破坏速度环的负反馈,
     *   造成"越转越给电流"的自激失控。镜像方向改为对指令取负(-ref)。 */
    friction_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    /* 右轮方向: 按 mc-02 的轴方向映射(只翻测量+输出, 目标不动), 等价于安装方向相反 */
    friction_config.controller_setting_init_config.feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE;
    friction_r = DJIMotorInit(&friction_config);

    /* ---------------- 拨弹盘: DM4310, 速度模式 ---------------- */
    DMMotor_Init_Config_s loader_config = {
        .can_init_config = {
            .can_handle = RM_CAN_SHOOT,
            .tx_id = LOADER_CAN_ID,  /* 控制帧: 速度模式 = 0x200 + ID */
            .rx_id = LOADER_RX_ID,   /* 反馈帧: 电机里的 Master ID(出厂 0) */
            .nominal_bitrate = 1000000,
        },
        .motor_id = LOADER_CAN_ID,
        .kp = 0.0f,
        .kd = 0.0f,
        .ctrl_mode = DM_MODE_VEL, /* 速度模式: 帧 ID = 0x200 + ID */
        .range = {
            .p_min = -DM_PMAX,
            .p_max = DM_PMAX,
            .v_min = -DM_VMAX,
            .v_max = DM_VMAX,
            .kp_min = DM_DEFAULT_KP_MIN,
            .kp_max = DM_DEFAULT_KP_MAX,
            .kd_min = DM_DEFAULT_KD_MIN,
            .kd_max = DM_DEFAULT_KD_MAX,
            .t_min = -DM_TMAX,
            .t_max = DM_TMAX,
        },
    };
    loader = DMMotorInit(&loader_config);

    boot_state = LOADER_BOOT_WAIT;
    boot_tick = NowMs();
    boot_mode_done = 0;

#if SHOOT_HEAT_ENABLE
    LoaderHeatInit();
#endif

    shoot_pub = PubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));
    shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));

    LOGINFO("[shoot] friction 2xM3508 + loader DM4310(CAN3 id %d) init", LOADER_CAN_ID);

    /* 按例程做法: 反馈帧不管落在哪个 ID 上都收 —— 主 ID + 0 + 0x100+ID + 0x200+ID */
    (void)DMMotorAttachAlias(loader, LOADER_CAN_ID);
    (void)DMMotorAttachAlias(loader, 0u);
    (void)DMMotorAttachAlias(loader, 0x100u + LOADER_CAN_ID);
    (void)DMMotorAttachAlias(loader, 0x200u + LOADER_CAN_ID);

#if CAN_LOOPBACK_SELFTEST
    /* CAN3 控制器内部回环自检: 区分 MCU 侧 与 外部收发器/接线 */
    if (BSPCAN_InternalLoopbackTest(BOARD_CAN3, 0x7FFu))
        LOGINFO("[can] CAN3 loopback selftest: OK (MCU side fine)");
    else
        LOGERROR("[can] CAN3 loopback selftest: FAIL (check CAN config/clock)");
#endif
}

/**
 * @brief 拨弹盘上电时序: 等自检 -> 失能 -> 写模式/电流 -> 使能
 * @return 1 表示可以正常发指令
 */
static uint8_t LoaderBootSequence(uint8_t allow_enable)
{
    const float now = NowMs();

    switch (boot_state)
    {
    case LOADER_BOOT_WAIT:
        if ((now - boot_tick) >= LOADER_BOOT_WAIT_MS)
        {
            DMMotorDisable(loader);
            boot_tick = now;
            boot_state = LOADER_BOOT_MODE;
        }
        return 0;

    case LOADER_BOOT_MODE:
        if ((now - boot_tick) < 50.0f)
            return 0;

        /* 写 0x0A = 3(速度模式) + 电流上限 0x03, 各写两次提高成功率 */
        DMMotorSetCtrlMode(loader, DM_MODE_VEL);
        DMMotorWriteParamFloat(loader, DM_RID_OC, LOADER_OC);
        boot_mode_done++;
        boot_tick = now;

        if (boot_mode_done >= 2)
        {
            boot_state = LOADER_BOOT_ENABLE;
            boot_tick = now;
            LOGINFO("[shoot] loader mode -> VEL(3), OC=%d%%", (int)(LOADER_OC * 100.0f));
        }
        return 0;

    case LOADER_BOOT_ENABLE:
        /* 速度模式下使能帧的 ID 是 0x200+ID, 按 500ms 重试。
         * ★ 急停档(S1 下)时不要使能, 否则会和 ShootTask 里的失能每 10ms 打架 */
        if (!allow_enable)
        {
            boot_tick = now;
            return 0;
        }
        if (DMMotorIsEnabled(loader))
        {
            boot_state = LOADER_READY;
            loader_enable_tick = now;
            LOGINFO("[shoot] loader enabled, ready");
            return 1;
        }
        if ((now - boot_tick) >= LOADER_ENABLE_RETRY_MS)
        {
            boot_tick = now;
            LoaderEnableFrames();
        }
        return 0;

    case LOADER_READY:
    default:
        /* 后面掉线/失能了也回到使能态重试 */
        if (!DMMotorIsEnabled(loader))
        {
            boot_state = LOADER_BOOT_ENABLE;
            boot_tick = now;
            return 0;
        }
        return 1;
    }
}

/**
 * @brief 卡弹 / 堵转 / 过扭矩保护 + 自恢复
 *
 * 判据: 要它转(cmd>=jam_min)、转不动(act < cmd*ratio)、扭矩还大(tq>=jam_tq)
 *
 * 恢复策略(赛场不能因为卡弹直接失能):
 *   - 单次卡弹   : 停 -> 反转退弹 -> 停 -> 继续
 *   - 连续 3 次  : 停 -> **回拨 LOADER_BACKOFF_DEG(默认 60°)** -> 停 -> 计数清零 -> 继续
 *                  (不再失能; 若之后又连续 3 次, 重复回拨)
 *   - 扭矩超限   : 不管是否在卡弹状态, 扭矩超过 LOADER_TQ_MAX 持续 LOADER_TQ_HOLD_MS,
 *                  都在**当前旋转方向的反方向**退让一次
 */
static void LoaderStartRecovery(float now, uint8_t big_backoff)
{
    jam_dir = (loader_cmd_rpm >= 0.0f) ? 1.0f : -1.0f;
    jam_next_backoff = big_backoff;
    jam_state = JAM_STOP;
    jam_tick = now;
#if RM_LOADER_ANGLE_ENABLE
    loader_shot_busy = 0; /* 放弃正在推进的那一发 */
#endif
}

static void LoaderProtect(float now)
{
    static float jam_cnt_ms = 0.0f;
    static float tq_cnt_ms = 0.0f;

    /* ---- 自恢复状态机 ---- */
    if (jam_state != JAM_IDLE)
    {
        const float el = now - jam_tick;

        switch (jam_state)
        {
        case JAM_STOP:
            loader_target_rpm = 0.0f;
            if (el >= (float)LOADER_SETTLE_MS)
            {
                if (jam_next_backoff)
                {
                    jam_backoff_start_deg = loader_pos_total_deg;
                    jam_state = JAM_BACKOFF;
                }
                else
                {
                    jam_state = JAM_REVERSE;
                }
                jam_tick = now;
            }
            break;

        case JAM_REVERSE:
            loader_target_rpm = -jam_dir * LOADER_REVERSE_RPM;
            if (el >= (float)LOADER_REVERSE_MS)
            {
                jam_state = JAM_SETTLE;
                jam_tick = now;
            }
            break;

        case JAM_BACKOFF:
        {
            /* 反向累计转过的角度(考虑 ±180 跳变) */
            float d = loader_pos_total_deg - jam_backoff_start_deg;
            float progress;

            while (d > 180.0f)
                d -= 360.0f;
            while (d < -180.0f)
                d += 360.0f;

            progress = -jam_dir * d; /* 反向转的角度为正 */

            loader_target_rpm = -jam_dir * LOADER_BACKOFF_RPM;

            if ((progress >= LOADER_BACKOFF_DEG) ||
                (el > (float)LOADER_BACKOFF_TIMEOUT_MS))
            {
                jam_state = JAM_SETTLE;
                jam_tick = now;
                LOGWARNING("[shoot] backoff done, %.0f deg", progress);
            }
            break;
        }

        case JAM_SETTLE:
            loader_target_rpm = 0.0f;
            if (el >= (float)LOADER_SETTLE_MS)
            {
                if (jam_next_backoff)
                {
                    jam_retry = 0; /* 回拨完成, 重新开始"连续 3 次"计数 */
                    jam_next_backoff = 0;
                }
                jam_state = JAM_IDLE;
                jam_tick = now;
                jam_cnt_ms = 0.0f;
                tq_cnt_ms = 0.0f;
            }
            break;

        default:
            jam_state = JAM_IDLE;
            break;
        }
        return; /* 恢复期间不接受新的发弹指令 */
    }

    if (!DMMotorIsEnabled(loader))
        return;
    if ((now - loader_enable_tick) < (float)LOADER_GRACE_MS) /* 起步宽限期 */
        return;

    const float abs_cmd = AbsF(loader_cmd_rpm);
    const float abs_act = AbsF(LoaderRadToRpm(loader->measure.velocity));
    const float abs_tq = AbsF(loader->measure.torque);

    /* ---- 1) 卡弹: 速度没跟上 + 扭矩顶住 ---- */
    if ((abs_cmd >= LOADER_JAM_MIN_RPM) &&
        (abs_act < abs_cmd * LOADER_JAM_SPEED_RATIO) &&
        (abs_tq >= LOADER_JAM_TQ))
    {
        jam_cnt_ms += 10.0f; /* ROBOT 任务 100Hz */
        jam_ok_ms = 0.0f;
    }
    else
    {
        jam_cnt_ms = 0.0f;

        /* 正常转了一段时间就把"连续卡弹次数"清零(只统计短时间内的连续卡弹) */
        jam_ok_ms += 10.0f;
        if (jam_ok_ms >= (float)LOADER_JAM_CLEAR_MS)
        {
            jam_ok_ms = 0.0f;
            jam_retry = 0;
        }
    }

    /* ---- 2) 扭矩超限: 与卡弹状态无关, 超阈值持续一段时间就反向退让 ---- */
    if (abs_tq >= LOADER_TQ_MAX)
    {
        tq_cnt_ms += 10.0f;
        if (tq_cnt_ms >= (float)LOADER_TQ_HOLD_MS)
        {
            tq_cnt_ms = 0.0f;
            jam_cnt_ms = 0.0f;
            jam_ok_ms = 0.0f;
            LOGWARNING("[shoot] torque over limit %.2f Nm, reverse", abs_tq);
            LoaderStartRecovery(now, 0); /* 只退让一次, 不计入卡弹次数 */
        }
    }
    else
    {
        tq_cnt_ms = 0.0f;
    }

    /* ---- 3) 卡弹计时到: 前 2 次反转退弹, 第 3 次起回拨大角度(不失能) ---- */
    if (jam_cnt_ms >= (float)LOADER_JAM_MS)
    {
        jam_cnt_ms = 0.0f;

        if (jam_retry < (uint8_t)(LOADER_RETRY_MAX - 1))
        {
            jam_retry++;
            LOGWARNING("[shoot] jam detected, retry %d/%d, reverse",
                       jam_retry, (int)LOADER_RETRY_MAX);
            LoaderStartRecovery(now, 0);
        }
        else
        {
            jam_retry = 0;
            LOGWARNING("[shoot] jam x%d -> backoff %.0f deg (never disable)",
                       (int)LOADER_RETRY_MAX, LOADER_BACKOFF_DEG);
            LoaderStartRecovery(now, 1);
        }
    }
}

void ShootTask()
{
    const float now = NowMs();

    SubGetMessage(shoot_sub, &shoot_cmd_recv);

    /* 位置反馈回绕处理: 16 位 POS 映射到 ±PMAX(默认 ±716°)会绕回,
     * 这里按"每 10ms 最短路径增量"累计成不回绕的总角度, 供单发/回拨使用 */
    if (loader != NULL)
    {
        const float raw_deg = loader->measure.position_deg;

        if (!loader_pos_inited)
        {
            loader_pos_total_deg = raw_deg;
            loader_pos_inited = 1;
        }
        else
        {
            /* 反馈 POS 是 16 位映射到 ±PMAX, 所以绕回周期是 (p_max - p_min),
             * 默认 ±12.5rad => 25rad = 1432.4°, 不是 360°! */
            const float range_deg = (loader->range.p_max - loader->range.p_min) * 57.2957795f;
            const float period_deg = (range_deg > 360.0f) ? range_deg : 360.0f;
            float dpos = raw_deg - loader_pos_prev_deg;

            if (dpos > 180.0f)
                dpos -= period_deg;
            else if (dpos < -180.0f)
                dpos += period_deg;

            loader_pos_total_deg += dpos;
        }
        loader_pos_prev_deg = raw_deg;
    }

    /* 右上拨杆: 在上 = 单发角度控制(每拨上一次打一发) */
    uint8_t rc_sw = 0;
    uint8_t angle_mode = 0;
    uint8_t angle_edge = 0;
#if RM_LOADER_ANGLE_ENABLE
    {
        RC_ctrl_t *rc = RobotCMDGetRC();
        rc_sw = (rc != NULL) ? (uint8_t)rc->rc.switch_right : 0u;
        angle_mode = (rc_sw == RC_SW_UP) ? 1u : 0u;
        angle_edge = (angle_mode && loader_shot_sw_prev != RC_SW_UP) ? 1u : 0u;
        loader_shot_sw_prev = rc_sw;
    }
#endif

#if SHOOT_HEAT_ENABLE
    /* ---------------- 热量管理：喂输入 + 更新 ----------------
     * 底层(数): 摩擦轮电流脉冲确认发弹, Qest += d, 每周期扣 a*dt
     * 上层(算): 按剩余热量反解允许射频; 稳态优先时约等于 a/d
     * 兜底(限): Qest 快满时先禁连发、再禁单发 */
    {
        const float aps_l = (friction_l != NULL) ? friction_l->measure.speed_aps : 0.0f;
        const float aps_r = (friction_r != NULL) ? friction_r->measure.speed_aps : 0.0f;
        const float i_l = AbsF(FrictionCurrentA(friction_l));
        const float i_r = AbsF(FrictionCurrentA(friction_r));
        const float i_a = (i_l > i_r) ? i_l : i_r;
        float cmd_rps = 0.0f;

        /* 只有连发才按指令射频累计发数; 单发/反转不参与(否则会虚增) */
        if (shoot_cmd_recv.shoot_mode != SHOOT_OFF && !angle_mode &&
            shoot_cmd_recv.load_mode == LOAD_BURSTFIRE)
            cmd_rps = AbsF(loader_cmd_rpm) * (float)LOADER_SLOTS / 60.0f;

        LoaderHeatFeedFriction(aps_l, aps_r, i_a);
        LoaderHeatFeedCommand(cmd_rps);
        LoaderHeatUpdate(now);
    }
#endif

#if RM_LOADER_HOLD_ENABLE
    /* 自检模式: 不用遥控, 只让拨盘"使能并保持 0 转"(灯变绿), 不做任何动作 */
    shoot_cmd_recv.shoot_mode = SHOOT_ON;
    shoot_cmd_recv.load_mode = LOAD_STOP;
    shoot_cmd_recv.friction_mode = FRICTION_OFF;
    shoot_cmd_recv.bullet_speed = SMALL_AMU_15;
#endif

#if LOADER_RX_AUTODETECT
    /* 收不到反馈 -> 在 rx_id = LOADER_RX_ID / LOADER_CAN_ID 之间来回试几次
     * (达妙反馈帧 ID = 电机里的 Master ID, 出厂常见 0 或 1) */
    {
        static float rx_probe_tick;
        static uint8_t rx_probe_cnt;
        static uint8_t rx_seen;

        if (loader != NULL && loader->motor_can_instance != NULL)
        {
            if (loader->measure.temperature_rotor != 0)
                rx_seen = 1;

            if (rx_seen)
            {
                rx_probe_cnt = 0;
            }
            else if (rx_probe_cnt < 4 && (now - rx_probe_tick) > 1200.0f)
            {
                const uint32_t cur = loader->motor_can_instance->rx_id;

                rx_probe_tick = now;
                rx_probe_cnt++;
                loader->motor_can_instance->rx_id =
                    (cur == (uint32_t)LOADER_RX_ID) ? (uint32_t)LOADER_CAN_ID : (uint32_t)LOADER_RX_ID;

                LOGINFO("[shoot] loader no feedback, rx_id -> %d (probe %d)",
                        (int)loader->motor_can_instance->rx_id, (int)rx_probe_cnt);
            }
        }
    }
#endif

    /* ---------------- 拨弹盘上电时序 ----------------
     * 急停档下不允许使能, 免得和下面的 SHOOT_OFF 失能逻辑每 10ms 互相打架 */
    const uint8_t loader_ready = LoaderBootSequence(shoot_cmd_recv.shoot_mode != SHOOT_OFF);

    /* ---------------- 发射总开关 ---------------- */
    if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
    {
        /* 急停: 全部停+失能 */
        DJIMotorStop(friction_l);
        DJIMotorStop(friction_r);
        DMMotorDisable(loader);
        loader_cmd_rpm = 0.0f;
        loader_target_rpm = 0.0f;
        jam_state = JAM_IDLE;
        jam_retry = 0;
        single_fire_active = 0;
    }
    else
    {
        /* 摩擦轮: 两个电调都"在线"才允许使能, 否则保持停止(并显式发 0) */
        if (FrictionPairOnline())
        {
            DJIMotorEnable(friction_l);
            DJIMotorEnable(friction_r);
        }
        else
        {
            DJIMotorStop(friction_l);
            DJIMotorStop(friction_r);
        }
        /* 只在没使能时补一帧, 不要每 10ms 都发(会白占 CAN 带宽) */
        if (loader_ready && !DMMotorIsEnabled(loader) && loader->stop_flag == MOTOR_STOP)
            LoaderEnableFrames();
    }

    /* ---------------- 拨盘目标转速 ---------------- */
    loader_target_rpm = 0.0f;

    if (shoot_cmd_recv.shoot_mode != SHOOT_OFF && loader_ready)
    {
        switch (shoot_cmd_recv.load_mode)
        {
        case LOAD_STOP:
            single_fire_active = 0;
            loader_target_rpm = 0.0f;
            /* 手动复位卡弹锁存: 打到停就清 */
                jam_retry = 0;
            break;

        case LOAD_1_BULLET:
            /* 逐发: 以固定转速转一个"弹距"的角度, 用时间换算
             *   t = angle / (rpm * 6) * 1000 ms  (rpm*6 = 度/秒)
             * 转完再等一个冷却, 这样按住拨轮是稳定的"一发一发", 不是连发 */
            if (!single_fire_active)
            {
                if (now >= single_fire_next)
                {
                    single_fire_active = 1;
                    single_fire_end = now + LOADER_ONE_BULLET_DEG /
                                                (LOADER_SINGLE_RPM * 6.0f) * 1000.0f;
                }
            }
            if (single_fire_active && now < single_fire_end)
            {
                loader_target_rpm = LOADER_DIR * LOADER_SINGLE_RPM;
            }
            else if (single_fire_active)
            {
                single_fire_active = 0;
                single_fire_next = now + LOADER_SINGLE_COOLDOWN_MS;
                loader_target_rpm = 0.0f;
            }
            else
            {
                loader_target_rpm = 0.0f;
            }
            break;

        case LOAD_BURSTFIRE:
            single_fire_active = 0;
            {
                /* 方向来自遥控(S1 上=顺时针, S1 中=左摇杆: 左推=顺时针/右推=逆时针) */
                float dir = shoot_cmd_recv.burst_dir;
                if (dir > 1.0f)
                    dir = 1.0f;
                else if (dir < -1.0f)
                    dir = -1.0f;

#if SHOOT_HEAT_ENABLE
                loader_target_rpm = LOADER_DIR * dir * LoaderHeatGetAllowedRps() *
                                    60.0f / (float)LOADER_SLOTS;
#else
                loader_target_rpm = LOADER_DIR * dir * LOADER_FIRE_RPM;
#endif
            }
            break;

        case LOAD_REVERSE:
            single_fire_active = 0;
            loader_target_rpm = -LOADER_DIR * LOADER_REVERSE_RPM;
            break;

        default:
            loader_target_rpm = 0.0f;
            break;
        }
    }

#if RM_LOADER_ANGLE_ENABLE
    /* 单发模式覆盖上面的速度指令(急停档优先, 不覆盖) */
    if (angle_mode && shoot_cmd_recv.shoot_mode != SHOOT_OFF && loader_ready)
    {
#if RM_LOADER_POS_MODE
        /* 单发: 速度模式 + 软件位置环(不切控制模式, 不会失能/使能抖动) */
        loader_target_rpm = LoaderPosShotStep(now, rc_sw, angle_edge);
#else
        loader_target_rpm = LoaderShotStep(now, rc_sw, angle_edge);
#endif
    }
#endif

#if SHOOT_HEAT_ENABLE
    /* 硬限幅优先级最高: 再打一发就超限 -> 拨盘停(连发与单发都拦下) */
    if (LoaderHeatIsBlocked() && shoot_cmd_recv.shoot_mode != SHOOT_OFF)
        loader_target_rpm = 0.0f;
#endif

    /* ---------------- 保护 ---------------- */
    if (loader_ready)
        LoaderProtect(now);

    /* ---------------- 斜坡 + 下发 ---------------- */
    {
        const float dt = 0.01f; /* ROBOT 任务 100Hz */
        const float step = LOADER_ACC_RPM_S * dt;

#if RM_LOADER_ANGLE_ENABLE
        if (angle_mode)
        {
            loader_cmd_rpm = loader_target_rpm; /* 单发直接下发, 不走斜坡 */
        }
        else
#endif
        if (loader_target_rpm > loader_cmd_rpm + step)
            loader_cmd_rpm += step;
        else if (loader_target_rpm < loader_cmd_rpm - step)
            loader_cmd_rpm -= step;
        else
            loader_cmd_rpm = loader_target_rpm;

        if (shoot_cmd_recv.shoot_mode == SHOOT_OFF)
            loader_cmd_rpm = 0.0f;

        DMMotorSetVelocity(loader, LoaderRpmToRad(loader_cmd_rpm));
    }

    /* ---------------- 摩擦轮: 斜坡起转 + 刹车停止 ----------------
     * 打开: 目标转速按 FRICTION_RAMP_MS 斜坡上升, 避免瞬间打满电流
     * 关闭: 先给 0 目标让速度环主动刹车, 转速降下来(或超时)再断输出
     * 方向: 两轮都用负指令(实测负反馈稳定, 镜像安装所以物理上对转)
     */
#if RM_FRICTION_WHEEL_ENABLE
    {
        static float friction_ref;      /* 斜坡后的目标转速 */
        static float brake_t0;          /* 刹车开始时刻 */
        static uint8_t braking;         /* 1=正在刹车 */

        const uint8_t want_on = (shoot_cmd_recv.friction_mode == FRICTION_ON) &&
                                (shoot_cmd_recv.shoot_mode != SHOOT_OFF);
        /* 有效目标转速: 台架测试用 FRICTION_TEST_RPM 限速, 为 0 时用初速反解值 */
        const float fr_target = (FRICTION_TEST_RPM > 0.0f) ? (FRICTION_TEST_RPM * 6.0f)
                                                           : FRICTION_TARGET_APS;

        if (want_on)
        {
            const float ramp_step = fr_target / FRICTION_RAMP_MS * 10.0f; /* 100Hz */

            braking = 0;
            brake_t0 = 0.0f;

            if (friction_ref < fr_target)
            {
                friction_ref += ramp_step;
                if (friction_ref > fr_target)
                    friction_ref = fr_target;
            }

            DJIMotorSetRef(friction_l, FRICTION_L_DIR * friction_ref);
            DJIMotorSetRef(friction_r, FRICTION_R_DIR * friction_ref);
            shoot_friction_cmd = FRICTION_L_DIR * friction_ref; /* VOFA 观测用 */

            if ((now - shoot_friction_log_t) > 1000.0f)
            {
                shoot_friction_log_t = now;
                LOGINFO("[friction] cmd %d deg/s | L %d deg/s %dmA | R %d deg/s %dmA",
                        (int)(FRICTION_L_DIR * friction_ref),
                        (int)((friction_l != NULL) ? friction_l->measure.speed_aps : 0.0f),
                        (int)(FrictionCurrentA(friction_l) * 1000.0f),
                        (int)((friction_r != NULL) ? friction_r->measure.speed_aps : 0.0f),
                        (int)(FrictionCurrentA(friction_r) * 1000.0f));
            }
        }
        else
        {
            const float spd_l = (friction_l != NULL) ? AbsF(friction_l->measure.speed_aps) : 0.0f;
            const float spd_r = (friction_r != NULL) ? AbsF(friction_r->measure.speed_aps) : 0.0f;
            const float spd = (spd_l > spd_r) ? spd_l : spd_r;

            if (!braking)
            {
                braking = 1;
                brake_t0 = now;
            }

            if ((spd > FRICTION_BRAKE_STOP_APS) && ((now - brake_t0) < FRICTION_BRAKE_MS))
            {
                /* 刹车阶段: 速度环目标 0, 输出反向电流主动减速 */
                DJIMotorSetRef(friction_l, 0.0f);
                DJIMotorSetRef(friction_r, 0.0f);
            }
            else
            {
                /* 停稳或超时: 断输出(显式发 0), 同时复位斜坡 */
                friction_ref = 0.0f;
                shoot_friction_cmd = 0.0f;
                DJIMotorStop(friction_l);
                DJIMotorStop(friction_r);
            }
        }
    }
#else
    DJIMotorStop(friction_l);
    DJIMotorStop(friction_r);
#endif

    /* ---------------- 上报 ---------------- */
    shoot_feedback_data.loader_cmd_rpm = loader_cmd_rpm;
    shoot_feedback_data.loader_act_rpm =
        (loader != NULL) ? LoaderRadToRpm(loader->measure.velocity) : 0.0f;

#if SHOOT_HEAT_ENABLE
    shoot_feedback_data.heat_q = LoaderHeatGetQ();
    shoot_feedback_data.heat_limit = LoaderHeatGetLimit();
    shoot_feedback_data.heat_allowed_rps = LoaderHeatGetAllowedRps();
    shoot_feedback_data.heat_cmd_rps = LoaderHeatGetCmdRps();
    shoot_feedback_data.heat_confirmed = (uint16_t)LoaderHeatGetConfirmed();
    shoot_feedback_data.heat_state = LoaderHeatGetState();
    shoot_feedback_data.heat_single_only = LoaderHeatIsSingleOnly();
    shoot_feedback_data.heat_blocked = LoaderHeatIsBlocked();
#endif

#if SHOOT_HEAT_ENABLE && SHOOT_HEAT_VOFA_ENABLE
    /* ---------------- VOFA+ 波形(UART1) ----------------
     * 通道: Qest / Q0 / 允许射频 / 指令射频 / 指令rpm / 实测rpm / 已确认发数 / 状态位 */
    {
        static uint8_t vofa_div;

        if (++vofa_div >= SHOOT_HEAT_VOFA_DIV)
        {
            float ch[8];

            /* 通道定义(VOFA+ 里按这个顺序看):
             *   0 摩擦轮L实测转速(度/秒, 转子)
             *   1 摩擦轮R实测转速(度/秒, 转子)
             *   2 摩擦轮L电流(A)
             *   3 摩擦轮R电流(A)
             *   4 摩擦轮指令转速(度/秒, 含方向)
             *   5 拨盘实测 rpm
             *   6 拨盘指令 rpm
             *   7 状态位: bit0-3 热量状态, bit4 只允许单发, bit5 已锁定
             */
            vofa_div = 0;
            ch[0] = (friction_l != NULL) ? friction_l->measure.speed_aps : 0.0f;
            ch[1] = (friction_r != NULL) ? friction_r->measure.speed_aps : 0.0f;
            ch[2] = FrictionCurrentA(friction_l);
            ch[3] = FrictionCurrentA(friction_r);
            ch[4] = shoot_friction_cmd;
            ch[5] = shoot_feedback_data.loader_act_rpm;
            ch[6] = loader_cmd_rpm;
            ch[7] = (float)((LoaderHeatGetState() & 0x0F) |
                            ((LoaderHeatIsSingleOnly() & 0x01) << 4) |
                            ((LoaderHeatIsBlocked() & 0x01) << 5));

            vofa_justfloat_output(ch, 8, SHOOT_HEAT_VOFA_UART);
        }
    }
#endif

    PubPushMessage(shoot_pub, (void *)&shoot_feedback_data);
}
