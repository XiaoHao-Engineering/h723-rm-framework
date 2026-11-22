/**
 * @file robot_cmd.c
 * @brief 整车指令层: DR16 遥控器 -> 底盘/云台/发射 三条指令
 *
 * 遥控映射(详见 rm_config.h 第七节):
 *   S1(左开关): 上 = 正常工作(云台+发射)   中 = 只调云台(发射禁用)   下 = 急停
 *   S2(右开关): 上 = 小陀螺               中 = 不跟随(可平移)        下 = 跟随云台
 *   右摇杆    : 上下 = 前进/后退, 左右 = 左右平移
 *   左摇杆    : 左右 = Yaw 转速, 上下 = Pitch 转速(增量控制)
 *   左拨轮    : 抬起来 = 摩擦轮开; 推到底 = 连发
 *
 * 云台是"增量控制": 摇杆给的是角速度, 累加到目标角上, 松手后目标角保持不变。
 * 这样摇杆的手感和 DJI 云台一致, 也不需要绝对角度基准。
 */

// app
#ifndef RC_PIN_PROBE
#define RC_PIN_PROBE 0 /* 1: 打印 UART5_RX(PD2) 边沿计数, 诊断接收机是否有信号 */
#endif

#include "bsp_can.h"
#include "robot_def.h"
#include "robot_cmd.h"
#include "rm_config.h"
// module
#include "remote_control.h"
#include "message_center.h"
#include "general_def.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_board.h"

#include <stdlib.h>
#include <math.h>

static Publisher_t *chassis_cmd_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
static Publisher_t *gimbal_cmd_pub;    // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;  // 云台反馈信息订阅者
static Publisher_t *shoot_cmd_pub;     // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;   // 发射反馈信息订阅者

static Chassis_Ctrl_Cmd_s chassis_cmd_send;
static Chassis_Upload_Data_s chassis_fetch_data;
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;
static Gimbal_Upload_Data_s gimbal_fetch_data;
static Shoot_Ctrl_Cmd_s shoot_cmd_send;
static Shoot_Upload_Data_s shoot_fetch_data;

static RC_ctrl_t *rc_data; // 遥控器数据,初始化时返回

static Robot_Status_e robot_state;
static float last_tick = 0.0f;

/* 首次对准标志: 上电时云台的物理位置是任意的, 如果直接把 pitch 目标设成 0,
 * 一使能小 P 就会往 0 度猛拉一下(可能撞限位)。所以等 IMU 有数据后,
 * 先把目标"对齐"到当前实际角度, 之后再让摇杆去改。 */
static uint8_t cmd_seeded = 0;
static uint16_t seed_cnt = 0;

/* 把角度归一化到 [-180, 180] */
static float Wrap180(float deg)
{
    while (deg > 180.0f)
        deg -= 360.0f;
    while (deg < -180.0f)
        deg += 360.0f;
    return deg;
}

/* 摇杆原始值(-660~660) -> 归一化(-1~1), 带死区 */
static float StickNorm(int16_t raw, int16_t deadband)
{
    if (abs(raw) <= deadband)
        return 0.0f;
    float v = (float)raw / 660.0f;
    if (v > 1.0f)
        v = 1.0f;
    if (v < -1.0f)
        v = -1.0f;
    return v;
}

void RobotCMDInit()
{
    /* DR16 接收机接板载 SBUS 口(带反相器), 对应 UART5 */
    rc_data = RemoteControlInit(BOARD_UART5);

    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

    /* 上电时云台目标对准当前的 IMU 朝向, 避免第一次收到摇杆就阶跃 */
    gimbal_cmd_send.yaw = 0.0f;
    gimbal_cmd_send.pitch = 0.0f;

    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    shoot_cmd_send.bullet_speed = SMALL_AMU_18;

    robot_state = ROBOT_READY;
    last_tick = DWT_GetTimeline_ms();
}
/**
 * @brief 取遥控器数据指针(供其他模块复用, 不重复初始化串口)
 */
RC_ctrl_t *RobotCMDGetRC(void)
{
    return rc_data;
}


/**
 * @brief 根据云台反馈算底盘与云台正方向的夹角
 *        yaw_relative_deg 是"云台相对底盘同向位置"的角度(0~360), 直接减就行
 */
static void CalcOffsetAngle()
{
    chassis_cmd_send.offset_angle = Wrap180(gimbal_fetch_data.yaw_relative_deg);
}

/**
 * @brief 遥控器控制: 输入 -> 三条指令
 */
static void RemoteControlSet(float dt)
{
    /* ---------------- 云台(增量控制) ---------------- */
    const float yaw_rate = StickNorm(rc_data[TEMP].rc.rocker_l_, RC_DEADBAND);
    const float pitch_rate = StickNorm(rc_data[TEMP].rc.rocker_l1, RC_DEADBAND);

    gimbal_cmd_send.yaw += RC_YAW_SIGN * yaw_rate * YAW_STICK_RATE_DEG_S * dt;
    gimbal_cmd_send.pitch += RC_PITCH_SIGN * pitch_rate * PITCH_STICK_RATE_DEG_S * dt;

    /* 云台限位: 只夹 pitch, yaw 可以连续转 */
    if (gimbal_cmd_send.pitch > PITCH_CMD_MAX_DEG)
        gimbal_cmd_send.pitch = PITCH_CMD_MAX_DEG;
    if (gimbal_cmd_send.pitch < PITCH_CMD_MIN_DEG)
        gimbal_cmd_send.pitch = PITCH_CMD_MIN_DEG;

    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

    /* ---------------- 底盘 ---------------- */
    chassis_cmd_send.vx = RC_VX_SIGN * StickNorm(rc_data[TEMP].rc.rocker_r1, RC_DEADBAND) * RC_VX_MAX;
    chassis_cmd_send.vy = RC_VY_SIGN * StickNorm(rc_data[TEMP].rc.rocker_r_, RC_DEADBAND) * RC_VY_MAX;

    /* S2: 上=小陀螺 中=不跟随 下=跟随云台 */
    if (switch_is_up(rc_data[TEMP].rc.switch_right))
        chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right))
        chassis_cmd_send.chassis_mode = CHASSIS_NO_FOLLOW;
    else
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;

    /* ---------------- 发射 ---------------- */
    /* ---------------- 发射 ----------------
     * 对齐达妙 EggPlate 例程的通道用法:
     *   S2(右) 下 = 停机; 中 = 速度模式(连发); 上 = 角度单发
     *   S1(左) 上 = 顺时针; 中 = 手动(左摇杆: 左推=顺时针, 右推=逆时针); 下 = 急停
     * 拨轮不再参与模式选择(例程也是这样), 磨擦轮由 RM_FRICTION_WHEEL_ENABLE 决定
     */
    {
        const uint8_t sw1 = rc_data[TEMP].rc.switch_left;
        const uint8_t sw2 = rc_data[TEMP].rc.switch_right;
        const int16_t stick_x = rc_data[TEMP].rc.rocker_l_; /* DBUS 左摇杆水平, 左为负 */

        if (switch_is_down(sw2)) /* S2 下: 停机 */
        {
            shoot_cmd_send.load_mode = LOAD_STOP;
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            shoot_cmd_send.burst_dir = 1.0f;
        }
        else
        {
            /* S2 中 = 连发(速度模式); S2 上 = 单发(角度模式, 见 shoot.c angle_mode) */
            shoot_cmd_send.load_mode = switch_is_mid(sw2) ? LOAD_BURSTFIRE : LOAD_1_BULLET;

#if RM_FRICTION_WHEEL_ENABLE
            /* 摩擦轮: 左上拨杆(S1)拨到下 -> 开(速度环控速); 其它位置 -> 关 */
            shoot_cmd_send.friction_mode = switch_is_down(sw1) ? FRICTION_ON : FRICTION_OFF;
#else
            shoot_cmd_send.friction_mode = FRICTION_OFF; /* 摩擦轮还没接 */
#endif

            /* 方向: S1 上=顺时针(+1); S1 中=左摇杆(左推=+1, 右推=-1, 中间=0 不转); S1 下=急停 */
            /* 连发开关与方向: 只看【左摇杆】, 与 S1 无关。
             * 摇杆居中 = 0(拨盘不转), 左推 = +1, 右推 = -1。
             * 这样 S1 拨到下(开摩擦轮)时拨盘不会自己转。 */
            if (stick_x < -RC_DEADBAND)
                shoot_cmd_send.burst_dir = 1.0f;
            else if (stick_x > RC_DEADBAND)
                shoot_cmd_send.burst_dir = -1.0f;
            else
                shoot_cmd_send.burst_dir = 0.0f;
        }
    }
}

/**
 * @brief 急停与恢复
 *   S1 打到下 = 全车零输出; 遥控器掉线 = 全车零输出。
 *   恢复条件: S1 回到上/中 且 遥控器在线。
 */
#if RM_BENCH_SHOOT_ENABLE
/**
 * @brief 台架调试: 不用遥控也能测发射机构
 *
 * 按键(PA15)循环切换: 停 -> 逐发 -> 连发 -> 停
 * 摩擦轮强制关闭(没接线也不会报错), 只让拨弹盘动。
 * 上场前把 rm_config.h 的 RM_BENCH_SHOOT_ENABLE 改回 0。
 */
static void BenchShootOverride(float now)
{
    static uint8_t prev_key;
    static uint8_t bench_mode; /* 0=停 1=逐发 2=连发 */
    static float last_key_ms;
    const uint8_t key = BoardKeyPressed(); /* 按下为 1 */

    if (key && !prev_key && (now - last_key_ms) > 200.0f)
    {
        last_key_ms = now;
        bench_mode = (uint8_t)((bench_mode + 1u) % 3u);
        LOGINFO("[bench] load_mode -> %d (%s)", bench_mode,
                (bench_mode == 0) ? "STOP" : (bench_mode == 1) ? "1-BULLET" : "BURST");
    }
    prev_key = key;

    shoot_cmd_send.shoot_mode = SHOOT_ON;
    shoot_cmd_send.friction_mode = FRICTION_OFF; /* 没接摩擦轮 */
    shoot_cmd_send.bullet_speed = SMALL_AMU_15;

    if (bench_mode == 2)
        shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
    else if (bench_mode == 1)
        shoot_cmd_send.load_mode = LOAD_1_BULLET;
    else
        shoot_cmd_send.load_mode = LOAD_STOP;
}
#endif

static void EmergencyHandler()
{
    const uint8_t rc_online = RemoteControlIsOnline();

    /* 急停: 右上拨杆(S2)拨到下 = 全车零输出; 遥控掉线 = 全车零输出
     * (原来用 S1 下做急停, 但 S1 下现在定义为"开摩擦轮", 所以急停挪到 S2 下) */
    if (switch_is_down(rc_data[TEMP].rc.switch_right)
#if !RM_BENCH_SHOOT_ENABLE
        || !rc_online
#endif
    )
    {
        if (robot_state != ROBOT_STOP)
            LOGERROR("[CMD] emergency stop! (rc_online=%d)", rc_online);
        robot_state = ROBOT_STOP;
    }
    else if (robot_state == ROBOT_STOP)
    {
        robot_state = ROBOT_READY;
        LOGINFO("[CMD] reinstate, robot ready");
    }

    if (robot_state == ROBOT_STOP)
    {
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        chassis_cmd_send.vx = 0.0f;
        chassis_cmd_send.vy = 0.0f;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;
    }
    else
    {
        shoot_cmd_send.shoot_mode = SHOOT_ON;
    }
}

/* 机器人核心控制任务, 100Hz */
void RobotCMDTask()
{
    const float now = DWT_GetTimeline_ms();
    float dt = (now - last_tick) * 0.001f;
    if (dt <= 0.0f || dt > 0.1f) /* 第一帧/被拖慢时兜底 */
        dt = 0.01f;
    last_tick = now;

    /* 从其他应用获取回传数据 */
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);
    SubGetMessage(chassis_feed_sub, &chassis_fetch_data);
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);

    CalcOffsetAngle();

    /* 上电后 ~200ms(等 INS 出数据)把云台目标对齐到当前姿态, 避免使能瞬间阶跃 */
    if (!cmd_seeded && ++seed_cnt > 20)
    {
        cmd_seeded = 1;
        gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        gimbal_cmd_send.pitch = gimbal_fetch_data.pitch_abs_deg;
        if (gimbal_cmd_send.pitch > PITCH_CMD_MAX_DEG)
            gimbal_cmd_send.pitch = PITCH_CMD_MAX_DEG;
        if (gimbal_cmd_send.pitch < PITCH_CMD_MIN_DEG)
            gimbal_cmd_send.pitch = PITCH_CMD_MIN_DEG;
        LOGINFO("[CMD] gimbal target aligned to current attitude");
    }

    if (RemoteControlIsOnline())
        RemoteControlSet(dt);

#if RC_PIN_PROBE
    /* 诊断: 每秒统计一次 PD2(UART5_RX) 的电平跳变次数
     *   跳变 >0 说明接收机确实在发(问题在 UART 配置/波特率)
     *   跳变 =0 说明线上没信号(接线/供电/接收机模式) */
    {
        static float probe_tick;
        if ((now - probe_tick) > 1000.0f)
        {
            uint32_t edges = 0, samples = 0, low = 0;
            uint32_t last = (GPIOD->IDR >> 2) & 1u;
            uint32_t t0 = DWT_GetTimeline_us();

            probe_tick = now;
            while ((DWT_GetTimeline_us() - t0) < 100000u) /* 采样 100ms */
            {
                uint32_t cur = (GPIOD->IDR >> 2) & 1u;

                samples++;
                if (cur == 0)
                    low++;
                if (cur != last)
                {
                    edges++;
                    last = cur;
                }
            }
            {
                extern volatile uint32_t rc_dbg_cb;
                extern volatile uint16_t rc_dbg_len;
                extern volatile uint8_t rc_dbg_head[8];
                static uint32_t cb_last;
                static uint16_t cb_rate;

                cb_rate = (uint16_t)(rc_dbg_cb - cb_last);
                cb_last = rc_dbg_cb;
                LOGINFO("[rc] PD2 edges=%d low=%d/%d | cb/s=%d len=%d %02X %02X %02X %02X %02X %02X",
                        (int)edges, (int)low, (int)samples, (int)cb_rate, (int)rc_dbg_len,
                        rc_dbg_head[0], rc_dbg_head[1], rc_dbg_head[2],
                        rc_dbg_head[3], rc_dbg_head[4], rc_dbg_head[5]);
            }
        }
    }
#endif

#if CAN_ID_CENSUS
    {
        static float cen_t;
        if ((now - cen_t) > 1000.0f)
        {
            cen_t = now;
            if (can_census_num > 0)
            {
                LOGINFO("[can] 1s ID census: %d ids", (int)can_census_num);
                for (uint8_t i = 0; i < can_census_num; ++i)
                    LOGINFO("   id 0x%03X : %d frames", (int)can_census_id[i], (int)can_census_cnt[i]);
            }
            for (uint8_t i = 0; i < can_census_num; ++i)
                can_census_cnt[i] = 0;
            can_census_num = 0;
        }
    }
#endif

    EmergencyHandler();

#if RM_BENCH_SHOOT_ENABLE
    BenchShootOverride(now); /* 台架: 覆盖成"发射开 + 按键切模式" */
#endif

    /* 推送消息 */
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
}
