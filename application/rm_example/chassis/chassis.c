/**
 * @file chassis.c
 * @brief 底盘应用: 4 x M3508 全向轮(十字布局), 运动学与电机注册都在 chassis_omni 里
 *
 * 这一层只做三件事:
 *   1. 订阅 cmd 发来的底盘指令(vx/vy/wz + 模式);
 *   2. 把"云台系速度"旋到底盘系(offset_angle), 再按模式决定 wz;
 *   3. 把速度交给 ChassisOmniSetVelocity(), 由它做逆解、限幅、轮速环参考。
 *
 * 注意: 这一版没有裁判系统 / 超级电容 / 底盘功率控制, 台架调试用。
 *       上整车后如果要功率限制, 在 chassis_omni.h 里改 OMNI_POWER_MAX。
 */

#include "chassis.h"
#include "robot_def.h"
#include "rm_config.h"

#include "chassis_omni.h"
#include "message_center.h"
#include "general_def.h"
#include "bsp_board.h"

#include <math.h>

static Publisher_t *chassis_pub;            // 用于发布底盘的数据
static Subscriber_t *chassis_sub;           // 用于订阅底盘的控制命令
static Chassis_Ctrl_Cmd_s chassis_cmd_recv; // 底盘接收到的控制命令
static Chassis_Upload_Data_s chassis_feedback_data;

/* 把角度归一化到 [-180, 180] */
static float Wrap180(float deg)
{
    while (deg > 180.0f)
        deg -= 360.0f;
    while (deg < -180.0f)
        deg += 360.0f;
    return deg;
}

void ChassisInit()
{
    /* 4 x M3508(全向轮) 在 CAN1 上注册, 见 chassis_omni.c */
    ChassisOmniInit();

    chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_Ctrl_Cmd_s));
    chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));

    LOGINFO("[chassis] omni 4-wheel chassis init");
}

/* 机器人底盘控制核心任务 */
void ChassisTask()
{
    SubGetMessage(chassis_sub, &chassis_cmd_recv);

    /* ---------------- 1. 按模式决定自转速度 ---------------- */
    float wz_cmd = chassis_cmd_recv.wz;

    switch (chassis_cmd_recv.chassis_mode)
    {
    case CHASSIS_ZERO_FORCE: // 急停: 四轮零输出
        ChassisOmniStop();
        PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
        return;

    case CHASSIS_NO_FOLLOW: // 不跟随: 保持全向平移, 不转
        wz_cmd = 0.0f;
        break;

    case CHASSIS_ROTATE: // 小陀螺: 固定角速度旋转
        wz_cmd = RC_WZ_MAX;
        break;

    case CHASSIS_FOLLOW_GIMBAL_YAW: // 跟随云台: 按角度偏差闭环
    {
        const float err = Wrap180(chassis_cmd_recv.offset_angle);
        if (fabsf(err) < CHASSIS_FOLLOW_DEADBAND_DEG)
            wz_cmd = 0.0f;
        else
        {
            /* 偏差越大转得越快, 平方项让近距离不过冲 */
            wz_cmd = -CHASSIS_FOLLOW_GAIN * err * fabsf(err) / 180.0f;
            if (wz_cmd > CHASSIS_FOLLOW_MAX_W)
                wz_cmd = CHASSIS_FOLLOW_MAX_W;
            if (wz_cmd < -CHASSIS_FOLLOW_MAX_W)
                wz_cmd = -CHASSIS_FOLLOW_MAX_W;
        }
        break;
    }

    default:
        wz_cmd = 0.0f;
        break;
    }

    /* ---------------- 2. 云台系速度 -> 底盘系 ---------------- */
    /* 云台指向的方向为 x 正方向, 底盘逆时针为正:
     * 把摇杆给的(vx,vy)按 offset_angle 旋转到底盘坐标系 */
    const float th = chassis_cmd_recv.offset_angle * DEGREE_2_RAD;
    const float c = cosf(th);
    const float s = sinf(th);
    const float chassis_vx = chassis_cmd_recv.vx * c - chassis_cmd_recv.vy * s;
    const float chassis_vy = chassis_cmd_recv.vx * s + chassis_cmd_recv.vy * c;

    /* ---------------- 3. 解算 + 下发 ---------------- */
    ChassisOmniSetVelocity(chassis_vx, chassis_vy, wz_cmd);
    ChassisOmniTask(); // 功率限幅 + 把轮速参考交给电机模块

    PubPushMessage(chassis_pub, (void *)&chassis_feedback_data);
}
