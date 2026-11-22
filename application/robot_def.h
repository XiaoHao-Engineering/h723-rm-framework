/**
 * @file robot_def.h
 * @brief 机器人/开发板全局配置
 *
 * 这个文件是"整车参数"的唯一入口:
 *   - 选择开发板类型(单板/底盘板/云台板, 仅 RM 示例 app 使用)
 *   - 选择默认运行哪一个应用: 达妙板示例 app(默认) 或 RoboMaster 步兵示例 app
 *   - 云台/底盘/发射机构的机械参数
 *
 * @note 修改本文件后需要重新编译; 与电机 ID、CAN 总线、串口对应关系请到
 *       application/demo/board_demo.c 或 application/rm_example 中修改.
 */

#pragma once
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"

/* ====================== 开发板类型(RM 示例 app 使用) ====================== */
/* 修改定义后需要重新编译, 只能存在一个定义! */
#define ONE_BOARD // 单板控制整车
// #define CHASSIS_BOARD //底盘板
// #define GIMBAL_BOARD  //云台板

/* ====================== 应用选择 ====================== */
/* 0: 默认编译达妙板示例 app(IMU + 蜂鸣器 + LED + VOFA + 电机示例)
 * 1: 编译 RoboMaster 步兵示例 app(参考用, 需要裁判系统/底盘/云台等硬件)
 * 注意: 该宏由 CMake 的 USE_RM_EXAMPLE 选项控制, 这里仅作为默认值参考 */
#ifndef USE_RM_EXAMPLE
#define USE_RM_EXAMPLE 0
#endif

/* ====================== 功率控制实验台(应用选择) ====================== */
/* 1: 开机直接跑 application/power_bench —— 实验一: 空载扫转速(阶梯保持)。
 *    此时不跑 demo app / RM 示例, INS(姿态)任务也会跳过, 因为实验一不需要姿态,
 *    少一个 1kHz 任务、少 12s IMU 标定, 开机更快也更不容易被干扰。
 * 0: 恢复正常 demo / RM 示例。
 *
 * 注意: 置 1 后上电是"待机等按键(PA15)"状态, 电机不动; 按一下才开始给电流。
 *       实验做完记得改回 0。 */
#define APP_POWER_BENCH 0

/* 视觉通信方式(RM 示例 app 使用): MC02 上用 USB 虚拟串口 */
#define VISION_USE_VCP // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

/* ====================== 达妙板示例 app 配置 ====================== */
#define DEMO_DM_MOTOR_ENABLE 0  // 1: 注册一台达妙电机(CAN1, ID=1), 需要实际接好电机
#define DEMO_DJI_MOTOR_ENABLE 0 // 1: 注册一台 DJI 电机(CAN2, ID=1, M3508), 需要实际接好电调
#define DEMO_VOFA_ENABLE 1      // 1: 通过 VOFA+ 上位机查看姿态/电压曲线
#define DEMO_BUZZER_ENABLE 1    // 1: 按键触发蜂鸣器
#define DEMO_LED_ENABLE 1       // 1: 板载 WS2812 呼吸/心跳灯
#define DEMO_REMOTE_ENABLE 1    // 1: 注册 DR16/DT7 接收机(接板载 SBUS 口 / UART5), 需要实际接收机

/* ====================== 机器人重要参数(RM 示例 app) ====================== */
/* 注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 2711  // 云台和底盘对齐指向相同方向时的电机编码器值
#define YAW_ECD_GREATER_THAN_4096 0 // ALIGN_ECD值是否大于4096,是为1,否为0
#define PITCH_HORIZON_ECD 3412      // 云台处于水平位置时编码器值
#define PITCH_MAX_ANGLE 0           // 云台竖直方向最大角度
#define PITCH_MIN_ANGLE 0           // 云台竖直方向最小角度
// 发射参数
#define ONE_BULLET_DELTA_ANGLE 36    // 发射一发弹丸拨盘转动的距离
#define REDUCTION_RATIO_LOADER 49.0f // 拨盘电机的减速比
#define NUM_PER_CIRCLE 10            // 拨盘一圈的装载量
// 底盘参数,单位为mm(毫米)
#define WHEEL_BASE 350              // 纵向轴距
#define TRACK_WIDTH 300             // 横向轮距
#define CENTER_GIMBAL_OFFSET_X 0    // 云台旋转中心距底盘几何中心的距离(前后)
#define CENTER_GIMBAL_OFFSET_Y 0    // 云台旋转中心距底盘几何中心的距离(左右)
#define RADIUS_WHEEL 76             // 轮子半径(mm), 与全向轮 OMNI_WHEEL_R 保持一致
#define REDUCTION_RATIO_WHEEL (268.0f / 17.0f) // 底盘电机减速比(与全向轮一致: 268/17)

#define GYRO2GIMBAL_DIR_YAW 1   // 陀螺仪数据相较于云台的yaw的方向,1为相同,-1为相反
#define GYRO2GIMBAL_DIR_PITCH 1 // 陀螺仪数据相较于云台的pitch的方向
#define GYRO2GIMBAL_DIR_ROLL 1  // 陀螺仪数据相较于云台的roll的方向

// 检查是否出现主控板定义冲突,只允许一个开发板定义存在
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || \
    (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||  \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#pragma pack(1) // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// 底盘模式设置
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,    // 电流零输入
    CHASSIS_ROTATE,            // 小陀螺模式
    CHASSIS_NO_FOLLOW,         // 不跟随，允许全向平移
    CHASSIS_FOLLOW_GIMBAL_YAW, // 跟随模式，底盘叠加角度环控制
} chassis_mode_e;

// 云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0, // 电流零输入
    GIMBAL_FREE_MODE,      // 云台自由运动模式
    GIMBAL_GYRO_MODE,      // 云台陀螺仪反馈模式
} gimbal_mode_e;

// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;

typedef enum
{
    FRICTION_OFF = 0, // 摩擦轮关闭
    FRICTION_ON,      // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LID_OPEN = 0, // 弹舱盖打开
    LID_CLOSE,    // 弹舱盖关闭
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0,  // 停止发射
    LOAD_REVERSE,   // 反转
    LOAD_1_BULLET,  // 单发
    LOAD_3_BULLET,  // 三发
    LOAD_BURSTFIRE, // 连发
} loader_mode_e;

// 功率限制,从裁判系统获取
typedef struct
{ // 功率控制
    float chassis_power_mx;
} Chassis_Power_Data_s;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    float vx;           // 前进方向速度
    float vy;           // 横移方向速度
    float wz;           // 旋转速度
    float offset_angle; // 底盘和归中位置的夹角
    chassis_mode_e chassis_mode;
    int chassis_speed_buff;
} Chassis_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{
    float yaw;
    float pitch;
    float chassis_rotate_wz;
    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    Bullet_Speed_e bullet_speed; // 弹速枚举
    uint8_t rest_heat;
    float shoot_rate; // 连续发射的射频,unit per s,发/秒
    float burst_dir; // 连发方向: +1=顺时针(正转) -1=逆时针 0=手动中位
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
typedef struct
{
    uint8_t rest_heat;           // 剩余枪口热量
    Bullet_Speed_e bullet_speed; // 弹速限制
    Enemy_Color_e enemy_color;   // 0 for blue, 1 for red
} Chassis_Upload_Data_s;

typedef struct
{
    attitude_t gimbal_imu_data;
    float yaw_relative_deg; // 云台相对底盘正方向的夹角(0~360)
    float pitch_abs_deg;    // 炮管绝对仰角(IMU)
    float bigp_deg;         // 大P 电机编码器角度
    float smallp_deg;       // 小P 电机编码器角度
} Gimbal_Upload_Data_s;

typedef struct
{
    float heat_q;              // 当前热量估计(状态机)
    float heat_limit;          // 热量上限 Q0
    float heat_allowed_rps;    // 规划出的允许射频(发/s)
    float heat_cmd_rps;        // 当前连发指令射频(发/s)
    float loader_cmd_rpm;      // 拨弹盘指令转速
    float loader_act_rpm;      // 拨弹盘实测转速
    uint16_t heat_confirmed;   // 已确认发数
    uint8_t heat_state;        // 状态机状态
    uint8_t heat_single_only;  // 1: 只允许单发
    uint8_t heat_blocked;      // 1: 禁止发射
} Shoot_Upload_Data_s;

#pragma pack() // 开启字节对齐

#endif // !ROBOT_DEF_H
