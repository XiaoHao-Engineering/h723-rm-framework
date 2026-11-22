/**
 * @file robot.c
 * @brief 应用层总入口: 初始化流程与任务分发
 *
 * 初始化顺序:
 *   BSPInit()  ->  各 app 初始化(注册 module, 此时外设按需初始化)  ->  OSTaskInit()
 * 任务分发:
 *   ROBOT 任务 -> RobotTask() -> 各 app 的 Task 函数
 */

#include "bsp_init.h"
#include "robot.h"
#include "robot_def.h"
#include "robot_task.h"

#include "bsp_log.h"
#include "bsp_board.h"
#include "buzzer.h"
#include "bsp_dwt.h"
#include "bsp_ws2812.h"

#if APP_POWER_BENCH
/* 功率控制实验台 app(实验一: 空载扫转速), 见 robot_def.h 的 APP_POWER_BENCH */
#include "power_bench.h"
#elif USE_RM_EXAMPLE
/* RoboMaster 步兵示例 app(参考代码, 需要裁判系统/底盘/云台等硬件) */
#include "rm_config.h"
#include "chassis.h"
#include "chassis_omni.h"
#include "gimbal.h"
#include "shoot.h"
#include "robot_cmd.h"
#else
/* 达妙 MC02 开发板示例 app(默认) */
#include "board_demo.h"
#endif

#if USE_RM_EXAMPLE
/* 提醒: RM 示例的参数如果不改, 机器人会有致命的控制问题 */
#pragma message "USE_RM_EXAMPLE=1: check the params in robot_def.h and motor ids in rm_example before running!"
#endif

void RobotInit(void)
{
    /* 关闭中断,防止在初始化过程中发生中断
     * 请不要在初始化过程中使用中断和延时函数; 若必须, 只允许使用 DWT_Delay() */
    __disable_irq();

    /* 1. bsp 层: DWT 计时, RTT 日志, 板载 PWM/WS2812/按键/ADC */
    BSPInit();

    /* 2. app 层: 注册 module(此时串口/CAN 等外设按需初始化) */
#if APP_POWER_BENCH
    PowerBenchInit(); // 实验一: 只注册 1 台 M3508(CAN1, 开环电流)
#elif USE_RM_EXAMPLE
    RobotCMDInit(); /* 遥控器/视觉/双板通信 */
    GimbalInit();   /* 云台 */
    ShootInit();    /* 发射机构(含拨弹盘单发) */

    /* 底盘: RM 麦轮示例 或 四轮全向轮(见 rm_config.h 的 RM_USE_OMNI_CHASSIS) */
#if RM_USE_OMNI_CHASSIS
    ChassisOmniInit();
#else
    ChassisInit();
#endif
#else
    BoardDemoInit();
#endif

    /* 3. 创建 rtos 任务(此时还没有开启调度器, 任务不会运行) */
    OSTaskInit();

    /* 初始化完成,开启中断; 调度器在 main() 中启动 */
    LOGINFO("[robot] init done, start scheduler");
    __enable_irq();
}

#if USE_RM_EXAMPLE
/* 开机音乐: 三音上行(G4-A4-E5), 与 demo app 的启动音一致 */
static const BuzzerNote_t rm_boot_music[] = {
    {392, 240},
    {440, 240},
    {655, 300},
};

/**
 * @brief 开机音乐: 只播一次, 等 daemon 任务里 BuzzerInit() 就绪后再触发
 */
static void RMBootMusic(void)
{
    static uint8_t played;

    if (played || !BuzzerIsReady())
        return;

    played = 1;
    BuzzerPlayMelody(rm_boot_music, (uint16_t)ARRAY_LEN(rm_boot_music), 0);
    LOGINFO("[robot] boot music playing");
}

/**
 * @brief 板载彩灯心跳: 1Hz 绿色闪烁, 表示系统在跑(RM app 没有 demo 的灯效)
 *
 * @note 只在 ROBOT 任务(100Hz)里调用, 与具体控制逻辑无关, 出问题也能看出系统是否活着
 */
static void RMLedHeartbeat(void)
{
    static float last_tick;
    static uint8_t on;

    const float now = DWT_GetTimeline_ms();

    if ((now - last_tick) < 500.0f)
        return;

    last_tick = now;
    on = on ? 0u : 1u;
    BSPWS2812SetRGB(0, on ? 60 : 0, 0);
}
#endif /* USE_RM_EXAMPLE */

void RobotTask(void)
{
#if APP_POWER_BENCH
    PowerBenchTask();
#elif USE_RM_EXAMPLE
    RobotCMDTask();
    GimbalTask();
    ShootTask();

#if RM_USE_OMNI_CHASSIS
    ChassisOmniTask();
#else
    ChassisTask();
#endif

    RMBootMusic();    /* 开机音乐(只播一次) */
    RMLedHeartbeat(); /* 1Hz 绿色心跳 */
#else
    BoardDemoTask();
#endif
}
