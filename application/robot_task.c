/**
 * @file robot_task.c
 * @brief 所有 RTOS 任务的创建与任务体
 *
 * 任务划分(与 F4 版本的框架保持一致):
 *   INSTASK   1kHz   读取 BMI088 并做四元数姿态解算
 *   MOTORTASK 1kHz   所有电机的串级 PID 计算与 CAN 报文发送
 *   DAEMONTASK 100Hz 守护线程(模块掉线检测) + 蜂鸣器
 *   ROBOTTASK 100Hz  机器人核心控制(示例 app) 或 RM 示例 app
 */

#include "robot_task.h"

#include "bsp_dwt.h"
#include "bsp_fault.h"

#if USE_RM_EXAMPLE
#include "master_process.h" // VisionSend
#include "referee_task.h"
#include "referee_UI.h"
#include "HT04.h"
#include "rm_config.h" // RM_USE_REFEREE
#endif

osThreadId_t insTaskHandle;
osThreadId_t motorTaskHandle;
osThreadId_t daemonTaskHandle;
osThreadId_t robotTaskHandle;
#if USE_RM_EXAMPLE
osThreadId_t uiTaskHandle;
#endif

static void StartINSTASK(void *argument);
static void StartMOTORTASK(void *argument);
static void StartDAEMONTASK(void *argument);
static void StartROBOTTASK(void *argument);
#if USE_RM_EXAMPLE && RM_USE_REFEREE
static void StartUITASK(void *argument);
#endif

/* 任务属性(CMSIS-RTOS v2): stack_size 单位是字节 -> 1024 words = 4096 bytes */
static const osThreadAttr_t ins_task_attr = {
    .name = "instask", .priority = osPriorityAboveNormal, .stack_size = 4096};
static const osThreadAttr_t motor_task_attr = {
    .name = "motortask", .priority = osPriorityNormal, .stack_size = 2048};
static const osThreadAttr_t daemon_task_attr = {
    .name = "daemontask", .priority = osPriorityNormal, .stack_size = 1024};
static const osThreadAttr_t robot_task_attr = {
    .name = "robottask", .priority = osPriorityNormal, .stack_size = 4096};
#if USE_RM_EXAMPLE && RM_USE_REFEREE
static const osThreadAttr_t ui_task_attr = {
    .name = "uitask", .priority = osPriorityNormal, .stack_size = 2048};
#endif

void OSTaskInit(void)
{
    /* v2: stack_size 单位是字节(v1 是字), 已按 x4 换算 */
    insTaskHandle = osThreadNew(StartINSTASK, NULL, &ins_task_attr);
    motorTaskHandle = osThreadNew(StartMOTORTASK, NULL, &motor_task_attr);
    daemonTaskHandle = osThreadNew(StartDAEMONTASK, NULL, &daemon_task_attr);
    robotTaskHandle = osThreadNew(StartROBOTTASK, NULL, &robot_task_attr);

#if USE_RM_EXAMPLE && RM_USE_REFEREE
    HTMotorControlInit(); // 没有注册 HT 电机则不会执行

    /* 裁判系统 UI 刷新任务 */
    uiTaskHandle = osThreadNew(StartUITASK, NULL, &ui_task_attr);
#endif
}

/* --------------------------------------------------------------------------
 * 任务超时告警(限速 1Hz)
 *
 * 1kHz 的任务一旦被拖慢, "每毫秒打一行" 会让情况变得更糟:
 * 一行 ~60 字节 × 1000 = 60kB/s, 115200 的串口根本发不出去(而且 HAL_UART_Transmit
 * 是阻塞忙等), 于是 "任务慢 -> 日志多 -> 串口更堵 -> 任务更慢" 形成正反馈,
 * 最后整块板子像死机一样。所以这里统一限速。
 * ------------------------------------------------------------------------ */
static void TaskDelayLog(const char *task_name, float dt_ms)
{
    static float last_log = 0.0f;
    float now = DWT_GetTimeline_ms();

    if (now - last_log < 1000.0f)
        return;
    last_log = now;

    LOGERROR("[freeRTOS] %s Task is being DELAY! dt = [%d]us (log rate-limited)",
             task_name, (int)(dt_ms * 1000));
}

__attribute__((noreturn)) static void StartINSTASK(void *argument)
{
#if APP_POWER_BENCH
    /* 功率控制实验台不开姿态: 跳过 BMI088 初始化(最长 12s 标定)与 1kHz 解算,
     * 让 CPU 全部留给电机任务和串口, 也避免 IMU 标定期间被轮子震动打断。 */
    (void)argument;
    LOGINFO("[freeRTOS] INS Task disabled (APP_POWER_BENCH=1)");
    for (;;)
        osDelay(1000);
#else
    static float ins_start;
    static float ins_dt;

    (void)argument;
    INS_Init(); // 确保 BMI088 被正确初始化
    LOGINFO("[freeRTOS] INS Task Start");

    for (;;)
    {
        ins_start = DWT_GetTimeline_ms();
        INS_Task();
        ins_dt = DWT_GetTimeline_ms() - ins_start;
        if (ins_dt > 1)
            TaskDelayLog("INS", ins_dt);

#if USE_RM_EXAMPLE && RM_USE_VISION
        VisionSend(); // 解算完成后发送视觉数据
#endif
        osDelay(1); // 1kHz
    }
#endif
}

__attribute__((noreturn)) static void StartMOTORTASK(void *argument)
{
    static float motor_start;
    static float motor_dt;

    (void)argument;
    LOGINFO("[freeRTOS] MOTOR Task Start");

    for (;;)
    {
        motor_start = DWT_GetTimeline_ms();
        MotorControlTask();
        motor_dt = DWT_GetTimeline_ms() - motor_start;
        if (motor_dt > 1)
            TaskDelayLog("MOTOR", motor_dt);
        osDelay(1); // 1kHz
    }
}

__attribute__((noreturn)) static void StartDAEMONTASK(void *argument)
{
    static float daemon_start;
    static float daemon_dt;

    (void)argument;
    BuzzerInit();
    LOGINFO("[freeRTOS] Daemon Task Start");

    for (;;)
    {
        daemon_start = DWT_GetTimeline_ms();
        DaemonTask();
        BuzzerTask();
        BuzzerMusicTask(); /* 推进旋律播放(10ms 一格) */
        daemon_dt = DWT_GetTimeline_ms() - daemon_start;
        if (daemon_dt > 10)
            TaskDelayLog("Daemon", daemon_dt);
        osDelay(10); // 100Hz
    }
}

__attribute__((noreturn)) static void StartROBOTTASK(void *argument)
{
    static float robot_start;
    static float robot_dt;

    (void)argument;
    LOGINFO("[freeRTOS] ROBOT core Task Start");

    for (;;)
    {
        robot_start = DWT_GetTimeline_ms();
        RobotTask();
        robot_dt = DWT_GetTimeline_ms() - robot_start;
        if (robot_dt > 10)
            TaskDelayLog("ROBOT core", robot_dt);
        osDelay(10); // 100Hz
    }
}

#if USE_RM_EXAMPLE && RM_USE_REFEREE
__attribute__((noreturn)) static void StartUITASK(void *argument)
{
    (void)argument;
    LOGINFO("[freeRTOS] UI Task Start");
    MyUIInit();
    LOGINFO("[freeRTOS] UI Init Done, communication with ref has established");
    for (;;)
    {
        UITask();
        osDelay(1);
    }
}
#endif

/* --------------------------------------------------------------------------
 * FreeRTOS 静态分配支持(FreeRTOSConfig.h 中 configSUPPORT_STATIC_ALLOCATION = 1)
 * 空闲任务的内存由用户提供, 否则链接时会报 vApplicationGetIdleTaskMemory 未定义.
 * ------------------------------------------------------------------------ */
static StaticTask_t idleTaskTCB;
static StackType_t idleTaskStack[configMINIMAL_STACK_SIZE];

/* timers=1 且开了静态分配时, 定时器任务的内存也要我们提供,
 * 否则链接会报 vApplicationGetTimerTaskMemory 未定义 */
void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize)
{
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[configTIMER_TASK_STACK_DEPTH];

    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}

void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
    *ppxIdleTaskTCBBuffer = &idleTaskTCB;
    *ppxIdleTaskStackBuffer = idleTaskStack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/* --------------------------------------------------------------------------
 * FreeRTOS 健壮性钩子(见 FreeRTOSConfig.h 的 configCHECK_FOR_STACK_OVERFLOW
 * 与 configUSE_MALLOC_FAILED_HOOK)
 *
 * 这两个问题以前的表现都是"板子跑着跑着就不动了, 没有任何提示":
 *   - 任务栈溢出会踩坏相邻任务/堆, 现场早已面目全非;
 *   - 堆耗尽时 pvPortMalloc 返回 NULL, 随后某个实例指针是空, 再一解引用就 HardFault。
 * 现在它们都会先打印现场(任务名/断言信息)再停机。
 * ------------------------------------------------------------------------ */
/* 栈溢出 / 堆耗尽 两个 hook 已移到 Core/Src/freertos.c 的 USER CODE 4/5 区
 * (CubeMX 会在那里生成同名空函数, 放在一起才不会有重复定义) */
