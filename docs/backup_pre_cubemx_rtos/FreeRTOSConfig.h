/* --------------------------------------------------------------------------
 * FreeRTOSConfig.h
 * 针对 STM32H723 + Cortex-M7 的 FreeRTOS 配置(基于 F4 版本框架的配置迁移)
 *
 * 与 F4 版本的区别:
 *   - configTOTAL_HEAP_SIZE 适当增大(module 层实例较多)
 *   - 明确关闭了 timers/coroutines, 打开 task notification
 *   - 内核时钟直接取 SystemCoreClock(275MHz)
 *   - 中断优先级分组由 CubeMX 配置为 NVIC_PRIORITYGROUP_4(4 位抢占优先级)
 * ----------------------------------------------------------------------- */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* Ensure definitions are only used by the compiler, and not by the assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
#include <stdint.h>
extern uint32_t SystemCoreClock;
#endif

#define configENABLE_FPU 1
#define configENABLE_MPU 0

#define configUSE_PREEMPTION                   1
#define configSUPPORT_STATIC_ALLOCATION        1
#define configSUPPORT_DYNAMIC_ALLOCATION       1
#define configUSE_IDLE_HOOK                    0
#define configUSE_TICK_HOOK                    0
#define configCPU_CLOCK_HZ                     (SystemCoreClock)
#define configTICK_RATE_HZ                     ((TickType_t)1000)
#define configMAX_PRIORITIES                   (7)
#define configMINIMAL_STACK_SIZE               ((uint16_t)128)
#define configTOTAL_HEAP_SIZE                  ((size_t)(32 * 1024))
#define configMAX_TASK_NAME_LEN                (32)
#define configUSE_16_BIT_TICKS                 0
#define configUSE_MUTEXES                      1
#define configQUEUE_REGISTRY_SIZE              8
#define configUSE_COUNTING_SEMAPHORES          1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TASK_NOTIFICATIONS           1
#define configUSE_TIMERS                       0
#define configMESSAGE_BUFFER_LENGTH_TYPE       size_t

/* ---- 健壮性开关(排查"卡死"必开) --------------------------------------
 * configCHECK_FOR_STACK_OVERFLOW = 2: 每次任务切换都检查栈指针有没有越界,
 *   越界立刻调用 vApplicationStackOverflowHook() 打印信息, 而不是"跑飞了
 *   之后在别的地方莫名其妙死掉"。
 * configUSE_MALLOC_FAILED_HOOK = 1: pvPortMalloc 失败时调用
 *   vApplicationMallocFailedHook()。本工程电机/串口实例都是 malloc 出来的,
 *   堆只有 32KB, 加设备加多了很容易在这里耗尽。
 * 两个钩子都实现在 application/robot_task.c。 */
#define configCHECK_FOR_STACK_OVERFLOW         2
#define configUSE_MALLOC_FAILED_HOOK           1

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES                  0
#define configMAX_CO_ROUTINE_PRIORITIES        (2)

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet               1
#define INCLUDE_uxTaskPriorityGet              1
#define INCLUDE_vTaskDelete                    1
#define INCLUDE_vTaskCleanUpResources          0
#define INCLUDE_vTaskSuspend                   1
#define INCLUDE_vTaskDelayUntil                1
#define INCLUDE_vTaskDelay                     1
#define INCLUDE_xTaskGetSchedulerState         1

/* Cortex-M specific definitions. */
#ifdef __NVIC_PRIO_BITS
/* __NVIC_PRIO_BITS will be specified when CMSIS is being used. */
#define configPRIO_BITS __NVIC_PRIO_BITS
#else
#define configPRIO_BITS 4
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority"
function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15

/* The highest interrupt priority that can be used by any interrupt service
routine that makes calls to interrupt safe FreeRTOS API functions. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

/* Interrupt priorities used by the kernel port layer itself. */
#define configKERNEL_INTERRUPT_PRIORITY (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* Normal assert() semantics without relying on the provision of an assert.h header file. */
extern void BSPFaultReport(const char *tag, const char *detail);

/* 原来的实现是 taskDISABLE_INTERRUPTS() + for(;;) —— 一旦断言失败(最常见的是
 * "在优先级高于 configMAX_SYSCALL_INTERRUPT_PRIORITY 的中断里调用了 FreeRTOS API",
 * 或者在调度器启动前调用), 板子就**静默卡死**, 串口一个字都没有。
 * 现在改成先把断言表达式打到串口上再停机, 至少知道"哪一句断言挂了"。 */
#define configASSERT(x) \
    if ((x) == 0)       \
    {                   \
        BSPFaultReport("configASSERT", #x); \
    }

/* Definitions that map the FreeRTOS port interrupt handlers to their CMSIS
standard names. */
#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler

/* IMPORTANT: This define is commented when used with STM32Cube firmware, when the timebase source is SysTick,
              to prevent overwriting SysTick_Handler defined within STM32Cube HAL */
#define xPortSysTickHandler SysTick_Handler

#endif /* FREERTOS_CONFIG_H */
