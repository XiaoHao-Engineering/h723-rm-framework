/**
 * @file bsp_fault.h
 * @brief 崩溃/异常现场转储(不依赖 HAL 中断、不依赖 RTOS、不依赖 malloc)
 *
 * 为什么要这个模块:
 *   Cortex-M7 出问题时如果只在 HardFault_Handler 里 `while(1)`, 表现就是
 *   "板子突然不动了, 串口没有任何输出, 也没有任何迹象"。这个模块把
 *   **栈帧 + 故障状态寄存器** 用"纯寄存器轮询"的方式打到日志串口上,
 *   于是每次卡死都能看到 "PC 停在哪、是哪种 fault、访问了哪个地址"。
 *
 * 为什么不用 LOGINFO / HAL_UART_Transmit:
 *   1. 故障可能发生在任何地方, 包括"中断关着"、"某个锁没释放"、"DMA 正在用串口";
 *   2. HAL_UART_Transmit 依赖 HAL_GetTick 超时, 而 tick 来自 TIM2 中断,
 *      一旦中断被关掉, 它会永远不返回 —— 在故障处理里等于二次死锁;
 *   3. 所以这里直接写 USART1 的 ISR/TDR 寄存器, 只等 TXE 标志, 不依赖任何中断。
 *
 * 注意: 串口必须是 115200-8N1(和 BSP_LOG_UART 一致), 否则看到的是乱码。
 */

#ifndef BSP_FAULT_H
#define BSP_FAULT_H

#include <stdint.h>

/**
 * @brief 硬件异常(HardFault/MemManage/BusFault/UsageFault)的现场转储
 *
 * @param frame      异常栈帧指针(R0,R1,R2,R3,R12,LR,PC,xPSR), 由 naked 汇编取 MSP/PSP 得到
 * @param exc_return EXC_RETURN(LR), 用来判断用的是 MSP 还是 PSP、有没有 FPU 扩展帧
 * @param ipsr       异常号: 3=HardFault 4=MemManage 5=BusFault 6=UsageFault
 *
 * @note 本函数不返回(转储完直接停机或复位)。
 */
void BSPFaultDump(uint32_t *frame, uint32_t exc_return, uint32_t ipsr);

/**
 * @brief 软件检出的故障(RTOS 断言失败/栈溢出/内存耗尽/Error_Handler)的转储
 *
 * @param tag    故障类型, 例如 "configASSERT"
 * @param detail 附加信息, 例如断言表达式、任务名; 可为 NULL
 *
 * @note 本函数不返回。
 */
void BSPFaultReport(const char *tag, const char *detail);

/**
 * @brief 故障后的"停机"动作: 关中断 + 死循环(调试器仍然可以连上)
 *
 * 如果更希望"自己重启恢复", 把 BSP_FAULT_RESET 设为 1。
 */
void BSPFaultHalt(void) __attribute__((noreturn));

#endif /* BSP_FAULT_H */
