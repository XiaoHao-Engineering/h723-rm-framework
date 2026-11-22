/**
 * @file bsp_fault.c
 * @brief 崩溃现场转储的实现: 纯寄存器轮询写 USART1, 不依赖 HAL / 中断 / RTOS
 */

#include "bsp_fault.h"
#include "bsp_board.h"

#include <string.h>

/* 转储完的处理: 1 = 自动复位(机器人自己恢复), 0 = 停机(方便接调试器看现场) */
#ifndef BSP_FAULT_RESET
#define BSP_FAULT_RESET 0
#endif

#if defined(BSP_LOG_UART)
#define FAULT_HUART (BSP_LOG_UART)
#else
#define FAULT_HUART ((UART_HandleTypeDef *)0)
#endif

#ifndef USART_ISR_TXE_TXFNF
#define USART_ISR_TXE_TXFNF (USART_ISR_TXE_TXFNF_Msk)
#endif

/* ------------------------------------------------------------------ 输出 */

static USART_TypeDef *f_uart(void)
{
#if defined(BSP_LOG_UART)
    /* huart1 是静态分配的句柄, 它的地址永远不为 NULL; 真正表示"串口还没初始化"
     * 的是 Instance 字段(CubeMX 在 MX_USART1_UART_Init 里才赋值)。 */
    return FAULT_HUART->Instance;
#else
    return NULL;
#endif
}

/**
 * @brief 轮询 TXE 写一个字节
 * @note  带 guard: 即使串口没初始化/波特率没配好, 也绝不会把故障处理卡在这里
 */
static void f_putc(char c)
{
    USART_TypeDef *u = f_uart();
    uint32_t guard = 0U;

    if (u == NULL)
        return;

    while (((u->ISR & USART_ISR_TXE_TXFNF) == 0U) && (++guard < 2000000U))
        ;

    u->TDR = (uint32_t)(uint8_t)c;
}

static void f_str(const char *s)
{
    if (s == NULL)
        return;
    while (*s)
        f_putc(*s++);
}

static void f_hex(uint32_t v)
{
    static const char d[] = "0123456789ABCDEF";
    int i;

    f_str("0x");
    for (i = 28; i >= 0; i -= 4)
        f_putc(d[(v >> i) & 0xFU]);
}

static void f_dec(int32_t v)
{
    char buf[12];
    int i = 0;
    uint32_t u;

    if (v < 0)
    {
        f_putc('-');
        u = (uint32_t)(-v);
    }
    else
        u = (uint32_t)v;

    if (u == 0U)
    {
        f_putc('0');
        return;
    }

    while (u > 0U && i < (int)sizeof(buf))
    {
        buf[i++] = (char)('0' + (u % 10U));
        u /= 10U;
    }
    while (i > 0)
        f_putc(buf[--i]);
}

/* --------------------------------------------------------------- 位域解码 */

static void f_cfsr_bits(uint32_t cfsr)
{
    /* MemManage (bit0~7) */
    if (cfsr & (1UL << 0))  f_str(" IACCVIOL");
    if (cfsr & (1UL << 1))  f_str(" DACCVIOL");
    if (cfsr & (1UL << 3))  f_str(" MUNSTKERR");
    if (cfsr & (1UL << 4))  f_str(" MSTKERR");
    if (cfsr & (1UL << 5))  f_str(" MLSPERR");
    if (cfsr & (1UL << 7))  f_str(" MMARVALID");
    /* BusFault (bit8~15) */
    if (cfsr & (1UL << 8))  f_str(" IBUSERR");
    if (cfsr & (1UL << 9))  f_str(" PRECISERR");
    if (cfsr & (1UL << 10)) f_str(" IMPRECISERR");
    if (cfsr & (1UL << 11)) f_str(" UNSTKERR");
    if (cfsr & (1UL << 12)) f_str(" STKERR");
    if (cfsr & (1UL << 13)) f_str(" LSPERR");
    if (cfsr & (1UL << 15)) f_str(" BFARVALID");
    /* UsageFault (bit16~31) */
    if (cfsr & (1UL << 16)) f_str(" UNDEFINSTR");
    if (cfsr & (1UL << 17)) f_str(" INVSTATE");
    if (cfsr & (1UL << 18)) f_str(" INVPC");
    if (cfsr & (1UL << 19)) f_str(" NOCP");
    if (cfsr & (1UL << 24)) f_str(" UNALIGNED");
    if (cfsr & (1UL << 25)) f_str(" DIVBYZERO");
}

static void f_hfsr_bits(uint32_t hfsr)
{
    if (hfsr & (1UL << 1))  f_str(" VECTTBL");
    if (hfsr & (1UL << 30)) f_str(" FORCED");
    if (hfsr & (1UL << 31)) f_str(" DEBUGEVT");
}

static const char *f_ipsr_name(uint32_t ipsr)
{
    switch (ipsr)
    {
    case 2:  return "NMI";
    case 3:  return "HardFault";
    case 4:  return "MemManage";
    case 5:  return "BusFault";
    case 6:  return "UsageFault";
    case 11: return "SVCall";
    case 14: return "PendSV";
    case 15: return "SysTick";
    default: return "Exception";
    }
}

/* --------------------------------------------------------------- 转储主体 */

static void f_header(void)
{
    f_str("\r\n\r\n");
    f_str("========== BSP FAULT DUMP ==========\r\n");
}

static void f_tail(void)
{
    f_str("------------------------------------\r\n");
    f_str("提示: PC 那一行是出错指令, 用\r\n");
    f_str("  arm-none-eabi-addr2line -f -e build/Debug/RM.elf <PC>\r\n");
    f_str("就能定位到源文件和行号。\r\n");
    f_str("====================================\r\n");
}

static void f_hw_state(void)
{
    f_str("CFSR    : "); f_hex(SCB->CFSR);  f_str("  "); f_cfsr_bits(SCB->CFSR);  f_str("\r\n");
    f_str("HFSR    : "); f_hex(SCB->HFSR);  f_str("  "); f_hfsr_bits(SCB->HFSR);  f_str("\r\n");
    f_str("DFSR    : "); f_hex(SCB->DFSR);  f_str("\r\n");
    f_str("MMFAR   : "); f_hex(SCB->MMFAR); f_str("\r\n");
    f_str("BFAR    : "); f_hex(SCB->BFAR);  f_str("\r\n");
    f_str("MSP     : "); f_hex(__get_MSP()); f_str("\r\n");
    f_str("PSP     : "); f_hex(__get_PSP()); f_str("\r\n");
    f_str("IPSR    : "); f_dec((int32_t)__get_IPSR()); f_str("\r\n");
}

void BSPFaultDump(uint32_t *frame, uint32_t exc_return, uint32_t ipsr)
{
    f_header();

    f_str("cause   : "); f_str(f_ipsr_name(ipsr));
    f_str(" (ipsr="); f_dec((int32_t)ipsr); f_str(")\r\n");

    if (frame != NULL)
    {
        f_str("R0      : "); f_hex(frame[0]); f_str("  R1 : "); f_hex(frame[1]); f_str("\r\n");
        f_str("R2      : "); f_hex(frame[2]); f_str("  R3 : "); f_hex(frame[3]); f_str("\r\n");
        f_str("R12     : "); f_hex(frame[4]); f_str("\r\n");
        f_str("LR      : "); f_hex(frame[5]); f_str("\r\n");
        f_str("PC      : "); f_hex(frame[6]); f_str("   <== 出错指令地址\r\n");
        f_str("xPSR    : "); f_hex(frame[7]); f_str("\r\n");
    }

    f_str("EXC_RET : "); f_hex(exc_return);
    f_str("  (bit2=1 用 PSP / bit4=0 有 FPU 扩展帧)\r\n");

    f_hw_state();
    f_tail();

    BSPFaultHalt();
}

void BSPFaultReport(const char *tag, const char *detail)
{
    f_header();

    f_str("cause   : "); f_str(tag ? tag : "software fault"); f_str("\r\n");
    if (detail != NULL)
    {
        f_str("detail  : "); f_str(detail); f_str("\r\n");
    }

    /* 调用点(不精确, 但足够定位到"是哪一类操作触发的") */
    f_str("return  : "); f_hex((uint32_t)(uintptr_t)__builtin_return_address(0)); f_str("\r\n");

    f_hw_state();
    f_tail();

    BSPFaultHalt();
}

void BSPFaultHalt(void)
{
#if BSP_FAULT_RESET
    f_str("reset in 200ms ...\r\n");
    {
        /* 用 DWT 而不是 HAL_Delay: 故障场景下 tick 可能已经停了 */
        uint32_t start = DWT->CYCCNT;
        volatile uint32_t wait = 0U;
        while ((DWT->CYCCNT - start) < (SystemCoreClock / 5U) && (wait < 200000000U))
            wait++;
    }
    __DSB();
    NVIC_SystemReset();
#endif

    __disable_irq();
    for (;;)
    {
        __NOP();
    }
}
