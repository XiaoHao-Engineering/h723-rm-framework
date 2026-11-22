#include "bsp_log.h"

#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"

/* bsp_board.h 中定义了 BSP_LOG_UART(串口日志开关) */
#include "bsp_board.h"

/* 串口日志输出(可选): 在 bsp_board.h 中定义 BSP_LOG_UART 即可开启 */
#if defined(BSP_LOG_UART)
static char log_uart_buf[160];
#endif

void BSPLogInit(void)
{
    SEGGER_RTT_Init();
}

void BSPLogOutput(const char *color, const char *type, const char *format, ...)
{
    va_list args;

    /* ---------------- 1. SEGGER RTT(带颜色) ---------------- */
    SEGGER_RTT_printf(BUFFER_INDEX, "  %s%s", color ? color : "", type ? type : "");
    va_start(args, format);
    SEGGER_RTT_vprintf(BUFFER_INDEX, format, &args);
    va_end(args);
    SEGGER_RTT_WriteString(BUFFER_INDEX, "\r\n" RTT_CTRL_RESET);

    /* ---------------- 2. 串口(纯文本, 无颜色转义) ---------------- */
#if defined(BSP_LOG_UART)
    /* 保护: 最小自检固件会跳过 MX_USARTx_Init(), 此时 huart->Instance 还是空,
     * 直接调用 HAL_UART_Transmit 会去解引用空指针。 */
    /* 关键保护: 如果当前处于"关中断"状态(例如 BSPInit 在 RobotInit 的
     * __disable_irq() 区域内运行), 就不能用 HAL_UART_Transmit 写串口 ——
     * 它靠 HAL_GetTick 超时, 而 tick 来自 TIM2 中断, 中断关了就永远不返回。
     * 这种情况下只走 RTT(不依赖中断), 保证日志绝不会把启动流程卡死。 */
    /* 再补一条: **中断上下文里一律不写串口**。
     * HAL_UART_Transmit 是阻塞忙等(一条日志最长 20ms), 在 1kHz 的 CAN 中断里
     * 打一行日志 = 把 CPU 的 40% 花在串口上, 而且中断风暴时会把整条日志输出
     * 拖成"滚雪球"——这是很多"莫名其妙卡死/卡顿"的真实原因。
     * ISR 里只写 RTT(内存拷贝, 不阻塞), 串口留给任务上下文。 */
    if ((BSP_LOG_UART->Instance != 0) &&
        ((__get_PRIMASK() & 1U) == 0U) &&
        (__get_IPSR() == 0U))
    {
        int n = 0;
        int len;

        log_uart_buf[n++] = ' ';
        log_uart_buf[n++] = ' ';
        if (type != NULL)
        {
            len = (int)strlen(type);
            if (len > (int)sizeof(log_uart_buf) - 8)
                len = (int)sizeof(log_uart_buf) - 8;
            memcpy(&log_uart_buf[n], type, len);
            n += len;
        }

        va_start(args, format);
        len = vsnprintf(&log_uart_buf[n], sizeof(log_uart_buf) - n - 3, format, args);
        va_end(args);

        if (len < 0)
            len = 0;
        if (len > (int)sizeof(log_uart_buf) - n - 3)
            len = (int)sizeof(log_uart_buf) - n - 3;

        n += len;
        log_uart_buf[n++] = '\r';
        log_uart_buf[n++] = '\n';

        HAL_UART_Transmit(BSP_LOG_UART, (uint8_t *)log_uart_buf, (uint16_t)n, 20);
    }
#endif
}

int PrintLog(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    int n = SEGGER_RTT_vprintf(BUFFER_INDEX, fmt, &args); // 一次可以开启多个buffer(多个终端),我们只用一个
    va_end(args);
    return n;
}

void Float2Str(char *str, float va)
{
    int flag = va < 0;
    int head = (int)va;
    int point = (int)((va - head) * 1000);
    head = abs(head);
    point = abs(point);
    if (flag)
        sprintf(str, "-%d.%d", head, point);
    else
        sprintf(str, "%d.%d", head, point);
}
