/**
 * @file bsp_init.c
 * @brief bsp 层统一初始化
 */

#include "bsp_init.h"
#include "bsp_board.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_usb.h"
#include "bsp_adc.h"
#include "bsp_ws2812.h"

/* CubeMX 已经在 main() 中调用了 MX_xxx_Init(), 这里只初始化框架需要、
 * 而 CubeMX 工程里没有配置的部分(板载PWM、WS2812、ADC 采样启动等). */
void BSPInit(void)
{
    /* 0. 日志最先初始化, 这样后面任何一步卡住都能看到最后一条日志 */
    BSPLogInit();
    LOGINFO("[boot] 1/6 bsp init start");

    /* 1. DWT 精确计时: DWT 的 CYCCNT 数的是 **CPU 时钟 (cpu_ck)**,
     *    在 H72x/H73x 上 cpu_ck 可能高于 AXI/HCLK(例如 550/275),
     *    所以必须用 SystemCoreClock(=cpu_ck) 而不是 HCLK。 */
    DWT_Init((uint32_t)(SystemCoreClock / 1000000U));

    /* 2. 打开 5V 输出(PC15, 高有效, 见 dm02_test 的 bsp_power.cpp)
     *    CubeMX 的 gpio.c 初始化时已经把它拉高, 这里再显式调一次, 保证顺序正确。 */
    BoardPower5V(1);
    DWT_Delay(0.01f); /* 等 5V 稳定 */
    LOGINFO("[boot] 2/6 5V output enabled (PC15=1)");

    /* 3. 板载 PWM/蜂鸣器/加热电阻等定时器 */
    BoardPWMHardwareInit();
    LOGINFO("[boot] 3/6 board pwm ok");

    /* 3.1 蜂鸣器自检: 上电立刻"滴"一声(约 20ms 后就能听到).
     *     它只依赖 5V + TIM12 + 蜂鸣器, 不依赖 SPI/彩灯/RTOS,
     *     所以"听到滴"就说明 5V 和定时器这一路是通的. */
    BoardBeep(2000, 60);
    LOGINFO("[boot] 3/6 buzzer self-test done");

    /* 4. 板载 WS2812 彩灯(SPI6) 与用户按键 */
    BSPWS2812Init();
    BoardKeyInit();
    LOGINFO("[boot] 4/6 ws2812 + key ok");

    /* 5. ADC: 启动电池电压的 DMA 循环采样 */
    BSPADCInit();
    LOGINFO("[boot] 5/6 adc ok");

    LOGINFO("[boot] 6/6 %s (%s) bsp init done | SYS=%dMHz HCLK=%dMHz PCLK1=%dMHz CPU=%dMHz",
            BOARD_NAME, BOARD_MCU_NAME,
            (int)(HAL_RCC_GetSysClockFreq() / 1000000U),
            (int)(HAL_RCC_GetHCLKFreq() / 1000000U),
            (int)(HAL_RCC_GetPCLK1Freq() / 1000000U),
            (int)(SystemCoreClock / 1000000U));
}
