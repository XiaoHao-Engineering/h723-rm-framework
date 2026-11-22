/**
 * @file minimal_test.c
 * @brief 零依赖定位固件: 只用 GPIO + CPU 忙等循环
 *
 * 为什么这么写:
 *   前面几版自检都依赖了某种"时间基准"(DWT 的 CYCCNT 或 HAL 的 TIM2 时基),
 *   一旦那个基准没跑起来, 整个自检第一行就卡住, 表现为"灯不亮、也不响",
 *   完全看不出卡在哪。这一版**不依赖任何定时器/中断/DWT/RTOS/SPI 初始化**,
 *   用"蜂鸣器响几声"来报告启动进度:
 *
 *     上电后:  3 声短鸣   -> 固件已进入主循环, CPU/时钟/GPIO 都正常
 *              然后每 1 秒 1 声短鸣(心跳) -> 主循环在跑
 *              同时尝试用 SPI6 给彩灯换色 -> 听到声但灯不换 = SPI/灯的问题
 *
 * 判读:
 *   - 听到声音: 固件确实在跑, 问题在后面的 SPI6/灯 或 框架初始化;
 *   - 完全没有声音: 固件没跑起来(或烧录失败), 或者蜂鸣器/5V 硬件问题。
 *
 * 蜂鸣器/彩灯引脚: PB15(蜂鸣器) / PA7(2812 数据线), 与手册一致。
 */

#include "minimal_test.h"

#include "bsp_board.h"
#include "bsp_ws2812.h"

/* ---------------- 纯 CPU 忙等(不依赖任何定时器) ---------------- */

/**
 * @brief 粗略的微秒延时: 275MHz 下每次空循环约 5 个周期, 1us 约 55 次
 * @note  只用于让蜂鸣器"听起来是某个音高", 不需要精确
 */
static void MTBusyUs(uint32_t us)
{
    for (volatile uint32_t i = 0; i < (us * 55U); ++i)
    {
        __NOP();
    }
}

static void MTBusyMs(uint32_t ms)
{
    MTBusyUs(ms * 1000U);
}

/**
 * @brief 用 PB15 位翻转发出一个方波(阻塞)
 * @note  PB15 此时被配置为普通推挽输出(不是 TIM12 复用), 所以不依赖定时器
 */
static void MTBeepBusy(uint32_t freq_hz, uint32_t ms)
{
    uint32_t half_us;
    uint32_t periods;

    if (freq_hz == 0U)
        return;

    half_us = 500000U / freq_hz; /* 半个周期(us) */
    periods = (ms * freq_hz) / 1000U;

    for (uint32_t i = 0; i < periods; ++i)
    {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
        MTBusyUs(half_us);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
        MTBusyUs(half_us);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
}

/**
 * @brief 只初始化用得上的 GPIO: PC15(5V) / PB15(蜂鸣器) / PA7(留给 WS2812)
 */
static void MTGpioInit(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;

    /* 5V 使能: PC15 高电平 = 打开(依据 dm02_test 的 bsp_power.cpp) */
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Pin = POWER_5V_Pin;
    HAL_GPIO_WritePin(POWER_5V_GPIO_Port, POWER_5V_Pin, GPIO_PIN_SET);
    HAL_GPIO_Init(POWER_5V_GPIO_Port, &gpio);

    /* 蜂鸣器: PB15 普通输出, 先拉低 */
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Pin = GPIO_PIN_15;
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
    HAL_GPIO_Init(GPIOB, &gpio);
}

void MinimalTestInit(void)
{
    /* 注意: 这里刻意不调用 BSPLogInit / DWT_Init / HAL_Delay,
     *       因为它们分别依赖 RTT、DWT 计数、TIM2 时基, 任何一个没起来都会卡死。 */
    MTGpioInit();
}

void MinimalTestLoop(void)
{
    static uint8_t stage = 0;
    static uint8_t color = 0;
    static const uint8_t colors[4][3] = {{40, 0, 0}, {0, 40, 0}, {0, 0, 40}, {40, 40, 40}};

    if (stage == 0)
    {
        stage = 1;

        /* ① 3 声短鸣: 证明固件已进入主循环, CPU/时钟/GPIO/5V 正常 */
        MTBeepBusy(2000, 60);
        MTBusyMs(120);
        MTBeepBusy(2000, 60);
        MTBusyMs(120);
        MTBeepBusy(2000, 60);
        MTBusyMs(300);

        /* ② 再初始化 SPI6 点灯(如果这里卡住, 上面的 3 声已经听到了) */
        BSPWS2812Init();
        BSPWS2812SetRGB(colors[0][0], colors[0][1], colors[0][2]);
    }

    /* ③ 心跳: 每 1 秒一声短鸣 + 换一次灯色 */
    MTBeepBusy(2000, 30);
    MTBusyMs(270);

    color = (uint8_t)((color + 1U) & 0x03U);
    BSPWS2812SetRGB(colors[color][0], colors[color][1], colors[color][2]);

    MTBusyMs(700);
}
