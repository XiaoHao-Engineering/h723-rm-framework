/**
 * @file bsp_board.c
 * @brief 板级初始化(CubeMX 未覆盖的板载外设)与电源控制
 */

#include "bsp_board.h"
#include "bsp_log.h"
#include "bsp_dwt.h"

/* CubeMX 的 tim.c 只初始化了 TIM3(IMU 加热电阻), 板载 4 路 PWM 与蜂鸣器用的
 * TIM1/TIM5/TIM12 由 bsp 自行初始化. 这样即使以后用 CubeMX 重新生成代码,
 * 也不会把这里的配置覆盖掉. */
TIM_HandleTypeDef bsp_htim1;  /* PE9  / PE13 */
TIM_HandleTypeDef bsp_htim5;  /* PA0  / PA2  */
TIM_HandleTypeDef bsp_htim12; /* PB15        */

/**
 * @brief 配置一个定时器为 PWM 模式(基时)
 */
static void BoardTIMBaseInit(TIM_HandleTypeDef *htim, TIM_TypeDef *instance,
                             uint32_t prescaler, uint32_t period)
{
    TIM_MasterConfigTypeDef master = {0};

    htim->Instance = instance;
    htim->Init.Prescaler = prescaler;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = period;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.RepetitionCounter = 0;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(htim) != HAL_OK)
    {
        LOGERROR("[bsp_board] TIM%d pwm init failed", (int)(instance == TIM1 ? 1 : (instance == TIM5 ? 5 : 12)));
        return;
    }

    master.MasterOutputTrigger = TIM_TRGO_RESET;
    master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(htim, &master);
}

/**
 * @brief 配置 PWM 输出通道(初始占空比 0)
 */
static void BoardTIMChannelInit(TIM_HandleTypeDef *htim, uint32_t channel)
{
    TIM_OC_InitTypeDef oc = {0};

    oc.OCMode = TIM_OCMODE_PWM1;
    oc.Pulse = 0;
    oc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode = TIM_OCFAST_DISABLE;
    oc.OCIdleState = TIM_OCIDLESTATE_RESET;
    oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    if (HAL_TIM_PWM_ConfigChannel(htim, &oc, channel) != HAL_OK)
    {
        LOGERROR("[bsp_board] pwm channel config failed");
    }
}

/**
 * @brief 把 GPIO 复用为定时器输出
 */
static void BoardTIMGpioInit(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;

    /* TIM1_CH1 = PE9, TIM1_CH3 = PE13 */
    gpio.Pin = GPIO_PIN_9 | GPIO_PIN_13;
    gpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOE, &gpio);

    /* TIM5_CH1 = PA0, TIM5_CH3 = PA2 */
    gpio.Pin = GPIO_PIN_0 | GPIO_PIN_2;
    gpio.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* TIM12_CH2 = PB15 (蜂鸣器) */
    gpio.Pin = GPIO_PIN_15;
    gpio.Alternate = GPIO_AF2_TIM12;
    HAL_GPIO_Init(GPIOB, &gpio);
}

/**
 * @brief 计算定时器的计数时钟
 *
 * @note STM32 的定时器时钟来自 APB 总线: APB 预分频为 1 时等于 PCLK,
 *       否则是 PCLK 的 2 倍。用运行时读取的值而不是写死频率,
 *       这样把 CPU 拉到 550MHz(HPRE=/2, 总线仍 275MHz) 时这里也不会算错。
 */
static uint32_t BoardTimerClock(uint32_t pclk, uint32_t hclk)
{
    return (pclk == hclk) ? pclk : (pclk * 2U);
}

void BoardPWMHardwareInit(void)
{
    /* 让定时器以 1MHz 计数(周期单位直接等于 us), 方便伺服/蜂鸣器使用 */
    uint32_t hclk = HAL_RCC_GetHCLKFreq();
    uint32_t tim1_clk = BoardTimerClock(HAL_RCC_GetPCLK2Freq(), hclk); /* TIM1 在 APB2 */
    uint32_t tim5_clk = BoardTimerClock(HAL_RCC_GetPCLK1Freq(), hclk); /* TIM5/TIM12 在 APB1 */
    uint32_t psc_apb2 = tim1_clk / 1000000U - 1U;
    uint32_t psc_apb1 = tim5_clk / 1000000U - 1U;

    BoardTIMGpioInit();

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_TIM12_CLK_ENABLE();

    /* 舵机 PWM 默认 50Hz(20ms), 蜂鸣器默认 4kHz(250us) */
    BoardTIMBaseInit(&bsp_htim1, TIM1, psc_apb2, 20000U - 1U);
    BoardTIMChannelInit(&bsp_htim1, TIM_CHANNEL_1);
    BoardTIMChannelInit(&bsp_htim1, TIM_CHANNEL_3);

    BoardTIMBaseInit(&bsp_htim5, TIM5, psc_apb1, 20000U - 1U);
    BoardTIMChannelInit(&bsp_htim5, TIM_CHANNEL_1);
    BoardTIMChannelInit(&bsp_htim5, TIM_CHANNEL_3);

    BoardTIMBaseInit(&bsp_htim12, TIM12, psc_apb1, 250U - 1U);
    BoardTIMChannelInit(&bsp_htim12, TIM_CHANNEL_2);

    LOGINFO("[bsp_board] board pwm(tim1/tim5/tim12) init done, tim clk=%dMHz",
            (int)(tim5_clk / 1000000U));
}

TIM_HandleTypeDef *BoardPWMGetHandle(BoardPWMChannel_e ch, uint32_t *channel_out)
{
    uint32_t channel = TIM_CHANNEL_1;
    TIM_HandleTypeDef *htim = NULL;

    switch (ch)
    {
    case BOARD_PWM_1:
        htim = &bsp_htim1;
        channel = TIM_CHANNEL_1;
        break;
    case BOARD_PWM_2:
        htim = &bsp_htim1;
        channel = TIM_CHANNEL_3;
        break;
    case BOARD_PWM_3:
        htim = &bsp_htim5;
        channel = TIM_CHANNEL_1;
        break;
    case BOARD_PWM_4:
        htim = &bsp_htim5;
        channel = TIM_CHANNEL_3;
        break;
    case BOARD_PWM_BUZZER:
        htim = &bsp_htim12;
        channel = TIM_CHANNEL_2;
        break;
    case BOARD_PWM_IMU_HEATER:
        htim = BOARD_TIM_IMU_HEATER;
        channel = BOARD_IMU_HEATER_CH;
        break;
    default:
        LOGERROR("[bsp_board] illegal pwm channel [%d]", (int)ch);
        break;
    }

    if (channel_out != NULL)
        *channel_out = channel;
    return htim;
}

void BoardPower5V(uint8_t enable)
{
    /* PC15 是 5V 输出(DC5)的 PMOS 使能脚, **高电平有效**:
     *   PC15 = 1 → 5V 输出打开
     *   PC15 = 0 → 5V 输出关闭
     *
     * 依据达妙官方例程 dm02_test 的 User_File/2_Device/BSP/Power/bsp_power.cpp:
     *   HAL_GPIO_WritePin(DC5__OUTPUT_GPIO_Port, DC5__OUTPUT_Pin,
     *                     __DC5_Status ? GPIO_PIN_SET : GPIO_PIN_RESET);
     * 即 Status=true 时输出高电平。
     */
    /* 使能 GPIO 时钟, 保证即使跳过 MX_GPIO_Init(最小自检固件)也能工作 */
    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(POWER_5V_GPIO_Port, POWER_5V_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void BoardPowerOut(uint8_t ch, uint8_t enable)
{
    /* PC14(OUT_EN1) / PC13(OUT_EN2) 与 DC5 一致: 高电平使能(见 dm02_test 的 Set_DC24_x) */
    GPIO_PinState state = enable ? GPIO_PIN_SET : GPIO_PIN_RESET;

    if (ch == 1)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, state); /* OUT_EN1 */
    else if (ch == 2)
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, state); /* OUT_EN2 */
    else
        LOGERROR("[bsp_board] illegal power output channel [%d]", (int)ch);
}

void BoardKeyInit(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    gpio.Pin = BOARD_KEY_PIN;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(BOARD_KEY_PORT, &gpio);
}

uint8_t BoardKeyPressed(void)
{
    return (HAL_GPIO_ReadPin(BOARD_KEY_PORT, BOARD_KEY_PIN) == GPIO_PIN_RESET) ? 1 : 0;
}

void BoardBeepStart(uint32_t freq_hz)
{
    uint32_t channel = TIM_CHANNEL_2;
    TIM_HandleTypeDef *htim = BoardPWMGetHandle(BOARD_PWM_BUZZER, &channel);
    uint32_t arr;

    if (htim == NULL || freq_hz == 0U)
        return;

    /* 定时器已被 BoardPWMHardwareInit() 配成 1MHz 计数, 所以 ARR = 1e6/f - 1,
     * 占空比 50% 时无源蜂鸣器最响(与官方 BUZZER 例程的 TIM12->CCR2 写法等价) */
    arr = 1000000U / freq_hz;
    if (arr < 2U)
        arr = 2U;

    __HAL_TIM_SET_AUTORELOAD(htim, arr - 1U);
    __HAL_TIM_SET_COMPARE(htim, channel, arr / 2U);
    htim->Instance->EGR = TIM_EGR_UG; /* 让新的 ARR 立即生效 */

    HAL_TIM_PWM_Start(htim, channel);
}

void BoardBeepStop(void)
{
    uint32_t channel = TIM_CHANNEL_2;
    TIM_HandleTypeDef *htim = BoardPWMGetHandle(BOARD_PWM_BUZZER, &channel);

    if (htim == NULL)
        return;

    __HAL_TIM_SET_COMPARE(htim, channel, 0U);
}

void BoardBeep(uint32_t freq_hz, uint32_t duration_ms)
{
    if (duration_ms == 0U)
        return;

    BoardBeepStart(freq_hz);

    /* 这里必须用 DWT 计时, 不能用 HAL_GetTick()/HAL_Delay():
     *   RobotInit() 一开始就 __disable_irq(), 而 BSPInit() 整段都跑在关中断区域里,
     *   此时 TIM2 中断不会触发, uwTick 永远不涨, 用 HAL_GetTick() 等时间会**死锁**。
     *   DWT 的 CYCCNT 由内核周期计数器提供, 与中断无关, 才是这里唯一安全的时间源。
     *   (DWT_Delay 内部有防死等保护, 即使 CYCCNT 异常也不会卡住系统) */
    DWT_Delay((float)duration_ms / 1000.0f);

    BoardBeepStop();
}
