/**
 * @file bsp_ws2812.c
 * @brief 板载 WS2812 彩灯驱动(SPI6 MOSI = PA7)
 *
 * 时序实现参考达妙官方例程 CtrBoard-H7_ALL/Bsp/ws2812.c:
 *   一个 WS2812 位用 8 个 SPI 位表示, 6MHz 下 8 位 = 1.33us
 *     '0' -> 0110 0000 (0x60)  高电平约 0.33us
 *     '1' -> 0111 1000 (0x78)  高电平约 0.67us
 *   数据顺序为 GRB, 高位先发; 发送完成后拉低 >50us 作为复位.
 */

#include "bsp_ws2812.h"
#include "bsp_board.h"
#include "bsp_log.h"

#include "spi.h"

#define WS2812_LED_CNT 1U
#define WS2812_BIT_BYTES (WS2812_LED_CNT * 24U)
#define WS2812_RESET_BYTES 100U

#define WS2812_CODE_LOW  0x60U
#define WS2812_CODE_HIGH 0x78U

static SPI_HandleTypeDef bsp_hspi6;
static uint8_t ws2812_ready = 0; /* SPI6 是否初始化成功 */

/**
 * @brief 由 SPI6 的实际时钟推算分频, 让 8 个 SPI 位 ≈ 1.25~1.9us
 *
 * @note WS2812 的时序只取决于 MOSI 上的位宽, 并不要求精确 6MHz:
 *       只要 SPI 时钟落在 4.2~6.4MHz 就能同时满足 T0H/T1H 的容差.
 *       这样即使 SPI6 的时钟源没切到 HSE(退化成 PCLK4 = 137.5MHz),
 *       也能选到 /32 ≈ 4.3MHz 继续工作, 而不是整体失效(灯全灭).
 */
static uint32_t BSPWS2812PickPrescaler(uint32_t fclk, uint32_t *out_div)
{
    static const uint32_t divs[8] = {2U, 4U, 8U, 16U, 32U, 64U, 128U, 256U};
    static const uint32_t hal[8] = {
        SPI_BAUDRATEPRESCALER_2, SPI_BAUDRATEPRESCALER_4, SPI_BAUDRATEPRESCALER_8,
        SPI_BAUDRATEPRESCALER_16, SPI_BAUDRATEPRESCALER_32, SPI_BAUDRATEPRESCALER_64,
        SPI_BAUDRATEPRESCALER_128, SPI_BAUDRATEPRESCALER_256,
    };

    if (fclk == 0U)
        fclk = 137500000U; /* 兜底: 按 PCLK4(275MHz/2) 估 */

    for (uint8_t i = 0; i < 8U; ++i)
    {
        if ((fclk / divs[i]) <= 6400000U)
        {
            if (out_div != 0)
                *out_div = divs[i];
            return hal[i];
        }
    }

    if (out_div != 0)
        *out_div = 256U;
    return SPI_BAUDRATEPRESCALER_256;
}

void BSPWS2812Init(void)
{
    RCC_PeriphCLKInitTypeDef periph_clk = {0};
    GPIO_InitTypeDef gpio = {0};
    uint32_t spi6_clk;
    uint32_t div = 0U;

    ws2812_ready = 0;

    /* 与官方例程 CtrBoard-H7_WS2812 完全一致: SPI6 时钟源 = HSE(24MHz), 4 分频 = 6MHz
     * (该例程就是在这块板上验证过的配置) */
    periph_clk.PeriphClockSelection = RCC_PERIPHCLK_SPI6;
    periph_clk.Spi6ClockSelection = RCC_SPI6CLKSOURCE_HSE;
    if (HAL_RCCEx_PeriphCLKConfig(&periph_clk) != HAL_OK)
    {
        LOGERROR("[bsp_ws2812] spi6 clock config failed");
        return;
    }

    __HAL_RCC_SPI6_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA7 -> SPI6_MOSI, PA5 -> SPI6_SCK (都是 AF8).
     * SCK 其实不接灯, 但官方例程也是这么配的, 保持一致避免踩未知的坑。 */
    gpio.Pin = GPIO_PIN_5 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF8_SPI6;
    HAL_GPIO_Init(GPIOA, &gpio);

    /* 读回 SPI6 的真实时钟: 如果 HSE 切换失败, 这里会显示 137500000 而不是 24000000,
     * 分频器会自动补偿, 日志里也能直接看出来 */
    spi6_clk = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SPI6);

    bsp_hspi6.Instance = SPI6;
    bsp_hspi6.Init.Mode = SPI_MODE_MASTER;
    bsp_hspi6.Init.Direction = SPI_DIRECTION_2LINES_TXONLY;
    bsp_hspi6.Init.DataSize = SPI_DATASIZE_8BIT;
    bsp_hspi6.Init.CLKPolarity = SPI_POLARITY_LOW;
    /* 相位与官方例程 CtrBoard-H7_WS2812 一致(2EDGE): 8 个 SPI 位 = 1 个 WS2812 位 */
    bsp_hspi6.Init.CLKPhase = SPI_PHASE_2EDGE;
    bsp_hspi6.Init.NSS = SPI_NSS_SOFT;
    bsp_hspi6.Init.BaudRatePrescaler = BSPWS2812PickPrescaler(spi6_clk, &div);
    bsp_hspi6.Init.FirstBit = SPI_FIRSTBIT_MSB;
    bsp_hspi6.Init.TIMode = SPI_TIMODE_DISABLE;
    bsp_hspi6.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    bsp_hspi6.Init.CRCPolynomial = 0;
    bsp_hspi6.Init.NSSPMode = SPI_NSS_PULSE_ENABLE; /* 与官方例程一致 */
    bsp_hspi6.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
    bsp_hspi6.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
    bsp_hspi6.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    bsp_hspi6.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
    bsp_hspi6.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
    bsp_hspi6.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
    bsp_hspi6.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
    bsp_hspi6.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
    bsp_hspi6.Init.IOSwap = SPI_IO_SWAP_DISABLE;

    if (HAL_SPI_Init(&bsp_hspi6) != HAL_OK)
    {
        LOGERROR("[bsp_ws2812] spi6 init failed");
        return;
    }

    ws2812_ready = 1;

    LOGINFO("[bsp_ws2812] ws2812 on PA7(SPI6) init done | spi6=%dHz /%d -> %dMHz",
            (int)spi6_clk, (int)div,
            (int)((div != 0U) ? (spi6_clk / div / 1000000U) : 0U));

    BSPWS2812Off();
}

void BSPWS2812SetRGB(uint8_t r, uint8_t g, uint8_t b)
{
    uint8_t txbuf[WS2812_BIT_BYTES];
    uint8_t zero = 0;

    /* SPI6 没初始化成功就直接返回: 绝不能在初始化失败后死等,
     * 否则整机在最早期就"卡死", 而且没有任何日志 */
    if (!ws2812_ready)
        return;

    /* GRB 顺序, 高位先发 */
    for (uint8_t i = 0; i < 8; ++i)
    {
        txbuf[7U - i] = ((g >> i) & 0x01U) ? WS2812_CODE_HIGH : WS2812_CODE_LOW;
        txbuf[15U - i] = ((r >> i) & 0x01U) ? WS2812_CODE_HIGH : WS2812_CODE_LOW;
        txbuf[23U - i] = ((b >> i) & 0x01U) ? WS2812_CODE_HIGH : WS2812_CODE_LOW;
    }

    /* 等待上一次传输完成: 阻塞发送本身保证传完, 这里只做有限次等待,
     * 避免硬件异常时死循环 */
    for (uint32_t wait = 0; (bsp_hspi6.State != HAL_SPI_STATE_READY) && (wait < 10000U); ++wait)
        ;

    HAL_SPI_Transmit(&bsp_hspi6, txbuf, WS2812_BIT_BYTES, 100);

    /* 复位: 输出低电平 >50us */
    for (uint32_t i = 0; i < WS2812_RESET_BYTES; ++i)
        HAL_SPI_Transmit(&bsp_hspi6, &zero, 1, 100);
}

void BSPWS2812Off(void)
{
    BSPWS2812SetRGB(0, 0, 0);
}
