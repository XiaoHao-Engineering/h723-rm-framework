/**
 * @file bsp_adc.c
 * @brief ADC 板级支持包(H7 版)
 *
 * 电压采集: ADC1(PC4 / ADC1_INP4) 由 CubeMX 配置为 "连续转换 + DMA 循环",
 *           这里只需要给出 DMA 缓冲区并启动一次, 之后随时读缓冲区即可.
 * 芯片温度: ADC3 内部温度传感器, 采用阻塞单次转换(不需要 DMA).
 */

#include "bsp_adc.h"
#include "bsp_board.h"
#include "bsp_log.h"

#include <string.h>

/* ADC1 有两个转换 rank(ioc 中两个 rank 都指向通道4), DMA 循环写入该缓冲区.
 * 注意: 必须放在 DMA 可访问的 RAM 中(DTCM 不行). */
static uint16_t adc1_dma_buffer[2] DMA_BUFFER;
static uint8_t adc_ready = 0;

void BSPADCInit(void)
{
    memset(adc1_dma_buffer, 0, sizeof(adc1_dma_buffer));

    if (HAL_ADC_Start_DMA(BOARD_ADC_VBAT, (uint32_t *)adc1_dma_buffer,
                          (uint32_t)ARRAY_LEN(adc1_dma_buffer)) != HAL_OK)
    {
        LOGERROR("[bsp_adc] ADC1 dma start failed");
        return;
    }

    adc_ready = 1;
    LOGINFO("[bsp_adc] ADC1(vbat) dma sampling started");
}

uint16_t ADCGetVbatRaw(void)
{
    return adc1_dma_buffer[0];
}

float ADCGetVbatVoltage(void)
{
    const float vref = BOARD_VBAT_ADV_REF;
    const float full_scale = 65535.0f; /* ADC1 为 16bit 分辨率 */
    const float ratio = (BOARD_VBAT_R1_KOHM + BOARD_VBAT_R2_KOHM) / BOARD_VBAT_R2_KOHM;

    if (!adc_ready)
        return 0.0f;

    return ((float)adc1_dma_buffer[0] / full_scale) * vref * ratio;
}

float ADCGetChipTemperature(void)
{
    /* H7 内部温度传感器: 12bit 分辨率(ADC3), 转换公式见参考手册
     *   T = (V30 - V_sense) / Avg_Slope + 30
     * 具体数值请以数据手册的电气特性表为准, 这里给出典型值. */
    const float v30 = 0.62f;      /* V, 30℃ 时的典型输出电压 */
    const float avg_slope = 0.0025f; /* V/℃ */
    uint32_t raw = 0;

    if (HAL_ADC_Start(BOARD_ADC_TEMP) != HAL_OK)
        return 0.0f;

    if (HAL_ADC_PollForConversion(BOARD_ADC_TEMP, 10) == HAL_OK)
        raw = HAL_ADC_GetValue(BOARD_ADC_TEMP);

    HAL_ADC_Stop(BOARD_ADC_TEMP);

    float v_sense = ((float)raw / 4095.0f) * BOARD_VBAT_ADV_REF;
    return (v30 - v_sense) / avg_slope + 30.0f;
}
