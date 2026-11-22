#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>

/**
 * @file bsp_adc.h
 * @brief ADC 板级支持包: 电池电压采集 + 芯片温度
 *
 * @note MC02 的电压采集电路接在 ADC1_INP4(PC4), 采用 DMA 循环采样, 不占用 CPU.
 *       分压电阻比请在 bsp_board.h 中按实际电路修改(BOARD_VBAT_R1_KOHM / R2).
 */

/**
 * @brief 启动 ADC1 的 DMA 循环采样(由 BSPInit 调用)
 */
void BSPADCInit(void);

/**
 * @brief 读取电池电压原始 ADC 值(16bit, 0~65535)
 */
uint16_t ADCGetVbatRaw(void);

/**
 * @brief 读取换算后的电池电压(单位 V)
 */
float ADCGetVbatVoltage(void);

/**
 * @brief 读取芯片内部温度(单位 ℃, 近似值, 需要按数据手册标定)
 */
float ADCGetChipTemperature(void);

#endif /* BSP_ADC_H */
