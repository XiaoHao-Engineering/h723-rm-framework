#ifndef BSP_WS2812_H
#define BSP_WS2812_H

#include <stdint.h>

/**
 * @file bsp_ws2812.h
 * @brief 板载 WS2812 彩灯(PA7 / SPI6_MOSI)驱动
 *
 * @note MC02 用 SPI 的 MOSI 驱动 WS2812: 一个 WS2812 位 = 8 个 SPI 位.
 *       SPI6 的时钟源选 HSE(24MHz) / 4 = 6MHz, 一个 WS 位约 1.33us, 满足 WS2812 时序.
 *       发送采用阻塞方式(1 颗灯约 40us), 不占用 DMA, 也不会和串口 DMA 抢资源.
 */

/**
 * @brief 初始化 SPI6 + PA7(由 BSPInit 调用)
 */
void BSPWS2812Init(void);

/* 开机自检: 上电时让彩灯走一遍 红->绿->蓝, 方便确认灯和 SPI6 通路正常.
 * 调试完成后可以改成 0 省掉这 0.36 秒. */
#define WS2812_BOOT_SELFTEST 1

/**
 * @brief 设置板载彩灯颜色
 * @param r/g/b 0~255
 */
void BSPWS2812SetRGB(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief 关闭彩灯
 */
void BSPWS2812Off(void);

#endif /* BSP_WS2812_H */
