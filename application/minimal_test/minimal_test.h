/**
 * @file minimal_test.h
 * @brief 最小自检固件(用于硬件/启动问题定位)
 *
 * 只做四件事, 其它全部跳过(USB / ADC / FDCAN / USART / FreeRTOS / IMU):
 *   1. 打开 5V 输出(PC15, 低有效)
 *   2. 蜂鸣器(PB15 / TIM12_CH2)循环响: 2kHz / 100ms, 每 2 秒一次
 *   3. 板载 WS2812(PA7 / SPI6)循环换色: 红 -> 绿 -> 蓝 -> 白, 每 500ms
 *   4. 不启动调度器, 直接在 main 的 while(1) 里跑
 *
 * 用法(在 CMake 里打开开关):
 *   cmake --preset Debug -DUSE_MINIMAL_TEST=ON
 *   cmake --build --preset Debug
 *   .\.flash\flash.ps1 -NoBuild
 *
 * 判断方法:
 *   - 听到"滴" + 看到灯换色  -> 硬件、时钟、5V、SPI、PWM 全部正常,
 *                               问题在框架的初始化流程里, 再逐段打开定位;
 *   - 完全没反应            -> 硬件/供电/烧录问题, 与框架代码无关。
 */

#ifndef MINIMAL_TEST_H
#define MINIMAL_TEST_H

/**
 * @brief 最小自检初始化(在 main() 的 USER CODE 2 里调用)
 */
void MinimalTestInit(void);

/**
 * @brief 最小自检主循环(在 main() 的 while(1) 里调用)
 */
void MinimalTestLoop(void);

#endif /* MINIMAL_TEST_H */
