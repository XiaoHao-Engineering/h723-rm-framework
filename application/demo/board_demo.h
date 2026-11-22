/**
 * @file board_demo.h
 * @brief 达妙 MC02 开发板示例应用
 *
 * 这是一个"最小可用"的板级示例, 用来验证移植后的三层框架是否工作正常:
 *   - BMI088(SPI2) 姿态解算, 通过 SEGGER RTT 打印欧拉角
 *   - 板载 WS2812 彩灯心跳, 用户按键触发蜂鸣器
 *   - ADC 采集电源电压
 *   - 通过 USART10 以 VOFA+ justfloat 协议输出波形(姿态/陀螺仪/电压/温度)
 *   - 可选: 注册一台达妙电机(CAN1, ID=1) 或 DJI 电机做闭环演示
 *
 * 所有开关都在 application/robot_def.h 的 "达妙板示例 app 配置" 一节.
 */

#ifndef BOARD_DEMO_H
#define BOARD_DEMO_H

/**
 * @brief 示例 app 初始化(注册 module, 需要在实际控制开始前调用)
 */
void BoardDemoInit(void);

/**
 * @brief 示例 app 周期任务, 由 ROBOT 任务以 100Hz 调用
 */
void BoardDemoTask(void);

#endif /* BOARD_DEMO_H */
