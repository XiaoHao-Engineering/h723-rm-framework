#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "tim.h"
#include "stdint.h"

#define PWM_DEVICE_CNT 16 // 最大支持的PWM实例数量

/* pwm实例结构体 */
typedef struct pwm_ins_temp
{
    TIM_HandleTypeDef *htim;                 // TIM句柄
    uint32_t channel;                        // 通道
    uint32_t tclk;                           // 定时器输入时钟频率(Hz)
    float period;                            // 周期(s)
    float dutyratio;                         // 占空比
    void (*callback)(struct pwm_ins_temp *); // DMA传输完成回调函数
    void *id;                                // 实例ID
} PWMInstance;

typedef struct
{
    TIM_HandleTypeDef *htim;        // TIM句柄
    uint32_t channel;               // 通道
    float period;                   // 周期(s)
    float dutyratio;                // 占空比
    void (*callback)(PWMInstance *); // DMA传输完成回调函数
    void *id;                       // 实例ID
} PWM_Init_Config_s;

/**
 * @brief 注册一个pwm实例(会立刻启动该通道)
 */
PWMInstance *PWMRegister(PWM_Init_Config_s *config);

void PWMStart(PWMInstance *pwm);
void PWMStop(PWMInstance *pwm);

/**
 * @brief 设置pwm占空比
 * @param dutyratio 占空比 0~1
 */
void PWMSetDutyRatio(PWMInstance *pwm, float dutyratio);

/**
 * @brief 设置pwm周期(会自动调整预分频, 保证 ARR 不超过 16 位)
 * @param period 周期 单位 s
 */
void PWMSetPeriod(PWMInstance *pwm, float period);

/**
 * @brief 启动pwm dma传输(如 WS2812 / 多路伺服)
 * @note 数据位宽必须和 CubeMX / bsp 中配置的 DMA 传输位宽一致
 */
void PWMStartDMA(PWMInstance *pwm, uint32_t *pData, uint32_t Size);

#endif // BSP_PWM_H
