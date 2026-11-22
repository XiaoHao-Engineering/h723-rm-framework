#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "gpio.h"
#include "stdint.h"

#define GPIO_MX_DEVICE_NUM 16

/**
 * @brief 用于判断中断来源,注意和CUBEMX中配置一致
 */
typedef enum
{
    GPIO_EXTI_MODE_RISING,
    GPIO_EXTI_MODE_FALLING,
    GPIO_EXTI_MODE_RISING_FALLING,
    GPIO_EXTI_MODE_NONE,
} GPIO_EXTI_MODE_e;

/**
 * @brief GPIO实例结构体定义
 */
typedef struct tmpgpio
{
    GPIO_TypeDef *GPIOx;        // GPIOA,GPIOB,GPIOC...
    GPIO_PinState pin_state;    // 引脚状态,Set,Reset;not frequently used
    GPIO_EXTI_MODE_e exti_mode; // 外部中断模式
    uint16_t GPIO_Pin;          // 引脚号(GPIO_PIN_0 ...)

    void (*gpio_model_callback)(struct tmpgpio *); // exti中断回调函数
    void *id;                                      // 区分不同的GPIO实例
} GPIOInstance;

/**
 * @brief GPIO初始化配置结构体定义
 */
typedef struct
{
    GPIO_TypeDef *GPIOx;        // GPIOA,GPIOB,GPIOC...
    GPIO_PinState pin_state;    // 引脚状态,Set,Reset
    GPIO_EXTI_MODE_e exti_mode; // 外部中断模式
    uint16_t GPIO_Pin;          // 引脚号(@note 这里的引脚号是GPIO_PIN_0,GPIO_PIN_1...)

    void (*gpio_model_callback)(GPIOInstance *); // exti中断回调函数
    void *id;                                    // 区分不同的GPIO实例
} GPIO_Init_Config_s;

GPIOInstance *GPIORegister(GPIO_Init_Config_s *GPIO_config);
void GPIOToggel(GPIOInstance *_instance);
void GPIOSet(GPIOInstance *_instance);
void GPIOReset(GPIOInstance *_instance);
GPIO_PinState GPIORead(GPIOInstance *_instance);

#endif /* BSP_GPIO_H */
