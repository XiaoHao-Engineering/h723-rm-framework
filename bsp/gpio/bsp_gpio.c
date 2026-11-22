/**
 * @file bsp_gpio.c
 * @brief GPIO/EXTI 板级支持包(H7 版)
 */

#include "bsp_gpio.h"
#include "bsp_log.h"

#include <stdlib.h>
#include <string.h>

static uint8_t idx;
static GPIOInstance *gpio_instance[GPIO_MX_DEVICE_NUM] = {NULL};

/**
 * @brief EXTI中断回调函数,根据GPIO_Pin找到对应的GPIOInstance,并调用模块回调函数(如果有)
 * @note 一个EXTI中断线只能连接一个GPIO引脚, 因此可以通过GPIO_Pin来判断
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    GPIOInstance *gpio;
    for (uint8_t i = 0; i < idx; i++)
    {
        gpio = gpio_instance[i];
        if (gpio->GPIO_Pin == GPIO_Pin && gpio->gpio_model_callback != NULL)
        {
            gpio->pin_state = HAL_GPIO_ReadPin(gpio->GPIOx, gpio->GPIO_Pin);
            gpio->gpio_model_callback(gpio);
            return;
        }
    }
}

GPIOInstance *GPIORegister(GPIO_Init_Config_s *GPIO_config)
{
    if (idx >= GPIO_MX_DEVICE_NUM)
        while (1)
            LOGERROR("[bsp_gpio] GPIO instance exceeded max num!");

    GPIOInstance *ins = (GPIOInstance *)malloc(sizeof(GPIOInstance));
    memset(ins, 0, sizeof(GPIOInstance));

    ins->GPIOx = GPIO_config->GPIOx;
    ins->GPIO_Pin = GPIO_config->GPIO_Pin;
    ins->pin_state = GPIO_config->pin_state;
    ins->exti_mode = GPIO_config->exti_mode;
    ins->id = GPIO_config->id;
    ins->gpio_model_callback = GPIO_config->gpio_model_callback;
    gpio_instance[idx++] = ins;
    return ins;
}

void GPIOToggel(GPIOInstance *_instance)
{
    HAL_GPIO_TogglePin(_instance->GPIOx, _instance->GPIO_Pin);
}

void GPIOSet(GPIOInstance *_instance)
{
    HAL_GPIO_WritePin(_instance->GPIOx, _instance->GPIO_Pin, GPIO_PIN_SET);
}

void GPIOReset(GPIOInstance *_instance)
{
    HAL_GPIO_WritePin(_instance->GPIOx, _instance->GPIO_Pin, GPIO_PIN_RESET);
}

GPIO_PinState GPIORead(GPIOInstance *_instance)
{
    return HAL_GPIO_ReadPin(_instance->GPIOx, _instance->GPIO_Pin);
}
