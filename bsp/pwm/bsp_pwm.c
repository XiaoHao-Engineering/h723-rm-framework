/**
 * @file bsp_pwm.c
 * @brief PWM 板级支持包(H7 版)
 *
 * @note 与 F4 版相比:
 *  1. 定时器时钟改为按 APB 预分频自动推算(H7 的 APB 预分频表与 F4 不同, 直接抄会算错);
 *  2. PWMSetPeriod 会自动选择预分频, 不再会因为 ARR 超过 16 位而溢出;
 *  3. 所有比较值都用 __HAL_TIM_SET_COMPARE 宏, 兼容 H7 的 TIM1/TIM5/TIM12.
 */

#include "bsp_pwm.h"
#include "bsp_log.h"

#include <stdlib.h>
#include <string.h>

// 配合中断以及初始化
static uint8_t idx;
static PWMInstance *pwm_instance[PWM_DEVICE_CNT] = {NULL};

/**
 * @brief 取得定时器的计数时钟频率
 * @note  APB 预分频为 1 时 TIMxCLK = PCLKx, 否则 TIMxCLK = 2 * PCLKx
 */
static uint32_t PWMGetTimerClock(TIM_HandleTypeDef *htim)
{
    uint32_t hclk = HAL_RCC_GetHCLKFreq();
    uint32_t pclk;
    TIM_TypeDef *tim = htim->Instance;

    /* APB2 上的定时器: TIM1, TIM8, TIM15~17; 其余在 APB1 上 */
    if (tim == TIM1 || tim == TIM8 || tim == TIM15 || tim == TIM16 || tim == TIM17)
        pclk = HAL_RCC_GetPCLK2Freq();
    else
        pclk = HAL_RCC_GetPCLK1Freq();

    return (pclk == hclk) ? pclk : (pclk * 2U);
}

/**
 * @brief pwm dma传输完成回调函数
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    for (uint8_t i = 0; i < idx; i++)
    {
        if (pwm_instance[i]->htim == htim && htim->Channel == (1U << (pwm_instance[i]->channel / 4U)))
        {
            if (pwm_instance[i]->callback)
                pwm_instance[i]->callback(pwm_instance[i]);
            return;
        }
    }
}

PWMInstance *PWMRegister(PWM_Init_Config_s *config)
{
    if (idx >= PWM_DEVICE_CNT) // 超过最大实例数,考虑增加或查看是否有内存泄漏
        while (1)
            LOGERROR("[bsp_pwm] PWM instance exceeded max num!");

    PWMInstance *pwm = (PWMInstance *)malloc(sizeof(PWMInstance));
    memset(pwm, 0, sizeof(PWMInstance));

    pwm->htim = config->htim;
    pwm->channel = config->channel;
    pwm->period = config->period;
    pwm->dutyratio = config->dutyratio;
    pwm->callback = config->callback;
    pwm->id = config->id;
    pwm->tclk = PWMGetTimerClock(pwm->htim);

    HAL_TIM_PWM_Start(pwm->htim, pwm->channel);
    PWMSetPeriod(pwm, pwm->period);
    PWMSetDutyRatio(pwm, pwm->dutyratio);

    pwm_instance[idx++] = pwm;
    return pwm;
}

void PWMStart(PWMInstance *pwm)
{
    HAL_TIM_PWM_Start(pwm->htim, pwm->channel);
}

void PWMStop(PWMInstance *pwm)
{
    HAL_TIM_PWM_Stop(pwm->htim, pwm->channel);
}

void PWMSetPeriod(PWMInstance *pwm, float period)
{
    if (period <= 0.0f)
        return;

    /* 以 1MHz 为计数基准: ARR 直接等于周期(us), 便于伺服/蜂鸣器使用 */
    uint32_t psc = pwm->tclk / 1000000U;
    if (psc == 0)
        psc = 1;

    float ticks = period * 1000000.0f; /* 1MHz 下的计数值 */
    uint32_t arr = (uint32_t)(ticks + 0.5f);

    /* 16 位定时器的 ARR 上限, 超了就加大预分频 */
    while (arr > 0xFFFFU && psc < 0xFFFFU)
    {
        psc <<= 1;
        arr >>= 1;
    }
    if (arr == 0)
        arr = 1;

    __HAL_TIM_SET_PRESCALER(pwm->htim, psc - 1U);
    __HAL_TIM_SET_AUTORELOAD(pwm->htim, arr - 1U);

    /* 让新的预分频/重装载值立刻生效 */
    pwm->htim->Instance->EGR = TIM_EGR_UG;

    pwm->period = period;
}

void PWMSetDutyRatio(PWMInstance *pwm, float dutyratio)
{
    if (dutyratio < 0.0f)
        dutyratio = 0.0f;
    if (dutyratio > 1.0f)
        dutyratio = 1.0f;

    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(pwm->htim);
    __HAL_TIM_SET_COMPARE(pwm->htim, pwm->channel, (uint32_t)(dutyratio * (float)(arr + 1U)));

    pwm->dutyratio = dutyratio;
}

void PWMStartDMA(PWMInstance *pwm, uint32_t *pData, uint32_t Size)
{
    HAL_TIM_PWM_Start_DMA(pwm->htim, pwm->channel, pData, Size);
}
