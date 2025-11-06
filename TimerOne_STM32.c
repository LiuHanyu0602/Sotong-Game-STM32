#include "TimerOne_STM32.h"

static uint32_t us_to_arr(TIM_HandleTypeDef *htim, uint32_t us)
{
    uint32_t pclk = HAL_RCC_GetPCLK1Freq();
    uint32_t prescaler = htim->Init.Prescaler + 1;
    return (pclk / prescaler / 1000000U) * us;
}

HAL_StatusTypeDef TimerOne_Init(TimerOne_STM32_t *t, TIM_HandleTypeDef *htim, uint32_t us)
{
    t->htim = htim;
    t->callback = NULL;

    __HAL_TIM_DISABLE(htim);
    __HAL_TIM_SET_COUNTER(htim, 0);
    __HAL_TIM_SET_AUTORELOAD(htim, us_to_arr(htim, us));
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);
    return HAL_OK;
}

void TimerOne_Start(TimerOne_STM32_t *t)
{
    HAL_TIM_Base_Start_IT(t->htim);
}

void TimerOne_Stop(TimerOne_STM32_t *t)
{
    HAL_TIM_Base_Stop_IT(t->htim);
}

uint32_t TimerOne_Read(TimerOne_STM32_t *t)
{
    return __HAL_TIM_GET_COUNTER(t->htim);
}

void TimerOne_SetPeriod(TimerOne_STM32_t *t, uint32_t us)
{
    __HAL_TIM_SET_AUTORELOAD(t->htim, us_to_arr(t->htim, us));
}

void TimerOne_AttachInterrupt(TimerOne_STM32_t *t, void (*isr)(void), uint32_t us)
{
    t->callback = isr;
    TimerOne_SetPeriod(t, us);
    HAL_TIM_Base_Start_IT(t->htim);
}

void TimerOne_DetachInterrupt(TimerOne_STM32_t *t)
{
    HAL_TIM_Base_Stop_IT(t->htim);
    t->callback = NULL;
}

void TimerOne_PWM(TimerOne_STM32_t *t, uint32_t channel, uint16_t duty, uint32_t period_us)
{
    TIM_OC_InitTypeDef sConfigOC = {0};
    uint32_t arr = us_to_arr(t->htim, period_us);
    __HAL_TIM_SET_AUTORELOAD(t->htim, arr);
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = (arr * duty) / 100;   // duty: 0~100%
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(t->htim, &sConfigOC, channel);
    HAL_TIM_PWM_Start(t->htim, channel);
}

void TimerOne_DisablePWM(TimerOne_STM32_t *t, uint32_t channel)
{
    HAL_TIM_PWM_Stop(t->htim, channel);
}

/* 全局回调 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    extern TimerOne_STM32_t Timer1;
    if (htim == Timer1.htim && Timer1.callback) {
        Timer1.callback();
    }
}

/* 定义全局实例 */
//TimerOne_STM32_t Timer1;
