#ifndef __TIMERONE_STM32_H__
#define __TIMERONE_STM32_H__

#include "stm32l4xx_hal.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    void (*callback)(void);
} TimerOne_STM32_t;

HAL_StatusTypeDef TimerOne_Init(TimerOne_STM32_t *t, TIM_HandleTypeDef *htim, uint32_t us);
void TimerOne_Start(TimerOne_STM32_t *t);
void TimerOne_Stop(TimerOne_STM32_t *t);
uint32_t TimerOne_Read(TimerOne_STM32_t *t);
void TimerOne_SetPeriod(TimerOne_STM32_t *t, uint32_t us);
void TimerOne_AttachInterrupt(TimerOne_STM32_t *t, void (*isr)(void), uint32_t us);
void TimerOne_DetachInterrupt(TimerOne_STM32_t *t);
void TimerOne_PWM(TimerOne_STM32_t *t, uint32_t channel, uint16_t duty, uint32_t period_us);
void TimerOne_DisablePWM(TimerOne_STM32_t *t, uint32_t channel);

#endif
