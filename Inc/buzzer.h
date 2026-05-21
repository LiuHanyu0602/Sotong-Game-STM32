#ifndef __BUZZER_H__
#define __BUZZER_H__

#include "stm32l4xx_hal.h"

#define BUZZER_GPIO_PORT   GPIOB
#define BUZZER_PIN         GPIO_PIN_2

#ifdef __cplusplus
extern "C" {
#endif

void Buzzer_Init(void);
void Buzzer_On(void);
void Buzzer_Off(void);

/* 非阻塞蜂鸣：开 ms 毫秒，真正的关由 Buzzer_Task() 来做 */
void Buzzer_Beep(uint32_t ms);

/* 在主循环里周期性调用，负责把到期的蜂鸣关掉 */
void Buzzer_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __BUZZER_H__ */
