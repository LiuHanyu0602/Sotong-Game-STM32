#ifndef APP_IO_H
#define APP_IO_H

#include <stdint.h>
#include "stm32l4xx_hal.h"

extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim2;

void UartClock_UseHSI_ForUSART1(void);
void MX_GPIO_Init(void);
void MX_USART1_UART_Init(void);
void MX_I2C1_Init(void);
void MX_IR_GPIO_Init(void);

void App_IO_I2C1TryRecover(void);

void App_LED_On(void);
void App_LED_Off(void);
void App_LED_Toggle(void);
void App_LED_BlinkStep(uint32_t period_ms);

uint8_t App_IR_IsMovementDetected(void);

#endif /* APP_IO_H */
