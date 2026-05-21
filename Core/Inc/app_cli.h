#ifndef APP_CLI_H
#define APP_CLI_H

#include <stdint.h>
#include "stm32l4xx_hal.h"

void App_CLI_Init(UART_HandleTypeDef *uart);
void App_CLI_Poll(void);
void App_CLI_OnRxComplete(UART_HandleTypeDef *uart);

void App_UART_SendString(const char *s);
void App_UART_Printf(const char *fmt, ...);
void App_UART_PrintfNoDisplay(const char *fmt, ...);

extern uint8_t g_uart_rx_byte;

#endif /* APP_CLI_H */
