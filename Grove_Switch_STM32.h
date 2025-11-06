#ifndef __GROVE_SWITCH_STM32_H__
#define __GROVE_SWITCH_STM32_H__

#include "stm32l4xx_hal.h"
#include <stdint.h>

/* 默认 I2C 地址 */
#define GROVE_SWITCH_I2C_ADDR   (0x03 << 1)  // STM32 需要左移1位（7bit地址）

/* 按键位标志（button[0..4] 分别代表 上 下 左 右 中） */
#define SW_BTN_UP     0
#define SW_BTN_DOWN   1
#define SW_BTN_LEFT   2
#define SW_BTN_RIGHT  3
#define SW_BTN_CENTER 4

/* 事件类型 */
typedef enum {
    SW_EVENT_NONE = 0,
    SW_EVENT_SINGLE_CLICK,
    SW_EVENT_DOUBLE_CLICK,
    SW_EVENT_LONG_PRESS
} SwitchEvent_t;

/* 全局结构体 */
typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t dev_addr;
} GroveSwitch_Handle_t;

/* 函数声明 */
HAL_StatusTypeDef GroveSwitch_Init(GroveSwitch_Handle_t *hsw, I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef GroveSwitch_ReadRaw(GroveSwitch_Handle_t *hsw, uint8_t *btn_buf);
SwitchEvent_t GroveSwitch_GetEvent(GroveSwitch_Handle_t *hsw, uint8_t *btn_pressed);

#endif
