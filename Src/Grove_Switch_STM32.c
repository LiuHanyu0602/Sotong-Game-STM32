#include "Grove_Switch_STM32.h"
#include "string.h"

HAL_StatusTypeDef GroveSwitch_Init(GroveSwitch_Handle_t *hsw, I2C_HandleTypeDef *hi2c)
{
    hsw->hi2c = hi2c;
    hsw->dev_addr = GROVE_SWITCH_I2C_ADDR;

    // 简单探测设备是否响应
    if (HAL_I2C_IsDeviceReady(hi2c, hsw->dev_addr, 2, 100) != HAL_OK) {
        return HAL_ERROR;
    }
    return HAL_OK;
}

/* 读取原始状态字节（6字节数据）：
   byte[0..4]：每个按键状态
   byte[5]：事件标志位（见原库定义） */
HAL_StatusTypeDef GroveSwitch_ReadRaw(GroveSwitch_Handle_t *hsw, uint8_t *btn_buf)
{
    return HAL_I2C_Master_Receive(hsw->hi2c, hsw->dev_addr, btn_buf, 6, 100);
}

/* 简化的事件解析：返回哪一键被按下（单击/双击/长按） */
SwitchEvent_t GroveSwitch_GetEvent(GroveSwitch_Handle_t *hsw, uint8_t *btn_pressed)
{
    uint8_t buf[6] = {0};
    if (GroveSwitch_ReadRaw(hsw, buf) != HAL_OK)
        return SW_EVENT_NONE;

    uint8_t evt = buf[5];  // 最后一个字节：事件类型
    *btn_pressed = 0xFF;   // 默认无键

    // 遍历前 5 个按钮（上/下/左/右/中）
    for (uint8_t i = 0; i < 5; i++) {
        if (buf[i] == 1) {
            *btn_pressed = i;
            break;
        }
    }

    switch (evt) {
    case 0x01: return SW_EVENT_SINGLE_CLICK;
    case 0x02: return SW_EVENT_DOUBLE_CLICK;
    case 0x04: return SW_EVENT_LONG_PRESS;
    default:   return SW_EVENT_NONE;
    }
}
