#ifndef __SEEEDOLED_H__
#define __SEEEDOLED_H__

#include "stm32l4xx_hal.h"

/* SSD130x / Seeed OLED 基本参数 */
#define SEEEDOLED_MAX_X              127
#define SEEEDOLED_MAX_Y              63

/* 模式 */
#define SEEEDOLED_PAGE_MODE          0x01
#define SEEEDOLED_HORIZONTAL_MODE    0x02

/* I2C 地址（7bit=0x3C → HAL 要左移一位） */
#define SEEEDOLED_I2C_ADDR           (0x3C << 1)

/* 控制字节 */
#define SEEEDOLED_CMD_MODE           0x80
#define SEEEDOLED_DATA_MODE          0x40

/* 基本命令 */
#define SEEEDOLED_DISPLAY_OFF        0xAE
#define SEEEDOLED_DISPLAY_ON         0xAF
#define SEEEDOLED_NORMAL_DISPLAY     0xA6
#define SEEEDOLED_INVERSE_DISPLAY    0xA7
#define SEEEDOLED_SET_BRIGHTNESS     0x81

#ifdef __cplusplus
extern "C" {
#endif

/* === 对外 API === */

/* 必须先调用：会发开机序列 + 清屏 + 切成 page mode */
void SeeedOLED_Init(void);

/* 清全屏（8页×16列×8像素） */
void SeeedOLED_ClearDisplay(void);

/* 设置亮度 0~255 */
void SeeedOLED_SetBrightness(uint8_t brightness);

/* 正常/反显 */
void SeeedOLED_SetNormalDisplay(void);
void SeeedOLED_SetInverseDisplay(void);

/* 设置 page 模式 / 横向模式 */
void SeeedOLED_SetPageMode(void);
void SeeedOLED_SetHorizontalMode(void);

/* 光标：第 row 页(0~7)，第 col 列(0~15)，1列=8像素宽 */
void SeeedOLED_SetTextXY(uint8_t row, uint8_t col);

/* 输出一个 ASCII 字符（32~127），其他显示空格 */
void SeeedOLED_PutChar(uint8_t c);

/* 输出 C 字符串，遇到 '\0' 结束 */
void SeeedOLED_PutString(const char *str);

/* 显示整数 */
uint8_t SeeedOLED_PutNumber(long n);

/* 显示浮点，带小数位数 */
uint8_t SeeedOLED_PutFloat(float f, uint8_t decimal);

/* 写一条命令（一般你不需要直接调） */
void SeeedOLED_SendCommand(uint8_t cmd);

/* 写一段数据（一般内部用） */
void SeeedOLED_SendData(uint8_t data);

#ifdef __cplusplus
}
#endif

#endif /* __SEEEDOLED_H__ */
