#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "main.h"

/* Board pin aliases used by the application layer. Keep CubeMX pin names in main.h. */
#define APP_LED_Pin             LED2_Pin
#define APP_LED_GPIO_Port       LED2_GPIO_Port

#define APP_BUTTON_Pin          BUTTON_EXTI13_Pin
#define APP_BUTTON_GPIO_Port    BUTTON_EXTI13_GPIO_Port
#define APP_BUTTON_EXTI_IRQn    BUTTON_EXTI13_EXTI_IRQn

/* IR Reflective Sensor uses Arduino D2 / PD14 on the B-L4S5I-IOT01A board. */
#define APP_IR_Pin              ARD_D2_Pin
#define APP_IR_GPIO_Port        ARD_D2_GPIO_Port

/* Game 1 timing. */
#define GREEN_PHASE_MS          10000U
#define RED_PHASE_MS            10000U
#define GREEN_ENV_TX_PERIOD     2000U
#define RED_MOTION_TX_PERIOD    2000U
#define RED_LED_TOGGLE_MS       500U

/* Game 1 motion thresholds. */
#define ACC_MOV_THRESH_G        0.25f
#define GYR_MOV_THRESH_DPS      50.0f

/* Game 2 thresholds. */
#define MAG_NEAR_THRESH_uT      250.0f
#define BUTTON_ESCAPE_WINDOW_MS 3000U
#define ENV_TEMP_SPIKE_C        32.0f
#define ENV_HUMI_SPIKE_PCT      75.0f
#define ENV_PRES_SPIKE_HPA      1015.0f

/* Game 2 blink bands. */
#define MAG_BAND1_uT            100.0f
#define MAG_BAND2_uT            160.0f
#define MAG_BAND3_uT            220.0f
#define BLINK_SLOW_MS           400U
#define BLINK_MED_MS            200U
#define BLINK_FAST_MS           80U

/* Game over and button handling. */
#define GAMEOVER_BLINK_MS       120U
#define GAMEOVER_HOLD_MS        3000U
#define BTN_DEBOUNCE_MS         20U
#define BTN_DOUBLE_WINDOW_MS    600U

#endif /* APP_CONFIG_H */
