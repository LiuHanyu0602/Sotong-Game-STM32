#ifndef APP_GAME_H
#define APP_GAME_H

#include <stdint.h>

typedef enum {
    MODE_RED_GREEN = 0,
    MODE_CATCH_RUN = 1
} GameMode;

typedef enum {
    PHASE_GREEN = 0,
    PHASE_RED = 1
} LightPhase;

void App_Game_Init(void);
void App_Game_Update(void);
void App_Game_HandleButtonExti(uint16_t gpio_pin);

GameMode App_Game_GetMode(void);
LightPhase App_Game_GetPhase(void);
uint8_t App_Game_IsOver(void);

void App_Game_SetMode(GameMode mode);
void App_Game_SetPlayerRole(uint8_t is_player);
void App_Game_SetMotionThresholds(float acc_g, float gyro_dps);

#endif /* APP_GAME_H */
