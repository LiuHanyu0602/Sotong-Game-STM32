#ifndef APP_DISPLAY_H
#define APP_DISPLAY_H

void App_Display_Init(void);
void App_Display_Log(const char *s);
void App_Display_PrintShort(const char *fmt, ...);
void App_Display_ShowEnv(float temp_c, float humidity_pct, float pressure_hpa);
void App_Display_ShowGameOver(void);
void App_Display_ShowEscapeProgress(unsigned int remain_ms, unsigned int total_ms);

#endif /* APP_DISPLAY_H */
