#ifndef APP_SENSORS_H
#define APP_SENSORS_H

#include <stdint.h>

void App_Sensors_Init(void);

float App_Sensors_AccelNormG(void);
float App_Sensors_GyroNormDps(void);
float App_Sensors_MagNormUT(void);

float App_Sensors_ReadTempC(void);
float App_Sensors_ReadHumidityPct(void);
float App_Sensors_ReadPressureHpa(void);

float HTS221_ReadTemperatureCached(uint16_t device_address);
float HTS221_ReadHumidityCached(uint16_t device_address);
float LPS22HB_ReadPressureCached(uint16_t device_address);

#endif /* APP_SENSORS_H */
