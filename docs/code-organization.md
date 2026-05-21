# Sotong Game STM32 Code Organization

This guide describes how to split the current single-file implementation into smaller STM32CubeIDE-friendly modules without disturbing CubeMX generated files.

## Target Layout

```text
Core/
├── Inc/
│   ├── app_config.h
│   ├── app_game.h
│   ├── app_sensors.h
│   ├── app_cli.h
│   ├── app_display.h
│   ├── app_io.h
│   └── main.h
└── Src/
    ├── main.c
    ├── app_game.c
    ├── app_sensors.c
    ├── app_cli.c
    ├── app_display.c
    ├── app_io.c
    └── stm32l4xx_it.c
```

## Module Boundaries

- `main.c`: HAL startup, peripheral initialization order, and the top-level `while (1)` loop.
- `app_config.h`: pin aliases, timing constants, thresholds, and board-level application settings.
- `app_game.c/.h`: Game 1 and Game 2 state machines, role handling, game-over flow, and button mode switching.
- `app_sensors.c/.h`: BSP sensor initialization, accelerometer/gyro/magnetometer norms, HTS221 cached calibration, and LPS22HB pressure filtering.
- `app_cli.c/.h`: UART RX interrupt queue, CLI line parsing, threshold commands, mode commands, and debug dump output.
- `app_display.c/.h`: OLED log buffer, short messages, environment screen, and escape progress display.
- `app_io.c/.h`: LED helpers, GPIO setup, UART setup, I2C setup, IR input setup, USART clock source selection, and cautious I2C recovery.

## Immediate Cleanup Notes

1. Keep `main.h` as a CubeMX-generated pin definition file. Put project-specific aliases in `app_config.h`.
2. Replace hard-coded LED, button, and IR pin macros with aliases based on CubeMX names.
3. Fix the IR comment in `main.c`: the IR sensor is on Arduino D2 / PD14, not PA2.
4. Fix the `dump` format string: integer values should use `%d`, not `%f`.
5. Avoid blocking UART transmit inside interrupt callbacks when possible. Queue RX and echo from the main loop if time allows.
6. Avoid recovering I2C immediately on every busy flag. Prefer a short timeout before reinitializing I2C1.

## Suggested Main Loop After Refactor

```c
int main(void)
{
    HAL_Init();

    UartClock_UseHSI_ForUSART1();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    MX_I2C1_Init();
    MX_IR_GPIO_Init();

    App_Display_Init();
    App_Sensors_Init();
    App_Game_Init();

    HAL_UART_Receive_IT(&huart1, &rx_it_byte, 1);

    while (1) {
        I2C1_TryRecover();
        Buzzer_Task();
        App_CLI_Poll();
        App_Game_Update();
    }
}
```
