# Core Source Notes

Place handwritten application modules here. Keep STM32CubeMX generated code in its normal generated sections, and move reusable game logic into dedicated `app_*` files.

Recommended source modules:

- `app_io.c`: LED, GPIO, UART, I2C, IR, and clock helper functions.
- `app_display.c`: OLED rendering and message helpers.
- `app_cli.c`: UART RX queue and command parser.
- `app_sensors.c`: sensor reads, calibration caches, and filtering.
- `app_game.c`: Red Light Green Light and Catch & Run state machines.
