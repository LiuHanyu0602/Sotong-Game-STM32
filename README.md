# Sotong Game on STM32

This repository contains an EE2028 Assignment 2 implementation of the Sotong Game on the ST B-L4S5I-IOT01A Discovery Kit, using the STM32L4S5VI microcontroller.

The project implements two interactive game modes using onboard sensors, UART output, LEDs, a push button, a Grove 5-way switch, an OLED display, an IR reflective sensor, and a buzzer.

## Game Modes

### Game 1: Red Light, Green Light

- Green phase: players may move while environmental data is displayed and printed.
- Red phase: movement is detected using accelerometer, gyroscope, and IR sensor readings.
- Player role: detected movement causes game over.
- Enforcer role: detected movement reports a player out while monitoring continues.

### Game 2: Catch & Run

- Uses magnetometer strength to detect a nearby enforcer/player.
- Starts a timed escape window when proximity is detected.
- Supports push-button escape and Grove 5-way switch direction gameplay.
- OLED and LED feedback show warning and escape status.

## Hardware

- Board: ST B-L4S5I-IOT01A Discovery Kit
- MCU: STM32L4S5VI
- UART: USART1 on PB6/PB7
- LED: PB14
- User button: PC13 EXTI
- I2C1: PB8/PB9
- Sensors:
  - HTS221 temperature/humidity sensor
  - LPS22HB pressure sensor
  - LSM6DSL accelerometer/gyroscope
  - LIS3MDL magnetometer
- External devices:
  - Seeed OLED display on I2C1
  - Grove 5-way switch on I2C1
  - IR reflective sensor on PD14
  - Buzzer

## Repository Layout

```text
.
├── Inc/                 # Application and STM32 headers
├── Src/                 # Application source files
├── B-L4S5I-IOT01/       # Board support package files
├── Components/          # Sensor and component drivers
├── Device/              # CMSIS device headers
├── Startup/             # STM32 startup assembly
├── docs/                # Project notes
└── README.md
```

## Key Implementation Features

- Non-blocking main loop using `HAL_GetTick()` timing.
- UART command-line interface for mode, role, threshold, and debug commands.
- Cached HTS221 calibration values to reduce repeated I2C transactions.
- LPS22HB pressure smoothing with an EMA filter.
- OLED display buffering to reduce display update lag.
- Single I2C1 bus shared by onboard sensors, OLED, and Grove switch.

## UART Commands

```text
g1              Switch to Game 1
g2              Switch to Game 2
role p          Set role to Player
role e          Set role to Enforcer
thr a <g>       Set acceleration threshold
thr w <dps>     Set gyroscope threshold
dump            Print current debug state
```

## Build Notes

This project is intended for STM32CubeIDE. Open the project folder in STM32CubeIDE, build the firmware, and flash it to the B-L4S5I-IOT01A board.

Build output folders such as `Debug/` and `Release/` are intentionally ignored by Git.
