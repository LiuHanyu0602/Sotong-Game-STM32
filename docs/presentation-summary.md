# Presentation Summary

This document keeps the main points from the original project presentation in a shorter, repository-friendly form.

## Overview

The Sotong Game project implements two games on the B-L4S5I-IOT01A Discovery Kit:

- Red Light, Green Light
- Catch & Run

Both games support Player and Enforcer roles. The implementation combines onboard sensors, UART output, LED status, button interrupts, OLED display output, Grove 5-way switch input, an IR reflective sensor, and buzzer feedback.

## System Design

The application uses a non-blocking main loop. Time-based behavior is driven by `HAL_GetTick()` instead of long blocking delays, which keeps UART input, buzzer updates, game logic, and display updates responsive.

Key runtime components:

- UART CLI for real-time commands and debugging.
- EXTI button handling for single-click and double-click behavior.
- Sensor reads for environmental and movement detection.
- OLED display for compact game status and sensor output.
- LED and buzzer feedback for phase and warning signals.

## Game 1: Red Light, Green Light

During the green phase, players may move. The system reports environmental data such as temperature, humidity, and pressure.

During the red phase, movement is detected using accelerometer, gyroscope, and IR sensor readings. In Player mode, movement triggers game over. In Enforcer mode, the system reports that a player is out and continues monitoring.

## Game 2: Catch & Run

The system uses magnetometer readings to detect nearby players or enforcers. When proximity exceeds the configured threshold, a timed escape window starts.

The player can respond using the onboard button or the Grove 5-way switch. Grove direction input supports a direction-matching challenge, while the OLED shows escape timing and progress feedback.

## Optimizations

- HTS221 temperature and humidity calibration values are cached after initialization to reduce repeated I2C reads.
- LPS22HB pressure readings are smoothed with an exponential moving average filter.
- OLED updates are limited and buffered to reduce I2C display lag.
- UART1 clock selection is forced to HSI to avoid baud-rate instability.
- Float-style debug output is avoided where needed for STM32 printf compatibility.

## Issues Addressed

- UART output failure or garbled characters caused by clock and formatting issues.
- OLED display lag caused by frequent blocking I2C updates.
- Button bounce handled with a debounce interval and double-click timing window.
- Sensor read overhead reduced by caching calibration data.
