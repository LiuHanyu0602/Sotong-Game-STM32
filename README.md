

Good morning, everyone.
We are Group B03_11, and today we are going to present our work for Assignment 2.
Our project focuses on the implementation, enhancement, and debugging of the Sotong Game on the STM32 board.
We will walk you through how our system was built, what improvements we made, and the issues we faced during development.

=================================================================================================================

Here’s the outline of our presentation.
First, we’ll introduce the code implementation, including the system overview, overall structure, and detailed functions.
Next, we’ll move on to Enhancements, where we show the improvements we added on top of the basic requirements. For example, sensor optimization, UART interface, and OLED display.
Finally, we’ll talk about the Issues and Solutions,  some challenges we met during testing, and how we fixed them.

=================================================================================================================




This project is based on the B-L4S5I-IOT01A Discovery Board, which uses the STM32L4S5VI microcontroller.
The main peripherals include UART1 on PB6 and PB7, LED on PB14, and the User Button on PC13 connected by EXTI interrupt.
The board also has four onboard sensors: HTS221 for temperature and humidity, LPS22HB for pressure, LSM6DSL for acceleration and gyro, and LIS3MDL for magnetometer.
We designed two games：Red Light, Green Light and Catch & Run,  each with two modes: Player and Enforcer.


============================================================================================================


Here is the overall software structure.
The core functions include the main loop, the game functions, and sensor reading functions.
The main loop handles initialization, game-over cases, and logic switching.
The game functions contain the two game loops and handle data collection, processing, and evaluation.
The sensor functions redefine BSP sensor reading functions to improve efficiency.
On the right side, we have non-blocking events:  UART CLI, EXTI button, and HAL millisecond tick.
UART CLI provides a command-line interface for real-time control.
EXTI handles single and double clicks.
The SysTick timer provides precise timing without using HAL_Delay().

============================================================================================================


The initialization pipeline starts with HAL_Init().
This function initializes the HAL library and sets up the SysTick interrupt for 1 millisecond timing.
Then, UartClock_UseHSI_ForUSART1() changes the clock source of USART1 to the High-Speed Internal oscillator.
This helps to avoid UART timing drift when USB or system clock fluctuates.

============================================================================================================



Next, MX_GPIO_Init() initializes the LED and the User Button.
The LED on PB14 is configured as push-pull output, low frequency, and turned off at the beginning.
The button on PC13 is configured as an EXTI input — falling edge triggered, pull-up enabled.
Finally, NVIC is also initialized for EXTI interrupt handling.

============================================================================================================



MX_USART1_UART_Init() configures the serial port.
It enables the GPIOB and USART1 clocks, then sets up UART parameters：
baud rate 115200, 8 data bits, no parity, and 1 stop bit.
It also enables NVIC interrupt for USART1, so we can use interrupt-based RX instead of polling.

============================================================================================================



Sensors_Init() initializes all onboard BSP sensors:
（the temperature and humidity sensor HTS221, the pressure sensor LPS22HB,
the accelerometer and gyroscope LSM6DSL, and the magnetometer LIS3MDL.）
If any sensor fails to initialize, the system prints “ERR” through UART.
Finally, we initialize the game state and enter the Green Light phase of Game 1.

============================================================================================================

This flowchart shows the logic of the main loop in my program.
The main loop starts after all initial setup is completed.
 The first step is to initialize clocks, sensors, and transmissions.
 This means the MCU enables all the required peripherals,
 such as UART for serial communication, I²C for sensors, and GPIO for LEDs and buttons.
After initialization, the system prints the Grove 5-way switch status on the terminal.
 This helps confirm that the external input device is working correctly before starting the game.
Then, the program enters Game 1, beginning with the Green Light phase.
 This sets the initial game state and turns on the LED.
Next, two background tasks start in a non-blocking way.
 The first is the buzzer task, which can play sound asynchronously.
 The second is the UART command-line interface echoing,
 allowing the player to type commands without stopping the main loop.
After that, the program enables game mode switching by double-clicking the pushbutton.
 This means the player can quickly change between Game 1 and Game 2 at any time.

============================================================================================================


Then the system checks if Game Over has been triggered.
 If not, it keeps running the current game.
 The flow then branches:
If the current Game Mode = 1, the loop goes to Game 1’s logic;
Otherwise, it goes to Game 2’s logic.
If the game is over, the system enters a short animation:
The LED blinks quickly as visual feedback.
The OLED shows “Game Over” information.
The system then waits three seconds in a non-blocking way. This means the system still runs background tasks during this period,
 without freezing like HAL_Delay() would do.
Finally, the game automatically restarts in Game 1 Green Light phase, and the whole loop repeats.
This design makes the program responsive and continuous, even when displaying, waiting, or printing, the main loop never stops running, ensuring smooth gameplay.

============================================================================================================


This flowchart shows the full logic of Game 1 — Red Light and Green Light, in player mode.
The game starts with the Green Light phase.
 The system first prints “Green Light!” on the terminal to inform the player.
Then, the program checks whether it has already stayed in green light for 10 seconds.
 If not, it keeps the LED constantly ON,meaning the player is allowed to move during this phase.
Inside the 10-second window, the system continuously reads sensor data（ temperature, humidity, and pressure） from the onboard sensors.
 These data are printed out through UART for monitoring.

============================================================================================================







After 10 seconds, the program switches to the Red Light phase and prints “Red Light!” on the terminal.
During this phase, the LED blinks every 0.5 seconds.
 It serves as a visual warning that movement is not allowed.
Here, we've added a new setting for Game 1. The ground beneath the player's feet is painted in a checkerboard-like pattern with black and white squares. During the red light phase, players must remain stationary within the white areas to avoid being captured.
The MCU now continuously reads data from the accelerometer, gyroscope, and infrared sensors.
 The accelerometer and gyro detect motion or rotation, while the IR sensor detects if the player’s hand or body moves in front of it.
All these readings are printed through the UART for observation.
 The system then checks if any value exceeds a threshold, or if the IR sensor detects motion.
If either condition is true, the player is caught. The game prints “Player Out!” and triggers Game Over.
If no movement is detected, the loop continues, alternating between red and green light phases endlessly.
This design integrates multiple onboard sensors to achieve real-time motion detection and timing control, making the gameplay interactive and hardware-driven.

============================================================================================================


This flowchart illustrates the logic of the Enforcer mode in the Traffic Light Game.
Unlike the Player mode, since the game involves multiple players and a single Enforcer, the Enforcer's game does not end immediately after a player is out.
In other words, the Enforcer end continues monitoring until the red light phase concludes.

============================================================================================================


This flowchart shows the logic of Game 2: Catch and Run, in player mode.
The game starts by reading the onboard sensors.
 These sensors monitor the surrounding environment and player condition.
Then, the system checks if the readout exceeds a threshold.
 If the data is normal, it loops back and keeps monitoring.
If the values exceed the threshold, the program prints out the abnormal reading.
Next, the system reads magnetic field intensity from the magnetometer (LIS3MDL).
This sensor helps detect nearby “enforcers”.
The LED will blink according to the readout. When the magnetic field is strong, it blinks faster.

============================================================================================================


If the magnetic field exceeds a threshold,  the system prints “Enforcer nearby!” and the buzzer emits a warning sound.
Then, the game randomly generates an enforcer direction. This is the relative position from the players.
This process adds randomness and requires the player to react quickly.

============================================================================================================


At this point, the player must press the Grove 5-way button within 3 seconds, and importantly, the pressed direction must not match the enforcer’s direction.
If the player reacts correctly within time, The game prints “Player escaped – Good job!” and continues.
If the player presses the wrong direction, or too slowly. The system ends the round and enters Game Over.


============================================================================================================



The enfircer's mode is nearly identical to that of the players. 
The only difference is that if a player successfully escapes, the game prints “Keep trying.”
 If a player is captured, the game prints “Good job,” and goes on. The enforcer must continue capturing other players.

Overall, this game demonstrates sensor fusion and real-time decision making:
 combining pressure, temperature, magnetic, and button inputs
 to create a fast-reaction escape challenge.
============================================================================================================

In this part, I will explain how we handle the button input.
First, we need to remove noise from the button signal, which is called debouncing.
Each time the button is pressed, small vibrations cause multiple triggers within a few milliseconds.
So, we use the system tick timer and the function HAL_GPIO_EXTI_Callback() to ignore any other press happens inside a 20-ms window.
This way, each button press will only be counted once.

After debouncing, we evaluate single clicks.
A single click is recognized only when it happens outside the double-click time window of the previous click.
We check the timestamp of each press using system ticks.
If this timestamp is different from the last click, it is recorded as a new single click.
Also, in Game 2, clicks inside the “3-second escape window” are ignored because they belong to gameplay events.

============================================================================================================



To detect double clicks, we open a 600-millisecond time window after each single click.
If another valid click happens inside this window, it is recognized as a double click.
In the main program, when a double click is detected, the game mode will switch between Game 1 and Game 2.
At the same time, the timer is reset for the next detection.
============================================================================================================




To manage this process, we use a variable called first_press_state to record the current button state.
This variable can have three values.
When it’s 0, it means no clicks yet.
When it changes to 1, the first click has been detected, waiting for a possible second click.
If another click comes within 600 milliseconds, the value changes to 2, meaning a double click is confirmed.
After the game mode changes, the variable will be reset to 0.
============================================================================================================


In the original BSP sensor functions, every time the program reads data from a sensor, it also reads the calibration values again from the registers through the I²C bus.
However, these calibration values normally do not change after startup.
This means we waste extra I²C communication time in every reading cycle.
As shown in the flow chart on the right, it first reads raw data, then reads calibration values, and finally performs linear interpolation to get the final result.
============================================================================================================


In the new design, we read and store the calibration values only once at initialization.
After that, each cycle only reads the raw sensor data and uses the cached calibration values for calculation.
This reduces I²C traffic and makes sensor reading much faster.


============================================================================================================


For the HTS221 temperature sensor, we read two calibration points t₀ and t₁ only once at startup.
 Then we pre-calculate the slope and intercept of the linear relationship between sensor counts and temperature.
 After that, the function HTS221_ReadTemperatureCached() just computes:
T = slope × count + intercept


============================================================================================================



The same logic is used for humidity.
Reading H₀ and H₁ once, and pre-calculating the slope and intercept.
The function HTS221_ReadHumidityCached() then calculates humidity as:
“RH = slope × count + intercept”
This makes humidity measurement much faster.
============================================================================================================

For the LPS22HB barometer, we add a simple EMA filter.
It combines the previous output and the current input to smooth the pressure readings.
The formula is:
EMAₜ = α × xₜ + (1 − α) × EMAₜ₋₁.
Here, α is 0.2 by default.
This filter helps remove noise and prevents sudden spikes in pressure data.
============================================================================================================



Next, we implemented a UART command line interface, or CLI.
This allows the user to control or configure the game directly from the serial terminal.
The program reads characters one by one and stores them in a buffer.
Each character is also echoed back to the terminal.
When the user presses Enter, the program checks the whole line and executes the corresponding command.
============================================================================================================





To make this process non-blocking, we use interrupt-based receiving.
At startup, UART1 is initialized, and we call HAL_UART_Receive_IT() to enable single-byte interrupt receiving.
Every time one byte arrives, an interrupt is triggered, and the byte is stored into the buffer.


============================================================================================================



Whenever a new byte is received, it goes through a standard interrupt service routine, or ISR, provided by the HAL library.
 The process involves three key functions:
USART1_IRQHandler() – this is the main interrupt handler registered in the vector table.
HAL_UART_IRQHandler(&huart1) – this function checks whether the interrupt is from TX or RX.
HAL_UART_RxCpltCallback() – this callback is called when a byte has been received successfully.
 In this callback, we can decide what to do with the received data.



============================================================================================================



Inside HAL_UART_RxCpltCallback(), the logic flow is as follows:
Read the received byte stored in rx_it_byte.


Push it into a ring buffer using rxq_push(c).


Re-arm the next interrupt by calling HAL_UART_Receive_IT() again.


Echo the received character back to the terminal.

This structure ensures that the program can keep receiving new characters continuously without losing any data.



============================================================================================================





In the main loop, another function cli_poll() runs in every iteration.
It pulls characters from the ring buffer one by one, ignores \n, and treats \r as the end of a line.
If the line reaches 63 characters, it is also treated as full.
Once a complete line is ready, the program calls cli_handle_line() to parse and execute the command.


============================================================================================================



Finally, these are the commands supported by our CLI system.
g1 or g2: change the game mode to Game 1 or Game 2.


role p: switch to the player role.


role e: switch to the enforcer role.


thr a <g>: change the acceleration threshold to a new g value.


thr w <dps>: change the gyro threshold to a new dps value.
We also have two helper commands.
 The first one is dump.
 It prints out the current game status,  including the game mode, the current role, all the threshold values, and the latest sensor readings like acceleration, gyro, and magnetometer.
 This command is mainly used for debugging, so we can quickly check what the system is doing now.
The second one is [unknown command].
 If the user types something that is not recognized, the system will automatically print a help message, showing all the valid commands that can be used.
 This makes the CLI more user-friendly and prevents confusion during testing.

 These commands help us debug the game and test different parameters without modifying the code.




============================================================================================================

In this part, we introduce the OLED display used in our project.
 We use a Grove OLED screen based on the SSD1308 controller.
 It is connected to the I²C1 bus, using PB8 for SCL and PB9 for SDA.
 This screen can show four lines, each with sixteen characters.
 We use it to display game status and sensor data in real time.
For initialization, we first enable and initialize I²C1.
 Then we call the built-in OLED initialization function, oled_log_init().
 After this, the screen is ready for text display.

============================================================================================================


We define three display modes to handle different use cases.
The first mode is oled_log_put.
 It refreshes only the bottom line of the screen, and is used to show short messages like “Player Out” or “Game Over”.
The second mode is oled_show_env.
 This mode displays all four lines of sensor data, for example, temperature, humidity, pressure, and acceleration.
The last one is a special mode for Game 2.
 It shows a progress bar on the screen, which indicates how much time is left for the player to escape.

============================================================================================================

Now I will explain how the OLED display is used in our two games.
In Game 1,
During the Green Light phase, the OLED shows the game phase, temperature, humidity, and barometer readings.
During the Red Light phase, it displays acceleration and gyro data, and also shows game-over information when a movement is detected.


In Game 2,
In the Normal state, the screen displays environmental data when any abnormal condition is detected.
During the Escape window, it shows a progress bar and the remaining time percentage for the player’s escape.
 It also displays game-over messages when the player fails to escape.

============================================================================================================




Here I describe the first issue we met: the UART serial port had no output at all.
 When the function SystemClock_Config() is called, there was nothing shown in the serial terminal.
This happens because the USART1 clock source can come from different options, such as PCLK2, SYSCLK, HSI16, or LSE.
 However, the default SystemClock_Config() function does not specify which one to use.
 As a result, UART may run with the wrong clock, and the baud rate becomes incorrect, causing no visible output.
============================================================================================================


To fix this issue, we manually select a stable clock source for UART.
 We define a helper function named UartClock_UseHSI_ForUSART1().
In this function, we first enable the High Speed Internal clock (HSI),
 wait until it becomes stable,
 then configure USART1 to use this HSI clock as its clock source.
This guarantees that UART1 always runs with a correct and stable frequency,
 so the serial output becomes normal again.
============================================================================================================

The second issue we encountered is garbled characters and incorrect number formats in the serial output.
 Sometimes the numbers are not displayed correctly, and sometimes random or nonsense characters appear on the screen.
There are two main reasons behind this.
The first one is that floating-point formatting is not supported by default in the STM32 sprintf() function.
 If we try to print a float number using %f, the IDE simply rejects it, so the output becomes strange symbols.
The second possible reason is a slightly mismatched baud rate between the STM32 and the serial terminal.
 Even a small difference in clock accuracy can cause garbled characters to appear occasionally.
============================================================================================================


To solve the second issue, we made two improvements.
First, the clock fix in Issue 1 already helps remove any timing misalignment that may cause wrong characters.
Second, for data formatting, we stop using float printing.
 Normally, we would print like this:
 sprintf("T=%.1f\r\n", t);
 But STM32 does not support float format by default.
So instead, we convert the float into two integer parts:  the integer part and the first decimal digit.
 For example:
 sprintf("T=%d.%01d\r\n", (int)t, abs((int)(t * 10.0f) % 10));
This way, we can correctly print one decimal place without using float formatting.
 The output becomes stable and readable on the serial monitor.
============================================================================================================




The next issue we found is about the OLED display.
 During early testing, we noticed that the OLED caused noticeable lag in the main game loop.
For example, the LED blinking became slower, and sensor readings were delayed, especially when text was frequently printed to the OLED screen.
The main reason is that the OLED is much slower than the MCU.
 Every time the MCU sends data to the OLED, the communication takes some time through the I²C bus.
 During this time, the main loop is blocked and cannot continue running.
 This results in delay and visible lag on the system.

============================================================================================================




To solve this problem, we optimized how the OLED is updated.
Instead of printing every character directly to the screen,
 we use a RAM buffer to store the text first.
 The buffer can hold four lines with sixteen characters per line.
Then, the screen is updated only when needed.
 For example, when the text actually changes.

 We also update only the necessary part of the OLED.
 So the rest of the screen remains untouched.

This way, we reduce I²C transmission time and prevent the main loop from being blocked.
 The game now runs smoothly again.
============================================================================================================

