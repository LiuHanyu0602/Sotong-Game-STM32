/**
* EE2028 "Sotong Game" - Referee Unit (B-L4S5I-IOT01A, STM32L4S5VI)
* Game 1: Red Light, Green Light
* Game 2: Catch & Run
*
* Peripherals:
*  - UART: text status (USART1, PB6/PB7)
*  - LED (PB14): phase/outcome indication
*  - Push Button (PC13 EXTI): single press / double press
*  - Sensors (on-board, all I2C1 PB8/PB9):
*      * HTS221 (Temperature/Humidity)  -> BSP_TSENSOR / BSP_HSENSOR
*      * LPS22HB (Pressure)             -> BSP_PSENSOR
*      * LSM6DSL (Accel/Gyro)           -> BSP_ACCELERO / BSP_GYRO
*      * LIS3MDL (Magnetometer)         -> BSP_MAGNETO
*  - External I2C1 devices (merged):
*      * Seeed OLED (I2C1 PB8/PB9)
*      * Grove 5-Way Switch (I2C1 PB8/PB9)   <-- 原来在 I2C2, 现在合到 I2C1
*  - External GPIO:
*      * IR Reflective Sensor (PD14 input, black=0)
*  - Timer:
*      * TIM2 used by TimerOne_STM32 (for your previous code)
*
* NOTE:
*  - Single I2C bus: PB8=SCL1, PB9=SDA1, no pin conflicts.
*  - All game features from both old main.c are kept.
*/
#include "main.h"
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>

/* ==== extra user libs ==== */
#include "seeedoled.h"          /* OLED on I2C1 */
#include "buzzer.h"             /* Buzzer non-blocking task */
#include "Grove_Switch_STM32.h" /* Grove 5-Way Switch (moved to I2C1) */
#include "TimerOne_STM32.h"     /* your timer wrapper */

/* ==== HAL / BSP includes ==== */
#include "stm32l4xx_hal.h"
#include "../../Drivers/BSP/Components/hts221/hts221.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_magneto.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_hsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_psensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"

/* ==== UART handle  ==== */
UART_HandleTypeDef huart1;
/* ==== I2C1 handle (PB8=SCL, PB9=SDA) ==== */
I2C_HandleTypeDef hi2c1;
/* ==== TIM handle (for TimerOne_STM32) ==== */
TIM_HandleTypeDef htim2;
TimerOne_STM32_t  Timer1;

/* ==== Grove 5-Way Switch handle ==== */
GroveSwitch_Handle_t hSwitch;
static uint8_t g_grove_ok = 0;

/* === HTS221 register  === */
#ifndef HTS221_T1_T0_MSB
#define HTS221_T1_T0_MSB   0x35U
#endif

/* ===== LPS22HB: device addr & regs  ===== */
#ifndef LPS22HB_DEV_ADDR
#define LPS22HB_DEV_ADDR   LPS22HB_I2C_ADDRESS
#endif
#ifndef LPS22HB_WHO_AM_I_REG
#define LPS22HB_WHO_AM_I_REG        0x0F
#endif
#ifndef LPS22HB_WHO_AM_I_VAL
#define LPS22HB_WHO_AM_I_VAL        0xB1
#endif
#ifndef LPS22HB_CTRL_REG1
#define LPS22HB_CTRL_REG1           0x10
#endif
#ifndef LPS22HB_CTRL_REG2
#define LPS22HB_CTRL_REG2           0x11
#endif
#ifndef LPS22HB_PRESS_OUT_XL_REG
#define LPS22HB_PRESS_OUT_XL_REG    0x28
#endif
#ifndef LPS22HB_CTRL1_BDU
#define LPS22HB_CTRL1_BDU           0x02
#endif
#ifndef LPS22HB_CTRL1_ODR_10HZ
#define LPS22HB_CTRL1_ODR_10HZ      0x20
#endif
#ifndef LPS22HB_CTRL2_IF_ADD_INC
#define LPS22HB_CTRL2_IF_ADD_INC    0x10
#endif
#ifndef LPS22HB_CTRL2_SWRESET
#define LPS22HB_CTRL2_SWRESET       0x04
#endif
#ifndef LPS22HB_CTRL2_ONE_SHOT
#define LPS22HB_CTRL2_ONE_SHOT      0x01
#endif

/* 统一设备地址宏 */
#ifndef HTS221_DEV_ADDR
#define HTS221_DEV_ADDR  HTS221_I2C_ADDRESS
#endif

/* ==== OLED log buffer  ==== */
#define OLED_LOG_LINES 4
#define OLED_LINE_LEN  16

/* ==== GPIO: LED (PB14), Button (PC13) ==== */
#define LED2_GPIO_PORT      GPIOB
#define LED2_PIN            GPIO_PIN_14
#define USER_BTN_GPIO_PORT  GPIOC
#define USER_BTN_PIN        GPIO_PIN_13
#define USER_BTN_EXTI_IRQn  EXTI15_10_IRQn

/* ==== IR Reflective Sensor (black=0) ==== */
#define IR_GPIO_PORT      GPIOD
#define IR_PIN            GPIO_PIN_14

/* ====  Role (1=Player, 0=Enforcer) ==== */
#define ROLE_IS_PLAYER      1

/* ==== Game timing thresholds ==== */
#define GREEN_PHASE_MS          10000U
#define RED_PHASE_MS            10000U
#define GREEN_ENV_TX_PERIOD     2000U
#define RED_MOTION_TX_PERIOD    2000U
#define RED_LED_TOGGLE_MS       500U

/* Game1 motion thresholds */
#define ACC_MOV_THRESH_G        0.25f
#define GYR_MOV_THRESH_DPS      50.0f

/* Game2 Magnetic proximity and environment thresholds */
#define MAG_NEAR_THRESH_uT      250.0f
#define BUTTON_ESCAPE_WINDOW_MS 3000U
#define ENV_TEMP_SPIKE_C        32.0f
#define ENV_HUMI_SPIKE_PCT      75.0f
#define ENV_PRES_SPIKE_HPA      1015.0f

/* Game2 blink bands */
#define MAG_BAND1_uT            100.0f
#define MAG_BAND2_uT            160.0f
#define MAG_BAND3_uT            220.0f
#define BLINK_SLOW_MS           400U
#define BLINK_MED_MS            200U
#define BLINK_FAST_MS           80U

/* GameOver */
#define GAMEOVER_BLINK_MS       120U
#define GAMEOVER_HOLD_MS        3000U   /* 用你第一个 main 的 3s，便于OLED提示 */

/* 按键防抖与双击判定 */
#define BTN_DEBOUNCE_MS         20U
#define BTN_DOUBLE_WINDOW_MS    600U

/* 毫秒时基 */
static inline uint32_t ms_now(void) { return HAL_GetTick(); }
static inline int is_due(uint32_t now, uint32_t deadline) { return (int32_t)(now - deadline) >= 0; }

/* 游戏状态 */
typedef enum { MODE_RED_GREEN = 0, MODE_CATCH_RUN = 1 } GameMode;
typedef enum { PHASE_GREEN = 0, PHASE_RED = 1 } LightPhase;

static volatile GameMode g_mode = MODE_RED_GREEN;
static volatile LightPhase g_phase = PHASE_GREEN;
/* 时间戳 */
static uint32_t phase_t0 = 0;
static uint32_t last_env_transmit = 0;
static uint32_t last_motion_transmit = 0;
static uint32_t last_led_toggle = 0;
static uint32_t game_over_since = 0;

/* 按键状态机 */
static volatile uint8_t  first_press_state = 0;
static volatile uint32_t first_press_ts    = 0;
static volatile uint32_t btn_last_isr_ts   = 0;

/* 运行期角色标志 */
static int role_is_player = 1;

/* Game2 逃脱窗口状态 */
static uint8_t  wait_escape     = 0;
static uint32_t escape_deadline = 0;
/* Game2 方向对抗（来自第二个 main） */
static uint8_t  g_use_dir_game  = 0;   /* 触发时置1 */
static uint8_t  g_enforcer_dir  = 0;   /* 0=UP,1=DOWN,2=LEFT,3=RIGHT */
static volatile uint8_t game_over = 0;

/* CLI 阈值 */
volatile float g_acc_thr_g      = ACC_MOV_THRESH_G;
volatile float g_gyr_thr_dps    = GYR_MOV_THRESH_DPS;

/* UART RX 环形缓冲区 */
#define RXQ_SIZE 128
static volatile uint8_t  rxq[RXQ_SIZE];
static volatile uint16_t rxq_head = 0, rxq_tail = 0;
static uint8_t rx_it_byte = 0;

/* CLI 行缓冲 */
static char cli_line[64];
static uint8_t cli_len = 0;

/* OLED log */
static char oled_lines[OLED_LOG_LINES][OLED_LINE_LEN + 1];

/* ==== forward decl ==== */
static void I2C1_TryRecover(void);

static void UartClock_UseHSI_ForUSART1(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_IR_GPIO_Init(void);
static void Sensors_Init(void);
static void cli_poll(void);
static void cli_handle_line(const char *s);
static void uart_send_cstr(const char *s);
static void uart_printf(const char *fmt, ...);
static void uart_printf_nooled(const char *fmt, ...);
static void oled_log_init(void);
static void oled_log_puts(const char *s);
static void oled_log_render(void);
static void oled_printf_short(const char *fmt, ...);
static void oled_show_env(float t, float h, float p);
static void LED_On(void);
static void LED_Off(void);
static void LED_Toggle(void);
static void LED_Blink_step(uint32_t period_ms);
static void loop_red_green(void);
static void loop_catch_run(void);
static float accel_norm_g(void);
static float gyro_norm_dps(void);
static float mag_norm_uT(void);
static float read_temp_C(void);
static float read_humi_pct(void);
static float read_pres_hPa(void);
static void maybe_switch_mode_on_double_click(void);
static uint8_t button_single_press_happened_since(uint32_t since_ms);
static void game_end(const char* reason);
void USART1_IRQHandler(void);

float HTS221_ReadTemperatureCached(uint16_t device_address);
static float HTS221_ReadHumidityCached(uint16_t dev);
static float LPS22HB_ReadPressureCached(uint16_t dev);
static void  hts221_calibration_init_once(uint16_t dev);

/* ====== LPS22HB + HTS221 cached read (keep) ====== */
/* ... 原来两份 main 都有的那大坨缓存读取我也保留 ... */
/* 为了不撑爆这里，我直接贴保留版 —— 和你上面的一样，只是搬到后面 */
/* ---- 先把 main 做出来 ---- */

/* ================== main ================== */
int main(void)
{
    HAL_Init();
    UartClock_UseHSI_ForUSART1();
    MX_GPIO_Init();
    MX_USART1_UART_Init();
    HAL_UART_Receive_IT(&huart1, &rx_it_byte, 1);

    MX_I2C1_Init();          /* 统一用 I2C1: PB8/PB9 */
    SeeedOLED_Init();        /* OLED 走 I2C1 */
    oled_log_init();

    Buzzer_Init();           /* 蜂鸣器任务 */

    MX_IR_GPIO_Init();       /* IR PA2 */

    /* 板载传感器 (也在 I2C1 上) */
    Sensors_Init();

    /* TimerOne (你原来的) */
    TimerOne_Init(&Timer1, &htim2, 1000000);  /* 1s tick，就保持你的写法 */

    /* Grove 5-Way Switch 现在也用 I2C1 */
    if (GroveSwitch_Init(&hSwitch, &hi2c1) == HAL_OK) {
        g_grove_ok = 1;
        uart_send_cstr("Grove 5-Way Switch detected on I2C1\r\n");
    } else {
        g_grove_ok = 0;
        uart_send_cstr("Grove 5-Way Switch NOT found on I2C1\r\n");
    }

    /* 默认进入 Game1 */
    {
        char msg[80];
        if (role_is_player)
            sprintf(msg, "Entering Red Light, Green Light as Player\r\n");
        else
            sprintf(msg, "Entering Red Light, Green Light as Enforcer\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 0xFFFF);
    }

    g_mode = MODE_RED_GREEN;
    g_phase = PHASE_GREEN;
    phase_t0 = ms_now();
    last_env_transmit = phase_t0;
    LED_On();

    while (1)
    {
        I2C1_TryRecover();   // 如果总线卡BUSY就重配一次

        /* 1) 非阻塞蜂鸣器 */
        Buzzer_Task();



        /* 这里就只做“打印一下按钮”这种调试，不参与博弈 */
        // && !(g_mode == MODE_CATCH_RUN && wait_escape)
        if (g_grove_ok && !(g_mode == MODE_CATCH_RUN && wait_escape) ) {
            uint8_t btn_idx = 0xFF;
            SwitchEvent_t sw_ev = GroveSwitch_GetEvent(&hSwitch, &btn_idx);
            if (sw_ev == SW_EVENT_SINGLE_CLICK) {
                switch (btn_idx) {
                case SW_BTN_UP:    uart_send_cstr("Grove: Up pressed\r\n"); break;
                case SW_BTN_DOWN:  uart_send_cstr("Grove: Down pressed\r\n"); break;
                case SW_BTN_LEFT:  uart_send_cstr("Grove: Left pressed\r\n"); break;
                case SW_BTN_RIGHT: uart_send_cstr("Grove: Right pressed\r\n"); break;
                case SW_BTN_CENTER:uart_send_cstr("Grove: Center pressed\r\n"); break;
                default: break;
                }
            } else if (sw_ev == SW_EVENT_DOUBLE_CLICK) {
                uart_send_cstr("Grove: Double click\r\n");
            } else if (sw_ev == SW_EVENT_LONG_PRESS) {
                uart_send_cstr("Grove: Long press\r\n");
            }
        }

        /* 3) CLI */
        cli_poll();

        /* 4) 如果已经 Game Over -> 走 Game Over 分支 */
        if (game_over) {
            /* 快闪 */
            LED_Blink_step(GAMEOVER_BLINK_MS);

            /* 允许双击切模式 */
            maybe_switch_mode_on_double_click();
            cli_poll();

            /* OLED 提醒 */
            SeeedOLED_SetTextXY(1,0);
            SeeedOLED_PutString("   GAME OVER    ");
            SeeedOLED_SetTextXY(2,0);
            SeeedOLED_PutString("Restart in 3s...");

            if (is_due(ms_now(), game_over_since + GAMEOVER_HOLD_MS)) {
                /* 自动重启到 Game1 */
                game_over = 0;
                first_press_state = 0;
                wait_escape = 0;
                g_use_dir_game = 0;

                g_mode  = MODE_RED_GREEN;
                g_phase = PHASE_GREEN;

                uint32_t now = ms_now();
                phase_t0 = now;
                last_env_transmit   = now;
                last_motion_transmit= now;
                last_led_toggle     = now;

                LED_On();
                SeeedOLED_ClearDisplay();
                SeeedOLED_SetTextXY(1,0);
                SeeedOLED_PutString("Restart Game 1");
                uart_send_cstr("Restarting: Game 1 (Green Light)\r\n");
            }
            continue;
        }

        /* 5) 双击切模式 */
        maybe_switch_mode_on_double_click();

        /* 6) 正常游戏循环 */
        if (g_mode == MODE_RED_GREEN) {
            loop_red_green();
        } else {
            loop_catch_run();
        }
    }
}

/* ================= Game 1 ================= */
static void loop_red_green(void)
{
    uint32_t now = ms_now();
    uint32_t phase_len = (g_phase == PHASE_GREEN) ? GREEN_PHASE_MS : RED_PHASE_MS;

    /* 到时间切相位 */
    if (is_due(now, phase_t0 + phase_len)) {
        g_phase = (g_phase == PHASE_GREEN) ? PHASE_RED : PHASE_GREEN;
        phase_t0 = now;
        if (g_phase == PHASE_GREEN) {
            uart_printf("Green Light!\r\n");
            LED_On();
            last_env_transmit = now;
            oled_show_env(read_temp_C(), read_humi_pct(), read_pres_hPa());
        } else {
            uart_printf("Red Light!\r\n");
            last_led_toggle     = now;
            last_motion_transmit= now;
            LED_Toggle();
        }
    }

    if (g_phase == PHASE_GREEN) {
        /* 每2秒上报环境 + OLED */
        if (is_due(now, last_env_transmit + GREEN_ENV_TX_PERIOD)) {
            last_env_transmit = now;
            float t = read_temp_C();
            float h = read_humi_pct();
            float p = read_pres_hPa();

            int t10 = (int)(t * 10.0f + (t>=0?0.5f:-0.5f));
            int h10 = (int)(h * 10.0f + (h>=0?0.5f:-0.5f));
            int p10 = (int)(p * 10.0f + (p>=0?0.5f:-0.5f));
            uart_printf_nooled("[ENV] T=%d.%01dC, H=%d.%01d%%, P=%d.%01dhPa\r\n",
                               t10/10, abs(t10%10),
                               h10/10, abs(h10%10),
                               p10/10, abs(p10%10));
            oled_show_env(t, h, p);
        }
    } else {
        /* 红灯：闪烁 + motion 采样 + IR 检测 */
        if (is_due(now, last_led_toggle + RED_LED_TOGGLE_MS)) {
            last_led_toggle = now;
            LED_Toggle();
        }

        /* 2) IR 检测（高频） —— 每一圈都看  */
//        if (HAL_GPIO_ReadPin(IR_GPIO_PORT, IR_PIN) == GPIO_PIN_SET) {
//            uart_send_cstr("[IR] Movement detected!\r\n");
//            if (role_is_player) {
//                game_end("Game Over: IR detected movement!\r\n");
//                return;
//            }
//        }

        if (is_due(now, last_motion_transmit + RED_MOTION_TX_PERIOD)) {
            last_motion_transmit = now;

            float an = accel_norm_g();
            float gn = gyro_norm_dps();
            int a100 = (int)(an * 100.0f + (an>=0?0.5f:-0.5f));
            int g10  = (int)(gn * 10.0f  + (gn>=0?0.5f:-0.5f));

            uart_printf_nooled("[MOTION] |a|=%d.%02dg, |w|=%d.%01ddps\r\n",
                               a100/100, abs(a100%100),
                               g10/10,  abs(g10%10));

            /* OLED 显示简短 */
            int a100_int = (int)(an * 100.0f + (an >= 0 ? 0.5f : -0.5f));
            oled_printf_short("MOVE a=%d.%02dg", a100_int/100, abs(a100_int%100));

            /* IR 检测 */
            if (HAL_GPIO_ReadPin(IR_GPIO_PORT, IR_PIN) == GPIO_PIN_SET) {
                uart_send_cstr("[IR] Movement detected!\r\n");
                if (role_is_player) {
                    game_end("Game Over: IR detected movement!\r\n");
                    return;
                }
            }

            /* 原来的加速度/角速度判动 */
            if (an > g_acc_thr_g || gn > g_gyr_thr_dps) {
                if (role_is_player) {
                    game_end("Game Over\r\n");
                    return;
                } else {
                    uart_send_cstr("Player Out!\r\n");
                }
            }
        }
    }
}

/* ================= Game 2 ================= */
static void loop_catch_run(void)
{
    uint32_t now = ms_now();

    /* 如果已经在 3 秒窗口里，就做窗口逻辑（非阻塞版） */
    if (wait_escape) {
        uint32_t remain_ms = (escape_deadline > now) ? (escape_deadline - now) : 0;

        /* ===== 新增：限频刷新 OLED，每100ms一次 ===== */
        static uint32_t last_oled_escape = 0;
        if (is_due(now, last_oled_escape + 100U)) {
            last_oled_escape = now;

            float ratio = (float)remain_ms / (float)BUTTON_ESCAPE_WINDOW_MS;
            if (ratio < 0.0f) ratio = 0.0f;
            if (ratio > 1.0f) ratio = 1.0f;

            SeeedOLED_SetTextXY(2, 0);
            char line[24];
            int remain10 = (int)(remain_ms / 1000.0f * 10.0f + 0.5f);
            snprintf(line, sizeof(line), "Escape: %d.%01ds", remain10/10, abs(remain10%10));
            SeeedOLED_PutString(line);

            int percent = (int)(ratio * 100.0f + 0.5f);
            int bar_len = (int)(ratio * 10.0f + 0.5f);
            char bar[12];
            for (int i = 0; i < 10; i++)
                bar[i] = (i < bar_len) ? 0xA5 : '-';
            bar[10] = '\0';

            SeeedOLED_SetTextXY(3, 0);
            snprintf(line, sizeof(line), "[%-10s]%3d%%", bar, percent);
            SeeedOLED_PutString(line);
        }
        /* ===== 限频结束 ===== */

        /* LED 闪烁提示 */
        LED_Blink_step(BLINK_MED_MS);

        /* 蜂鸣器任务 */
        Buzzer_Task();

        /* CLI / 按钮 */
        cli_poll();
        maybe_switch_mode_on_double_click();

        /* 1) 来自第一个 main 的：PC13 单击逃脱 */
        if (button_single_press_happened_since(escape_deadline - BUTTON_ESCAPE_WINDOW_MS)) {
            if (role_is_player)
                uart_send_cstr("Player escaped, good job!\r\n");
            else
                uart_send_cstr("Player captured, good job!\r\n");
            wait_escape = 0;
            g_use_dir_game = 0;
            return;
        }

        /* 2) 来自第二个 main 的：Grove 方向对抗 */
        /* 2) Grove 按键处理：CENTER 永远是立即逃脱，其他是方向博弈 */
        if (g_grove_ok) {
            uint8_t btn_idx = 0xFF;
            SwitchEvent_t ev = GroveSwitch_GetEvent(&hSwitch, &btn_idx);
            if (ev != SW_EVENT_NONE) {

                /* 2.1 CENTER：无条件逃脱 */
                if (btn_idx == SW_BTN_CENTER) {
                    uart_send_cstr("Player escaped by Grove CENTER!\r\n");
                    wait_escape   = 0;
                    g_use_dir_game= 0;
                    return;
                }

                /* 2.2 这次窗口本来就是要方向博弈的（有 Grove） */
                if (g_use_dir_game) {
                    int8_t player_dir = -1;
                    switch (btn_idx) {
                    case SW_BTN_UP:    player_dir = 0; break;
                    case SW_BTN_DOWN:  player_dir = 1; break;
                    case SW_BTN_LEFT:  player_dir = 2; break;
                    case SW_BTN_RIGHT: player_dir = 3; break;
                    default: break;
                    }

                    if (player_dir >= 0) {
                        const char *dir_name[4] = {"UP","DOWN","LEFT","RIGHT"};
                        uart_printf("[G2] Player chose %s\r\n", dir_name[player_dir]);

                        if (player_dir == g_enforcer_dir) {
                            /* 猜中 → 被抓 */
                            if (role_is_player) {
                                game_end("Caught by enforcer (direction matched)!\r\n");
                            } else {
                                uart_send_cstr("You caught the player!\r\n");
                            }
                        } else {
                            /* 猜错 → 逃掉 */
                            if (role_is_player)
                                uart_send_cstr("Player escaped by choosing different direction!\r\n");
                            else
                                uart_send_cstr("Player escaped, try again.\r\n");
                        }

                        wait_escape    = 0;
                        g_use_dir_game = 0;
                        return;
                    }
                }
            }
        }


        /* 3) 时间到还没逃脱 */
        if (remain_ms == 0) {
            if (role_is_player) {
                game_end("Game Over!\r\n");
            } else {
                uart_send_cstr("Player escaped! Keep trying.\r\n");
            }
            wait_escape = 0;
            g_use_dir_game = 0;
        }

        return; /* 窗口中只做这些 */
    }

    /* === 不在窗口中，正常 Game2 逻辑 === */

    /* 每秒检查环境一次 (跟第二个 main 一致) */
    static uint32_t last_env_poll = 0;
    if (is_due(now, last_env_poll + 1000U)) {
        last_env_poll = now;
        float t = read_temp_C();
        float h = read_humi_pct();
        float p = read_pres_hPa();

        int t10 = (int)(t * 10.0f + (t>=0?0.5f:-0.5f));
        int h10 = (int)(h * 10.0f + (h>=0?0.5f:-0.5f));
        int p10 = (int)(p * 10.0f + (p>=0?0.5f:-0.5f));

        char short_msg[32] = {0};

        if (t > ENV_TEMP_SPIKE_C) {
            uart_printf_nooled("Temperature spike! T=%d.%01dC (>%dC)\r\n",
                               t10/10, abs(t10%10), (int)ENV_TEMP_SPIKE_C);
            snprintf(short_msg, sizeof(short_msg), "T high %d.%1dC", t10/10, abs(t10%10));
        } else if (h > ENV_HUMI_SPIKE_PCT) {
            uart_printf_nooled("Humidity spike! H=%d.%01d%% (>%d%%)\r\n",
                               h10/10, abs(h10%10), (int)ENV_HUMI_SPIKE_PCT);
            snprintf(short_msg, sizeof(short_msg), "H high %d.%1d%%", h10/10, abs(h10%10));
        } else if (p > ENV_PRES_SPIKE_HPA) {
            uart_printf_nooled("Pressure spike! P=%d.%01dhPa (>%d hPa)\r\n",
                               p10/10, abs(p10%10), (int)ENV_PRES_SPIKE_HPA);
            snprintf(short_msg, sizeof(short_msg), "P high %d.%1dh", p10/10, abs(p10%10));
        }

        if (short_msg[0]) {
            SeeedOLED_SetTextXY(3, 0);
            SeeedOLED_PutString("                ");
            SeeedOLED_SetTextXY(3, 0);
            SeeedOLED_PutString(short_msg);
        }
    }

    /* 用磁力计估算距离 */
    float bn = mag_norm_uT();

    /* 按磁场强度灯闪烁 */
    uint32_t blink_ms = 0;
    if (bn > MAG_BAND3_uT)      blink_ms = BLINK_FAST_MS;
    else if (bn > MAG_BAND2_uT) blink_ms = BLINK_MED_MS;
    else if (bn > MAG_BAND1_uT) blink_ms = BLINK_SLOW_MS;
    else                        blink_ms = 0;

    if (blink_ms) LED_Blink_step(blink_ms);
    else          LED_Off();

    /* 触发靠近 -> 开 3 秒窗口 + 生成方向 (合并两个 main 的做法) */
    if (bn > MAG_NEAR_THRESH_uT) {
        if (role_is_player) {
            uart_send_cstr("Enforcer nearby! Be careful.\r\n");
            oled_printf_short("ALERT: Enforcer!");
        } else {
            uart_send_cstr("Player is Nearby! Move faster.\r\n");
            oled_printf_short("ALERT: Enforcer!");
        }

        Buzzer_Beep(200);

        /* 有 Grove 的话：这一次窗口一定玩方向博弈 */
        if (g_grove_ok) {
            uint32_t seed = ms_now();
            g_enforcer_dir = (uint8_t)(seed & 0x03);   // 0~3
            g_use_dir_game = 1;                        // ←←← 关键：只要有 Grove 就开方向
            const char *dir_name[4] = {"UP","DOWN","LEFT","RIGHT"};
            uart_printf("[G2] Enforcer direction = %s\r\n", dir_name[g_enforcer_dir]);
        } else {
            /* 没 Grove：退回 PC13 方案 */
            g_use_dir_game = 0;
        }

        wait_escape     = 1;
        escape_deadline = now + BUTTON_ESCAPE_WINDOW_MS;
        return;
    }


    /* 正常情况也要处理 CLI / 按键 */
    cli_poll();
    maybe_switch_mode_on_double_click();
}

/* ====== Sensors / Helpers ====== */
static void Sensors_Init(void)
{
    if (BSP_ACCELERO_Init() != 0) Error_Handler();
    if (BSP_GYRO_Init()     != 0) Error_Handler();
    if (BSP_MAGNETO_Init()  != 0) Error_Handler();
    if (BSP_TSENSOR_Init()  != 0) Error_Handler();
    if (BSP_HSENSOR_Init()  != 0) Error_Handler();
    if (BSP_PSENSOR_Init()  != 0) Error_Handler();
}

static float accel_norm_g(void)
{
    int16_t axyz[3];
    BSP_ACCELERO_AccGetXYZ(axyz);
    float fx = axyz[0] / 1000.0f;
    float fy = axyz[1] / 1000.0f;
    float fz = axyz[2] / 1000.0f - 1.01f;
    return sqrtf(fx*fx + fy*fy + fz*fz);
}

static float gyro_norm_dps(void)
{
    float gxyz[3] = {0.f,0.f,0.f};
    BSP_GYRO_GetXYZ(gxyz);
    float gx = gxyz[0], gy = gxyz[1], gz = gxyz[2];
    gx -= 70.0f; gy += 980.0f; gz -= 770.0f;
    gx *= 0.001f; gy *= 0.001f; gz *= 0.001f;
    return sqrtf(gx*gx + gy*gy + gz*gz);
}

static float mag_norm_uT(void)
{
    int16_t mxyz[3];
    BSP_MAGNETO_GetXYZ(mxyz);  /* mGauss */
    float fx = mxyz[0] * 0.1f;
    float fy = mxyz[1] * 0.1f;
    float fz = mxyz[2] * 0.1f;
    return sqrtf(fx*fx + fy*fy + fz*fz);
}

static float read_temp_C(void)  { return HTS221_ReadTemperatureCached(HTS221_DEV_ADDR); }
static float read_humi_pct(void){ return HTS221_ReadHumidityCached(HTS221_DEV_ADDR); }
static float read_pres_hPa(void){ return LPS22HB_ReadPressureCached(LPS22HB_DEV_ADDR); }

/* ==== LED ==== */
static void LED_On(void)      { HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_SET); }
static void LED_Off(void)     { HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET); }
static void LED_Toggle(void)  { HAL_GPIO_TogglePin(LED2_GPIO_PORT, LED2_PIN); }
static void LED_Blink_step(uint32_t period_ms)
{
    static uint32_t last = 0;
    uint32_t now = ms_now();
    if (is_due(now, last + period_ms)) {
        last = now;
        LED_Toggle();
    }
}

/* ==== Button / Mode Switch ==== */
static uint8_t button_single_press_happened_since(uint32_t since_ms)
{
    static uint32_t last_report_ts = 0;
    if (first_press_state == 1) {
        if (is_due(ms_now(), first_press_ts + BTN_DOUBLE_WINDOW_MS)) {
            first_press_state = 0;
            if (first_press_ts >= since_ms && first_press_ts != last_report_ts) {
                last_report_ts = first_press_ts;
                return 1;
            }
        }
    }
    return 0;
}

static void maybe_switch_mode_on_double_click(void)
{
    if (first_press_state == 2) {
        first_press_state = 0;
        /* 切模式 */
        g_mode = (g_mode == MODE_RED_GREEN) ? MODE_CATCH_RUN : MODE_RED_GREEN;

        {
            char msg[80];
            if (g_mode == MODE_RED_GREEN) {
                if (role_is_player)
                    sprintf(msg, "Entering Game 1 Red Light, Green Light as Player\r\n");
                else
                    sprintf(msg, "Entering Game 1 Red Light, Green Light as Enforcer\r\n");
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 0xFFFF);

                g_phase = PHASE_GREEN;
                phase_t0 = ms_now();
                last_env_transmit = phase_t0;
                LED_On();
            } else {
                if (role_is_player)
                    sprintf(msg, "Entering Game 2 Catch And Run as Player\r\n");
                else
                    sprintf(msg, "Entering Game 2 Catch And Run as Enforcer\r\n");
                HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 0xFFFF);

                LED_Off();
                wait_escape = 0;
                g_use_dir_game = 0;
            }
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin != USER_BTN_PIN) return;
    uint32_t now = ms_now();
    if (!is_due(now, btn_last_isr_ts + BTN_DEBOUNCE_MS)) return;
    btn_last_isr_ts = now;

    if (first_press_state == 0) {
        first_press_state = 1;
        first_press_ts = now;
    } else if (first_press_state == 1) {
        if (!is_due(now, first_press_ts + BTN_DOUBLE_WINDOW_MS)) {
            first_press_state = 2;
        } else {
            first_press_state = 1;
            first_press_ts = now;
        }
    }
}

/* ==== CLI ==== */
/* 环形缓冲 */
static inline int rxq_push(uint8_t c){
    uint16_t n = (uint16_t)((rxq_head + 1u) % RXQ_SIZE);
    if (n == rxq_tail) return 0;
    rxq[rxq_head] = c; rxq_head = n;
    return 1;
}
static inline int rxq_pop(uint8_t *pc){
    if (rxq_head == rxq_tail) return 0;
    *pc = rxq[rxq_tail];
    rxq_tail = (uint16_t)((rxq_tail + 1u) % RXQ_SIZE);
    return 1;
}

static void cli_poll(void)
{
    uint8_t c;
    while (rxq_pop(&c)) {
        if (c == '\n') continue;
        if (c == '\r' || cli_len >= 63) {
            cli_line[cli_len] = '\0';
            if (cli_len) cli_handle_line(cli_line);
            cli_len = 0;
        } else if (c == 0x08 || c == 0x7F) {
            if (cli_len) cli_len--;
        } else {
            cli_line[cli_len++] = (char)c;
        }
    }
}

static void cli_handle_line(const char *s)
{
    char buf[64];
    strncpy(buf, s, sizeof(buf));
    buf[sizeof(buf)-1] = '\0';
    for (int i = (int)strlen(buf)-1; i >= 0 && (buf[i]==' ' || buf[i]=='\t'); --i) buf[i] = '\0';

    if (buf[0] == '\0') {
        uart_send_cstr("help: g1|g2 | role p|role e | thr a <g> | thr w <dps> | dump\r\n");
        return;
    }

    if (strcmp(buf, "g1") == 0) {
        g_mode  = MODE_RED_GREEN;
        g_phase = PHASE_GREEN;
        uint32_t now = ms_now();
        phase_t0 = now;
        last_env_transmit = now;
        LED_On();
        uart_send_cstr("OK: switch to Game1 (Green phase)\r\n");
        return;
    }

    if (strcmp(buf, "g2") == 0) {
        g_mode = MODE_CATCH_RUN;
        wait_escape = 0;
        g_use_dir_game = 0;
        LED_Off();
        uart_send_cstr("OK: switch to Game2\r\n");
        return;
    }

    if (strcmp(buf, "role p") == 0) {
        role_is_player = 1;
        uart_send_cstr("OK: role=Player\r\n");
        return;
    }
    if (strcmp(buf, "role e") == 0) {
        role_is_player = 0;
        uart_send_cstr("OK: role=Enforcer\r\n");
        return;
    }

    if (strncmp(buf, "thr a ", 6) == 0) {
        float v = strtof(buf + 6, NULL);
        if (v > 0.01f && v < 5.0f) {
            g_acc_thr_g = v;
            uart_printf("OK: ACC thr=%.3fg\r\n", g_acc_thr_g);
        } else {
            uart_send_cstr("ERR: acc thr out of range (0.01~5.0)\r\n");
        }
        return;
    }

    if (strncmp(buf, "thr w ", 6) == 0) {
        float v = strtof(buf + 6, NULL);
        if (v > 1.0f && v < 2000.0f) {
            g_gyr_thr_dps = v;
            uart_printf("OK: GYR thr=%.1fdps\r\n", g_gyr_thr_dps);
        } else {
            uart_send_cstr("ERR: gyro thr out of range (1~2000)\r\n");
        }
        return;
    }

    if (strcmp(buf, "dump") == 0) {
        float an = accel_norm_g();
        float wn = gyro_norm_dps();
        float bn = mag_norm_uT();

        int a_thr_1000 = (int)(g_acc_thr_g    * 1000.0f + 0.5f);
        int w_thr_10   = (int)(g_gyr_thr_dps  * 10.0f   + 0.5f);
        int a_1000     = (int)(an * 1000.0f   + 0.5f);
        int w_10       = (int)(wn * 10.0f     + 0.5f);
        int B_10       = (int)(bn * 10.0f     + 0.5f);

        uart_printf("mode=%d phase=%d player=%d "
                    "a_thr=%d.%03dg w_thr=%d.%01ddps "
                    "|a|=%d.%03fg |w|=%d.%01fdps |B|=%d.%01fuT\r\n",
                    (int)g_mode, (int)g_phase, role_is_player,
                    a_thr_1000/1000, abs(a_thr_1000%1000),
                    w_thr_10/10,     abs(w_thr_10%10),
                    a_1000/1000,     abs(a_1000%1000),
                    w_10/10,         abs(w_10%10),
                    B_10/10,         abs(B_10%10));
        return;
    }

    uart_send_cstr("help: g1|g2 | role p|role e | thr a <g> | thr w <dps> | dump\r\n");
}

/* ==== UART print ==== */
static void uart_tx(const uint8_t *buf, size_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)buf, (uint16_t)len, HAL_MAX_DELAY);
}
static void uart_printf(const char *fmt, ...)
{
    char buf[256];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0)
        uart_tx((uint8_t*)buf, (size_t)((n < (int)sizeof(buf)) ? n : (int)sizeof(buf)));

    /* 先不要每条都刷OLED，会拖慢整个系统 */
    // oled_log_puts(buf);
}
static void uart_printf_nooled(const char *fmt, ...)
{
    char buf[256];
    va_list ap; va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) HAL_UART_Transmit(&huart1, (uint8_t*)buf,
                                 (uint16_t)((n < (int)sizeof(buf)) ? n : (int)sizeof(buf)),
                                 HAL_MAX_DELAY);
}
static void oled_printf_short(const char *fmt, ...)
{
    char buf[32];
    va_list ap; va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    for (int i=0; buf[i]; i++) {
        if (buf[i] < 32 || buf[i] > 126) buf[i] = ' ';
    }
    oled_log_puts(buf);
}
static void oled_show_env(float t, float h, float p)
{
    char line[17];
    int t10 = (int)(t * 10.0f + (t>=0?0.5f:-0.5f));
    int h10 = (int)(h * 10.0f + (h>=0?0.5f:-0.5f));
    int p10 = (int)(p * 10.0f + (p>=0?0.5f:-0.5f));

    /* 行1：温度 */
    SeeedOLED_SetTextXY(0,0);
    snprintf(line, sizeof(line), "T:%02d.%1dC     ", t10/10, abs(t10%10));
    SeeedOLED_PutString(line);

    /* 行2：湿度 */
    SeeedOLED_SetTextXY(1,0);
    snprintf(line, sizeof(line), "H:%02d.%1d%%    ", h10/10, abs(h10%10));
    SeeedOLED_PutString(line);

    /* 行3：气压 */
    SeeedOLED_SetTextXY(2,0);
    snprintf(line, sizeof(line), "P:%04d.%1dh ", p10/10, abs(p10%10));
    SeeedOLED_PutString(line);

    /* 行4：状态 */
    SeeedOLED_SetTextXY(3,0);
    if (g_mode == MODE_RED_GREEN) {
        SeeedOLED_PutString(g_phase == PHASE_GREEN ? "Game1 GREEN   " : "Game1  RED    ");
    } else {
        SeeedOLED_PutString("Game2  RUN     ");
    }
}


/* ==== OLED log ==== */
static void oled_log_init(void)
{
    for (int i=0;i<OLED_LOG_LINES;i++)
        memset(oled_lines[i], 0, OLED_LINE_LEN+1);
    SeeedOLED_ClearDisplay();
    oled_log_render();
}
static void oled_log_render(void)
{
    for (int row=0; row<OLED_LOG_LINES; row++) {
        SeeedOLED_SetTextXY(row,0);
        SeeedOLED_PutString(oled_lines[row]);
        int len = strlen(oled_lines[row]);
        for (int k=len; k<OLED_LINE_LEN; k++)
            SeeedOLED_PutChar(' ');
    }
}
static void oled_log_puts(const char *s)
{
    while (*s) {
        char chunk[OLED_LINE_LEN+1];
        int idx = 0;
        while (*s && idx < OLED_LINE_LEN) {
            char c = *s++;
            if (c=='\r' || c=='\n') break;
            chunk[idx++] = c;
        }
        chunk[idx] = '\0';

        /* 内存里滚动4行 */
        for (int i = 0; i < OLED_LOG_LINES - 1; i++)
            memcpy(oled_lines[i], oled_lines[i+1], OLED_LINE_LEN+1);
        memcpy(oled_lines[OLED_LOG_LINES-1], chunk, OLED_LINE_LEN+1);

        /* 只写最后一行，别整屏刷 */
        SeeedOLED_SetTextXY(OLED_LOG_LINES-1, 0);
        SeeedOLED_PutString(oled_lines[OLED_LOG_LINES-1]);
        int len = strlen(oled_lines[OLED_LOG_LINES-1]);
        for (int k = len; k < OLED_LINE_LEN; k++)
            SeeedOLED_PutChar(' ');

        while (*s=='\r' || *s=='\n') s++;
    }
}


/* ==== 初始化 ==== */
static void MX_GPIO_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* LED */
    GPIO_InitStruct.Pin   = LED2_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStruct);
    LED_Off();

    /* User Button PC13 */
    GPIO_InitStruct.Pin  = USER_BTN_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(USER_BTN_GPIO_PORT, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(USER_BTN_EXTI_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USER_BTN_EXTI_IRQn);
}

static void MX_IR_GPIO_Init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin  = IR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(IR_GPIO_PORT, &GPIO_InitStruct);
}

static void MX_USART1_UART_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = 115200;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
    HAL_NVIC_SetPriority(USART1_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

static void MX_I2C1_Init(void)
{
    /* I2C1 on PB8/PB9, AF4 */
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
    PeriphClkInit.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hi2c1.Instance             = I2C1;
    hi2c1.Init.Timing          = 0x00303D5B;   /* 100 kHz，稳定版 */
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.OwnAddress2Masks= I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        Error_Handler();
    }
}

/* USART1 IRQ */
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}

/* UART RX 完成回调 (带回显 + 入队 + 继续收) */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) return;

    uint8_t c = rx_it_byte;

    if (c == '\r') {
        const char *crlf = "\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)crlf, 2, 10);
    } else {
        HAL_UART_Transmit(&huart1, &c, 1, 10);
    }

    rxq_push(c);
    HAL_UART_Receive_IT(&huart1, &rx_it_byte, 1);
}

/* USART1 clock source -> HSI */
static void UartClock_UseHSI_ForUSART1(void)
{
    __HAL_RCC_HSI_ENABLE();
    while (!__HAL_RCC_GET_FLAG(RCC_FLAG_HSIRDY)) { }

    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        Error_Handler();
    }
}

/* Game 结束 */
static void game_end(const char* reason)
{
    game_over = 1;
    wait_escape = 0;
    g_use_dir_game = 0;
    uart_send_cstr(reason);
    game_over_since = ms_now();
    LED_Off();
}

/* 简单串口输出 + OLED log 同步接口上面已经写了 */

/* Error Handler */
void Error_Handler(void)
{
    __disable_irq();
    if (huart1.Instance) {
        const char *s = "ERR\r\n";
        HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), 10);
    }
    while (1) {
        LED_Blink_step(120);
    }
}

static void uart_send_cstr(const char *s)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)s, strlen(s), 0xFFFF);
    /* 先不要每条都刷OLED，会拖慢整个系统 */
    // oled_log_puts(s);
}

/* HTS221 湿度快速读取函数 */
typedef struct {
  int inited;   //是否已经完成过校准寄存器的读取flag。
  float m;      // 斜率m： (H1_rh - H0_rh) / (H1_out - H0_out)
  float b;      // 截距b： H0_rh - m * H0_out
} hts221_humidity_calibration_type;
// 声明一个静态变量 static_humidity_calibration 作为全局缓存，把成员全置零
static hts221_humidity_calibration_type static_humidity_calibration = {0};
static void hts221_calibration_init_once(uint16_t dev)
{
	// 如果已经初始化过（inited==1），就直接返回，避免重复读 I²C
	if (static_humidity_calibration.inited) return;
	// 准备一个 2 字节缓冲 buf 用来装每次读出的低/高字节
	uint8_t buf[2];
	// 两点校准湿度
	int16_t H0_rh, H1_rh;
	// 两点校准时对应的 ADC 原始输出（有符号 16 位）
	int16_t H0_T0_out, H1_T0_out;
	// 读 H0/H1_rH_x2（0.5%RH）→ 转成 %RH（右移一位）
	// 从地址 H0_RH_X2 连续读 2 字节（| 0x80 置位自动地址递增位，可以一次读两个相邻寄存器）
	// buf[0]：H0_RH_X2（单位是0.5%RH） buf[1]：H1_RH_X2（同样是 0.5%RH）
	SENSOR_IO_ReadMultiple(dev, (HTS221_H0_RH_X2 | 0x80), buf, 2);
	H0_rh = buf[0] >> 1;
	H1_rh = buf[1] >> 1;
	// 读 H0_T0_OUT
	// 从 H0_T0_OUT_L 开始连续读两字节（低在前，高在后），组合成有符号 16 位
	SENSOR_IO_ReadMultiple(dev, (HTS221_H0_T0_OUT_L | 0x80), buf, 2);
	H0_T0_out = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
	// 读 H1_T0_OUT
	SENSOR_IO_ReadMultiple(dev, (HTS221_H1_T0_OUT_L | 0x80), buf, 2);
	H1_T0_out = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
	// 预计算线性映射参数：RH = m * H_T_out + b
	float denom = (float)(H1_T0_out - H0_T0_out);
	// 一旦校准数据异常（两点 ADC 输出相同导致除零），程序会立刻进入 Error_Handler()
	if (denom == 0.0f) {
		Error_Handler();         // 进入错误处理（一般会停机并闪灯）
		return;
	}
	static_humidity_calibration.m = (float)(H1_rh - H0_rh) / denom;
	static_humidity_calibration.b = (float)H0_rh - static_humidity_calibration.m * (float)H0_T0_out;
	// 标记初始化完成，下次再调用就会直接返回，不重复读 I²C。
	static_humidity_calibration.inited = 1;
}
/* 只读一次校准，以后每次仅读 HR_OUT 并做一次乘加 */
float HTS221_ReadHumidityCached(uint16_t dev)
{
  hts221_calibration_init_once(dev);
  uint8_t buf[2];
  SENSOR_IO_ReadMultiple(dev, (HTS221_HR_OUT_L_REG | 0x80), buf, 2);
// 从当前湿度原始输出寄存器 HR_OUT_L 连续读 2 字节（低→高），组装成 int16_t 的原始 ADC 值 H_T_out
  int16_t H_T_out = (int16_t)(((uint16_t)buf[1] << 8) | buf[0]);
  // 线性映射：RH = m * H_T_out + b
  float rh = static_humidity_calibration.m * (float)H_T_out + static_humidity_calibration.b;
  // 对结果做物理范围裁剪：相对湿度限定在 0~100% 之间
  if (rh < 0.0f)   rh = 0.0f;
  if (rh > 100.0f) rh = 100.0f;
  return rh;  // 单位：%RH
}

/* HTS221 温度快速读取函数 */
typedef struct {
  int initialized;              /* 是否完成过一次性校准寄存器读取flag */
  float slope_degrees_per_count;/* 斜率： (T1_degC - T0_degC) / (T1_out - T0_out) */
  float intercept_degrees;      /* 截距： T0_degC - slope * T0_out */
} hts221_temperature_calibration_t;
/* 全局静态缓存 */
static hts221_temperature_calibration_t temperature_calibration_cache = {0};
/* 一次性读取并计算温度校准参数：T = slope * TEMP_OUT + intercept */
static void hts221_temperature_calibration_initialize_once(uint16_t device_address)
{
	if (temperature_calibration_cache.initialized) {
	  return; /* 已初始化则直接返回，避免重复 I2C 访问 */
	}
	uint8_t buffer[2];
	/* 1) 读取 T0_degC_x8 与 T1_degC_x8 的低 8 位（单位为 1/8 °C） */
	SENSOR_IO_ReadMultiple(device_address, (HTS221_T0_DEGC_X8 | 0x80), buffer, 2);
	uint8_t temperature_point0_degrees_celsius_times8_least_significant_byte = buffer[0];
	uint8_t temperature_point1_degrees_celsius_times8_least_significant_byte = buffer[1];
	/* 2) 读取高位寄存器 T1_T0_MSB：
		bit[0:1] 是 T0_degC_x8 的高 2 位
		bit[2:3] 是 T1_degC_x8 的高 2 位 */
	uint8_t t1_t0_most_significant_bits;
	SENSOR_IO_ReadMultiple(device_address, HTS221_T1_T0_MSB, &t1_t0_most_significant_bits, 1);
	/* 3) 组合出 10 位的 T0_degC_x8 与 T1_degC_x8 */
	uint16_t temperature_point0_degrees_celsius_times8 =
	  (uint16_t)(((uint16_t)(t1_t0_most_significant_bits & 0x03) << 8) |
				 (uint16_t)temperature_point0_degrees_celsius_times8_least_significant_byte);
	uint16_t temperature_point1_degrees_celsius_times8 =
	  (uint16_t)((((uint16_t)(t1_t0_most_significant_bits & 0x0C) >> 2) << 8) |
				 (uint16_t)temperature_point1_degrees_celsius_times8_least_significant_byte);
	/* 4) 转换为摄氏度（除以 8） */
	float temperature_point0_degrees_celsius =
	  ((float)temperature_point0_degrees_celsius_times8) / 8.0f;
	float temperature_point1_degrees_celsius =
	  ((float)temperature_point1_degrees_celsius_times8) / 8.0f;
	/* 5) 读取 T0_OUT 与 T1_OUT（温度在两点标定时对应的 ADC 原始输出，带符号） */
	SENSOR_IO_ReadMultiple(device_address, (HTS221_T0_OUT_L | 0x80), buffer, 2);
	int16_t temperature_point0_output_count =
	  (int16_t)(((uint16_t)buffer[1] << 8) | (uint16_t)buffer[0]);
	SENSOR_IO_ReadMultiple(device_address, (HTS221_T1_OUT_L | 0x80), buffer, 2);
	int16_t temperature_point1_output_count =
	  (int16_t)(((uint16_t)buffer[1] << 8) | (uint16_t)buffer[0]);
	/* 6) 计算线性映射参数：T = slope * TEMP_OUT + intercept */
	float output_count_difference =
	  (float)((int32_t)temperature_point1_output_count - (int32_t)temperature_point0_output_count);
	if (output_count_difference == 0.0f) {
	  /* 异常：两点原始输出相同会导致除零，进入错误处理 */
	  Error_Handler();
	  return;
	}
	float temperature_difference_degrees =
	  temperature_point1_degrees_celsius - temperature_point0_degrees_celsius;
	temperature_calibration_cache.slope_degrees_per_count =
	  temperature_difference_degrees / output_count_difference;
	temperature_calibration_cache.intercept_degrees =
	  temperature_point0_degrees_celsius -
	  temperature_calibration_cache.slope_degrees_per_count *
	  (float)temperature_point0_output_count;
	/* 7) 标记已完成初始化 */
	temperature_calibration_cache.initialized = 1;
}
/* 只读一次校准；后续每次仅仅读 TEMP_OUT 并做一次乘加 */
float HTS221_ReadTemperatureCached(uint16_t device_address)
{
	hts221_temperature_calibration_initialize_once(device_address);
	uint8_t buffer[2];
	/* 1) 读取当前温度原始输出 TEMP_OUT（带符号，LSB在前） */
	SENSOR_IO_ReadMultiple(device_address, (HTS221_TEMP_OUT_L_REG | 0x80), buffer, 2);
	int16_t temperature_output_count =
	  (int16_t)(((uint16_t)buffer[1] << 8) | (uint16_t)buffer[0]);
	/* 2) 线性映射为摄氏度：T = slope * TEMP_OUT + intercept */
	float temperature_degrees_celsius =
	  temperature_calibration_cache.slope_degrees_per_count *
	  (float)temperature_output_count +
	  temperature_calibration_cache.intercept_degrees;
	/* 3)物理范围裁剪，先随便设置了一个 [-40, 80] ℃
	 if (temperature_degrees_celsius < -40.0f)  temperature_degrees_celsius = -40.0f;
	 if (temperature_degrees_celsius > 120.0f)  temperature_degrees_celsius = 80.0f;
	*/
	return temperature_degrees_celsius;
}

/* LPS22HB Fast Pressure Read (init-once + burst read +  EMA) */
typedef struct {
    int   inited;        /* 是否已完成一次性初始化 */
    int   ema_ready;     /* EMA 是否已有第一帧 */
    float ema_alpha;     /* EMA 系数 */
    float ema_value;     /* EMA 当前值 (hPa) */
} lps22hb_fast_ctx_t;
static lps22hb_fast_ctx_t g_lps22hb = {0};

#ifndef LPS22HB_STATUS_REG
#define LPS22HB_STATUS_REG 0x27
#endif

static int LPS22HB_InitOnce_Quick(uint16_t dev)
{
    static int inited = 0;
    if (inited) return 1;

    SENSOR_IO_Init();

    /* RES_CONF：低功耗模式 */
    uint8_t res = SENSOR_IO_Read(dev, LPS22HB_RES_CONF_REG);
    res &= ~LPS22HB_LCEN_MASK;
    res |= 0x01;
    SENSOR_IO_Write(dev, LPS22HB_RES_CONF_REG, res);

    /* CTRL_REG1：ODR=25Hz，BDU=1 */
    uint8_t c1 = SENSOR_IO_Read(dev, LPS22HB_CTRL_REG1);
    c1 &= ~LPS22HB_ODR_MASK;   // 清 ODR
    c1 |=  0x30;               // 25Hz
    c1 &= ~LPS22HB_BDU_MASK;   // 清 BDU 位
    c1 |=  0x02;               // 置 BDU=1
    SENSOR_IO_Write(dev, LPS22HB_CTRL_REG1, c1);

    /*  EMA 参数 */
    g_lps22hb.inited    = 1;
    g_lps22hb.ema_ready = 0;
    g_lps22hb.ema_alpha = 0.20f;

    inited = 1;
    return 1;
}
/* 读取一次原始压力并换算为 hPa） */
static float LPS22HB_ReadPressureRaw_hPa(uint16_t dev)
{
    uint8_t b0 = SENSOR_IO_Read(dev, LPS22HB_PRESS_OUT_XL_REG + 0);
    uint8_t b1 = SENSOR_IO_Read(dev, LPS22HB_PRESS_OUT_XL_REG + 1);
    uint8_t b2 = SENSOR_IO_Read(dev, LPS22HB_PRESS_OUT_XL_REG + 2);

    /* 坏帧保护 */
    if ((b0 | b1 | b2) == 0x00) return NAN;

    uint32_t tmp = ((uint32_t)b0) | ((uint32_t)b1 << 8) | ((uint32_t)b2 << 16);
    if (tmp & 0x00800000U) tmp |= 0xFF000000U;      // 符号扩展为 32bit
    int32_t raw = (int32_t)tmp;                     // LSB = 1/4096 hPa

    /* 换算： (raw*100)/4096 再 /100 → hPa
       用等效整数：raw * 25 / 1024 => 0.01 hPa */
    int32_t cHpa = (raw * 25) / 1024;               // 0.01 hPa
    return ((float)cHpa) / 100.0f;                  // hPa
}
/* 对外快速读（带 EMA） */
float LPS22HB_ReadPressureCached(uint16_t dev)
{
    if (!LPS22HB_InitOnce_Quick(dev)) {
        return NAN;
    }

    /* 只看一次状态，不等 */
    uint8_t stat = SENSOR_IO_Read(dev, LPS22HB_STATUS_REG);
    if (!(stat & 0x02)) {
        /* 还没新数据，就先用上一次的 */
        if (g_lps22hb.ema_ready)
            return g_lps22hb.ema_value;
        else
            return 0.0f;   /* 初次还没数的话就给个0或1013都行 */
    }

    float p = LPS22HB_ReadPressureRaw_hPa(dev);
    if (!isnanf(p)) {
        if (!g_lps22hb.ema_ready) {
            g_lps22hb.ema_value = p;
            g_lps22hb.ema_ready = 1;
        } else {
            g_lps22hb.ema_value += g_lps22hb.ema_alpha * (p - g_lps22hb.ema_value);
        }
    }
    return g_lps22hb.ema_value;
}

static void I2C1_TryRecover(void)
{
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY)) {
        HAL_I2C_DeInit(&hi2c1);
        MX_I2C1_Init();
    }
}


