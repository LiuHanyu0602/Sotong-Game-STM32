#include "buzzer.h"

static uint32_t buzzer_deadline = 0;
static uint8_t  buzzer_active = 0;

void Buzzer_Init(void)
{
    /* 开 GPIOA 时钟，若你改了端口要同步改 */
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin   = BUZZER_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUZZER_GPIO_PORT, &GPIO_InitStruct);

    Buzzer_Off();
}

void Buzzer_On(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_PIN, GPIO_PIN_SET);
}

void Buzzer_Off(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_PIN, GPIO_PIN_RESET);
    buzzer_active = 0;
}

/* 开一个持续 ms 的蜂鸣，非阻塞 */
void Buzzer_Beep(uint32_t ms)
{
    Buzzer_On();
    buzzer_active = 1;
    buzzer_deadline = HAL_GetTick() + ms;
}

/* 放到 while(1) 里一直跑 */
void Buzzer_Task(void)
{
    if (buzzer_active) {
        if ((int32_t)(HAL_GetTick() - buzzer_deadline) >= 0) {
            Buzzer_Off();
        }
    }
}
