#include "app_uart.h"
#include "usart.h"
#include "vofa_debug.h"

static uint8_t s_rx1;
static uint8_t s_rx5;
static Hwt101 s_imu;

static void arm_rx(UART_HandleTypeDef *huart, uint8_t *byte)
{
    (void)HAL_UART_Receive_IT(huart, byte, 1U);
}

void AppUart_Init(void)
{
    hwt101_init(&s_imu);
    arm_rx(&huart1, &s_rx1);
    arm_rx(&huart5, &s_rx5);
}

Hwt101 *AppUart_GetImu(void)
{
    return &s_imu;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uint32_t now = HAL_GetTick();

    if (huart == &huart1) {
        hwt101_feed_byte(&s_imu, s_rx1, now);
        arm_rx(&huart1, &s_rx1);
    } else if (huart == &huart5) {
        VofaDebug_FeedByte(s_rx5);
        arm_rx(&huart5, &s_rx5);
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (!huart) return;

    __HAL_UART_CLEAR_PEFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_OREFLAG(huart);
    (void)HAL_UART_AbortReceive(huart);

    if (huart == &huart1) arm_rx(&huart1, &s_rx1);
    else if (huart == &huart5) arm_rx(&huart5, &s_rx5);
}
