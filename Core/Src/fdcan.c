#include "fdcan.h"

FDCAN_HandleTypeDef hfdcan2;

void MX_FDCAN2_Init(void)
{
    hfdcan2.Instance = FDCAN2;
    hfdcan2.Init.ClockDivider = FDCAN_CLOCK_DIV1;
    hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan2.Init.AutoRetransmission = ENABLE;
    hfdcan2.Init.TransmitPause = DISABLE;
    hfdcan2.Init.ProtocolException = DISABLE;

    /*
     * FDCAN kernel = PCLK1 = 170 MHz
     * 170 MHz / 17 / (1 + 15 + 4) = 500000 bit/s
     * sample point = (1 + 15) / 20 = 80 %
     */
    hfdcan2.Init.NominalPrescaler = 17;
    hfdcan2.Init.NominalSyncJumpWidth = 4;
    hfdcan2.Init.NominalTimeSeg1 = 15;
    hfdcan2.Init.NominalTimeSeg2 = 4;

    hfdcan2.Init.DataPrescaler = 1;
    hfdcan2.Init.DataSyncJumpWidth = 1;
    hfdcan2.Init.DataTimeSeg1 = 1;
    hfdcan2.Init.DataTimeSeg2 = 1;

    hfdcan2.Init.StdFiltersNbr = 0;
    hfdcan2.Init.ExtFiltersNbr = 1;
    hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

    if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
        Error_Handler();
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcanHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    if (fdcanHandle->Instance != FDCAN2)
        return;

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
        Error_Handler();

    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* PB12 FDCAN2_RX, PB13 FDCAN2_TX, AF9 */
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *fdcanHandle)
{
    if (fdcanHandle->Instance != FDCAN2)
        return;

    __HAL_RCC_FDCAN_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12 | GPIO_PIN_13);
    HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
}
