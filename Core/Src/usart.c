#include "usart.h"

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;

static void UART_CommonInit(UART_HandleTypeDef *huart)
{
    huart->Init.BaudRate = 115200;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_NONE;
    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;
    huart->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart->Init.ClockPrescaler = UART_PRESCALER_DIV1;
}

static void UART_PostInit(UART_HandleTypeDef *huart)
{
    if (HAL_UARTEx_SetTxFifoThreshold(huart, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
        Error_Handler();
    if (HAL_UARTEx_SetRxFifoThreshold(huart, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
        Error_Handler();
    if (HAL_UARTEx_DisableFifoMode(huart) != HAL_OK)
        Error_Handler();
}

void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    UART_CommonInit(&huart1);
    if (HAL_UART_Init(&huart1) != HAL_OK) Error_Handler();
    UART_PostInit(&huart1);
}

void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    UART_CommonInit(&huart2);
    if (HAL_UART_Init(&huart2) != HAL_OK) Error_Handler();
    UART_PostInit(&huart2);
}

void MX_USART3_UART_Init(void)
{
    huart3.Instance = USART3;
    UART_CommonInit(&huart3);
    if (HAL_UART_Init(&huart3) != HAL_OK) Error_Handler();
    UART_PostInit(&huart3);
}

void MX_UART4_Init(void)
{
    huart4.Instance = UART4;
    UART_CommonInit(&huart4);
    if (HAL_UART_Init(&huart4) != HAL_OK) Error_Handler();
    UART_PostInit(&huart4);
}

void MX_UART5_Init(void)
{
    huart5.Instance = UART5;
    UART_CommonInit(&huart5);
    if (HAL_UART_Init(&huart5) != HAL_OK) Error_Handler();
    UART_PostInit(&huart5);
}

void HAL_UART_MspInit(UART_HandleTypeDef *uartHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    if (uartHandle->Instance == USART1)
    {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
        PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();

        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        /* PC4 TX, PC5 RX */
        GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
    else if (uartHandle->Instance == USART2)
    {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
        PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();

        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        /* PA2 TX, PA3 RX */
        GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART2_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
    else if (uartHandle->Instance == USART3)
    {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART3;
        PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();

        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOE_CLK_ENABLE();

        /* PB10 TX */
        GPIO_InitStruct.Pin = GPIO_PIN_10;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* PE15 RX */
        GPIO_InitStruct.Pin = GPIO_PIN_15;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART3_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
    else if (uartHandle->Instance == UART4)
    {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART4;
        PeriphClkInit.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();

        __HAL_RCC_UART4_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();

        /* PC10 TX, PC11 RX */
        GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
        GPIO_InitStruct.Alternate = GPIO_AF5_UART4;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(UART4_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(UART4_IRQn);
    }
    else if (uartHandle->Instance == UART5)
    {
        PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_UART5;
        PeriphClkInit.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
        if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) Error_Handler();

        __HAL_RCC_UART5_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();

        /* PC12 TX */
        GPIO_InitStruct.Pin = GPIO_PIN_12;
        GPIO_InitStruct.Alternate = GPIO_AF5_UART5;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* PD2 RX */
        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitStruct.Alternate = GPIO_AF5_UART5;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(UART5_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(UART5_IRQn);
    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef *uartHandle)
{
    if (uartHandle->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4 | GPIO_PIN_5);
        HAL_NVIC_DisableIRQ(USART1_IRQn);
    }
    else if (uartHandle->Instance == USART2)
    {
        __HAL_RCC_USART2_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3);
        HAL_NVIC_DisableIRQ(USART2_IRQn);
    }
    else if (uartHandle->Instance == USART3)
    {
        __HAL_RCC_USART3_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);
        HAL_GPIO_DeInit(GPIOE, GPIO_PIN_15);
        HAL_NVIC_DisableIRQ(USART3_IRQn);
    }
    else if (uartHandle->Instance == UART4)
    {
        __HAL_RCC_UART4_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10 | GPIO_PIN_11);
        HAL_NVIC_DisableIRQ(UART4_IRQn);
    }
    else if (uartHandle->Instance == UART5)
    {
        __HAL_RCC_UART5_CLK_DISABLE();
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);
        HAL_NVIC_DisableIRQ(UART5_IRQn);
    }
}
