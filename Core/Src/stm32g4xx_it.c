#include "main.h"
#include "stm32g4xx_it.h"

extern FDCAN_HandleTypeDef hfdcan2;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;

void NMI_Handler(void) { while (1) { } }
void HardFault_Handler(void) { while (1) { } }
void MemManage_Handler(void) { while (1) { } }
void BusFault_Handler(void) { while (1) { } }
void UsageFault_Handler(void) { while (1) { } }
void SVC_Handler(void) { }
void DebugMon_Handler(void) { }
void PendSV_Handler(void) { }

void SysTick_Handler(void)
{
    HAL_IncTick();
}

void USART1_IRQHandler(void) { HAL_UART_IRQHandler(&huart1); }
void USART2_IRQHandler(void) { HAL_UART_IRQHandler(&huart2); }
void USART3_IRQHandler(void) { HAL_UART_IRQHandler(&huart3); }
void UART4_IRQHandler(void)  { HAL_UART_IRQHandler(&huart4); }
void UART5_IRQHandler(void)  { HAL_UART_IRQHandler(&huart5); }
void FDCAN2_IT0_IRQHandler(void) { HAL_FDCAN_IRQHandler(&hfdcan2); }
