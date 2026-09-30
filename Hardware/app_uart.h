#ifndef __APP_UART_H
#define __APP_UART_H

#include "hwt101_uart.h"

void AppUart_Init(void);
Hwt101 *AppUart_GetImu(void);

#endif
