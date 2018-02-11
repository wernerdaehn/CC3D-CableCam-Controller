#ifndef UART_CALLBACK_H_INCLUDED
#define UART_CALLBACK_H_INCLUDED

#include "stm32f4xx_hal.h"

void uart_init(UART_HandleTypeDef *huart, uint8_t * rxbuffer, uint16_t rxbuffersize);
void UARTX_IRQHandler(UART_HandleTypeDef *huart);
uint16_t uart_bytesunread(UART_HandleTypeDef *huart, uint16_t lastreadpos);
uint16_t UART3_ReceiveString(void);

#endif /* UART_CALLBACK_H_INCLUDED */
