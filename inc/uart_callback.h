#ifndef UART_CALLBACK_H_INCLUDED
#define UART_CALLBACK_H_INCLUDED

#include "stm32f4xx_hal.h"

void UART3_init(void);
uint16_t UART3_bytesunread(uint16_t lastreadpos);
uint16_t UART3_ReceiveString(void);

void UART3Append(uint8_t *ptr, uint32_t len);
void UART3Flush(void);

#endif /* UART_CALLBACK_H_INCLUDED */
