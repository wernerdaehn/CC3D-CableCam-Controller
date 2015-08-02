#ifndef __STM32_UART_H
#define __STM32_UART_H


//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "stm32f1xx.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_uart.h"




//--------------------------------------------------------------
// Defines fuer das Empfangen
//--------------------------------------------------------------
#define  RX_BUF_SIZE   50    // Grösse vom RX-Puffer in Bytes
#define  RX_FIRST_CHR  0x20  // erstes erlaubte Zeichen (Ascii-Wert)
#define  RX_LAST_CHR   0x7E  // letztes erlaubt Zeichen (Ascii-Wert)
#define  RX_END_CHR    0x0D  // Endekennung (Ascii-Wert)


//--------------------------------------------------------------
// Globale Funktionen
//--------------------------------------------------------------
void Uart_Init(uint32_t baudrate);
void Uart_SendByte(uint16_t wert);
void Uart_SendString(char *ptr);
void Uart_SendBuffer(uint8_t *ptr, uint16_t len);
uint16_t Uart_ReceiveString(char *ptr, uint16_t maxsize);

//--------------------------------------------------------------
#endif
