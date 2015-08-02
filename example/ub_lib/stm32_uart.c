
#include "stm32_uart.h"
#include "stdio.h"
#include "math.h"
#include "errno.h"
#include "main.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#include "string.h"

UART_HandleTypeDef Uart3Handle;

/* Buffer used for transmission */
uint8_t aTxBuffer[80];
uint8_t aRxBuffer[80];
uint8_t aLastLine[80];

void Uart_Init(uint32_t baudrate)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	__GPIOB_CLK_ENABLE();
	__HAL_RCC_AFIO_CLK_ENABLE();
	__USART3_CLK_ENABLE();

	//pb10
	GPIO_InitStructure.Pin = GPIO_PIN_10;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	// PB11
	GPIO_InitStructure.Pin = GPIO_PIN_11;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);

	HAL_NVIC_SetPriority(USART3_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(USART3_IRQn);

	Uart3Handle.Instance        = USART3;
	Uart3Handle.Init.BaudRate   = baudrate;
	Uart3Handle.Init.WordLength = UART_WORDLENGTH_8B;
	Uart3Handle.Init.StopBits   = UART_STOPBITS_1;
	Uart3Handle.Init.Parity     = UART_PARITY_NONE;
	Uart3Handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	Uart3Handle.Init.Mode       = UART_MODE_TX_RX;

	if(HAL_UART_DeInit(&Uart3Handle) != HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_UART_Init(&Uart3Handle) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_UART_String_Listener(&Uart3Handle, aRxBuffer, sizeof(aRxBuffer)) != HAL_OK)
	{
		Error_Handler();
	}

}

//--------------------------------------------------------------
// ein Byte per UART senden
//--------------------------------------------------------------
void Uart_SendByte(uint16_t wert)
{
	aTxBuffer[0] = (uint8_t) wert;
	HAL_UART_Transmit(&Uart3Handle, (uint8_t*)aTxBuffer, 1, 1000);
}

void Uart_SendString(char *ptr)
{
	HAL_UART_Transmit(&Uart3Handle, (uint8_t*)ptr, strlen(ptr), 1000);
}

void Uart_SendBuffer(uint8_t *ptr, uint16_t len) {
	HAL_UART_Transmit(&Uart3Handle, ptr, len, 1000);
}


uint16_t Uart_ReceiveString(char *ptr, uint16_t maxsize)
{
	return HAL_UART_String_Getter(&Uart3Handle, (uint8_t *) ptr, maxsize);
}






/**
 * @brief  Tx Transfer completed callback
 * @param  UartHandle: UART handle.
 * @note   This example shows a simple way to report end of IT Tx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
}

/**
 * @brief  Rx Transfer completed callback
 * @param  UartHandle: UART handle
 * @note   This example shows a simple way to report end of DMA Rx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
}

/**
 * @brief  UART error callbacks
 * @param  UartHandle: UART handle
 * @note   This example shows a simple way to report transfer error, and you can
 *         add your own implementation.
 * @retval None
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
}


