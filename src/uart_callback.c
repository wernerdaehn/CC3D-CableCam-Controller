#include "stm32f4xx_hal.h"
#include "uart_callback.h"

void uart_init(UART_HandleTypeDef *huart, uint8_t * rxbuffer, uint16_t rxbuffersize)
{
    HAL_UART_Receive_IT(huart, rxbuffer, rxbuffersize);
}

void UARTX_IRQHandler(UART_HandleTypeDef *huart)
{
    uint32_t isrflags   = READ_REG(huart->Instance->SR);
    uint32_t cr1its     = READ_REG(huart->Instance->CR1);
    uint32_t cr3its     = READ_REG(huart->Instance->CR3);
    uint32_t errorflags = 0x00U;

    uint8_t byteReceived = (uint8_t)(huart->Instance->DR & (uint16_t)0x00FF);

    /* If no error occurs */
    errorflags = (isrflags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
    if(errorflags == RESET)
    {
        /* UART in mode Receiver -------------------------------------------------*/
        if(((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
        {
            huart->pRxBuffPtr[huart->RxXferCount % huart->RxXferSize] = byteReceived;
            huart->RxXferCount++;
        }
    }
    else
    {
        /* UART parity error interrupt occurred ----------------------------------*/
        if(((isrflags & USART_SR_PE) != RESET) && ((cr1its & USART_CR1_PEIE) != RESET))
        {
            huart->ErrorCode |= HAL_UART_ERROR_PE;
            __HAL_UART_CLEAR_PEFLAG(huart);
        }

        /* UART noise error interrupt occurred -----------------------------------*/
        if(((isrflags & USART_SR_NE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
        {
            huart->ErrorCode |= HAL_UART_ERROR_NE;
            __HAL_UART_CLEAR_NEFLAG(huart);
        }

        /* UART frame error interrupt occurred -----------------------------------*/
        if(((isrflags & USART_SR_FE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
        {
            huart->ErrorCode |= HAL_UART_ERROR_FE;
            __HAL_UART_CLEAR_FEFLAG(huart);
        }

        /* UART Over-Run interrupt occurred --------------------------------------*/
        if(((isrflags & USART_SR_ORE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
        {
            huart->ErrorCode |= HAL_UART_ERROR_ORE;
            __HAL_UART_CLEAR_OREFLAG(huart);
        }

    } /* End if some error occurs */
}

uint16_t uart_bytesunread(UART_HandleTypeDef *huart, uint16_t lastreadpos)
{
    if (lastreadpos == huart->RxXferCount)
    {
        return 0;
    }
    else if (lastreadpos > huart->RxXferCount)
    {
        // Overflow of the RxXferCount int16
        return UINT16_MAX - lastreadpos + huart->RxXferCount;
    }
    else
    {
        return huart->RxXferCount - lastreadpos;
    }
}
