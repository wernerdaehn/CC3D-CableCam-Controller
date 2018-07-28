#include "stm32f4xx_hal.h"
#include "uart_callback.h"
#include "usbd_cdc_if.h"
#include "protocol.h"

char uart3_commandlinebuffer[RXBUFFERSIZE];
int16_t uart3_commandlinebuffer_pos = 0;
uint16_t uart3_bytes_scanned = 0;
uint8_t uart3_rxbuffer_overflow = 0;


extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart2;

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
            if (controllerstatus.vesc_config)
            {
                // In this mode send all received chars from bluetooth to the VESC
                HAL_UART_Transmit(&huart2, (uint8_t *) &byteReceived, 1, 10);
            }
            else
            {
                if (controllerstatus.bluetooth_passthrough)
                {
                    PrintSerial_char(byteReceived, EndPoint_USB);
                }
                huart->pRxBuffPtr[huart->RxXferCount % huart->RxXferSize] = byteReceived;
                huart->RxXferCount++;
            }
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

/** \brief USB_ReceiveString
 *         Does extract an entire line from the USB buffer using the line terminator
 *         character \r and/or \n. If there was an overflow, the line is ignored.
 *
 * This method is called periodically and tries to catchup with the received USB packets
 * by scanning from the last scanned position up to the last received character.
 *
 * bytes_scanned is the absolute position of the last byte read
 * bytes_received is the absolute position of the last received byte
 *
 * The ring buffer rxbuffer is scanned for \n chars and everything between to \n chars is
 * copied into a line buffer.
 * Hence the line buffer starts with the first char after a \n and ends
 *
 * \return uint16_t returns 1 in case a line was found
 *
 */
uint16_t UART3_ReceiveString()
{
    /*
    * Go through all characters and locate the next \n. If one is found
    * copy the entire text from the start position to the position of the next found \n
    * character into the commandlinebuffer.
    */
    while (uart3_bytes_scanned < huart3.RxXferCount)
    {
        uint8_t c = huart3.pRxBuffPtr[uart3_bytes_scanned % huart3.RxXferSize];
        uart3_bytes_scanned++;
        HAL_UART_Transmit(&huart3, (uint8_t *) &c, 1, 1000);
        if (c == '\n' || c == '\r')
        {
            if (uart3_commandlinebuffer_pos < RXBUFFERSIZE-1)   // in case the string does not fit into the commandlinebuffer, the entire line is ignored
            {
                uart3_commandlinebuffer[uart3_commandlinebuffer_pos++] = c;
                uart3_commandlinebuffer_pos = 0;
                return 1;
            }
            uart3_commandlinebuffer_pos = 0;
        }
        else if (c == 0x08) // backspace char
        {
            /*
             *                      commandlinebuffer_pos
             *                               |
             * ....... 0x65 0x66 0x67 0x68 0x08
             * User deleted char 0x68, hence instead of moving the commandlinebuffer_pos one ahead, it is
             * moved backwards by one step.
             */
            uart3_commandlinebuffer_pos--;
            if (uart3_commandlinebuffer_pos < 0)
            {
                // Obviously extra backspace chars have to be ignored
                uart3_commandlinebuffer_pos = 0;
            }
        }
        else
        {
            if (uart3_commandlinebuffer_pos < RXBUFFERSIZE)
            {
                uart3_commandlinebuffer[uart3_commandlinebuffer_pos++] = c;
            }
        }
    }
    // No newline char was found
    if (uart3_rxbuffer_overflow == 1)
    {
        uart3_bytes_scanned = huart3.RxXferCount;
        uart3_rxbuffer_overflow = 0;
    }
    return 0;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    UNUSED(huart);
}

