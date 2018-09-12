#include "stm32f4xx_hal.h"
#include "uart_callback.h"
#include "usbd_cdc_if.h"
#include "protocol.h"
#include "stm32f4xx_it.h"

uint8_t uart3_rxbuffer[RXBUFFERSIZE];
char uart3_commandlinebuffer[RXBUFFERSIZE];
int16_t uart3_commandlinebuffer_pos = 0;
uint16_t uart3_bytes_scanned = 0;
uint16_t uart3_received_pos = 0;

uint32_t uart3_bytes_sent = 0;
uint32_t uart3_bytes_written = 0;
uint8_t uart3TxBuffer[APP_TX_DATA_SIZE];

extern UART_HandleTypeDef huart3;

void UART3_init()
{
    if(HAL_UART_Receive_DMA(&huart3, uart3_rxbuffer, RXBUFFERSIZE) != HAL_OK)
    {
        Error_Handler();
    }
}

uint16_t UART3_bytesunread(uint16_t lastreadpos)
{
    if (lastreadpos == huart3.RxXferCount)
    {
        return 0;
    }
    else if (lastreadpos > huart3.RxXferCount)
    {
        // Overflow of the RxXferCount int16
        return UINT16_MAX - lastreadpos + huart3.RxXferCount;
    }
    else
    {
        return huart3.RxXferCount - lastreadpos;
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
    uint16_t currentreceivepos = huart3.RxXferSize - __HAL_DMA_GET_COUNTER(huart3.hdmarx);
    if (currentreceivepos != uart3_received_pos)
    {
        huart3.RxXferCount += currentreceivepos - uart3_received_pos;
        uart3_received_pos = currentreceivepos;


        /*
        * Go through all characters and locate the next \n. If one is found
        * copy the entire text from the start position to the position of the next found \n
        * character into the commandlinebuffer.
        */
        while (uart3_bytes_scanned < huart3.RxXferCount)
        {
            uint8_t c = huart3.pRxBuffPtr[uart3_bytes_scanned % huart3.RxXferSize];
            uart3_bytes_scanned++;
            PrintSerial_char(c, EndPoint_UART3);
            if (uart3_commandlinebuffer_pos < RXBUFFERSIZE-1)
            {
                if (c == '\n' || c == '\r')
                {
                    uart3_commandlinebuffer[uart3_commandlinebuffer_pos++] = c;
                    uart3_commandlinebuffer[uart3_commandlinebuffer_pos++] = 0;
                    uart3_commandlinebuffer_pos = 0;
                    return 1;
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
                    uart3_commandlinebuffer[uart3_commandlinebuffer_pos++] = c;
                }
            }
            else
            {
                uart3_commandlinebuffer_pos = 0; // in case the string does not fit into the commandlinebuffer, the entire line is ignored
            }
        }
    }
    return 0;
}

void UART3Append(uint8_t *ptr, uint32_t len)
{
    uint32_t rel_pos = uart3_bytes_written % APP_TX_DATA_SIZE;
    if (rel_pos + len > APP_TX_DATA_SIZE)
    {
        uint32_t l = APP_TX_DATA_SIZE - rel_pos;
        memcpy(&uart3TxBuffer[rel_pos], ptr, l);
        memcpy(uart3TxBuffer, &ptr[l], len - l);
    }
    else
    {
        memcpy(&uart3TxBuffer[rel_pos], ptr, len);
    }
    uart3_bytes_written += len;
}

void UART3Flush()
{
    if(uart3_bytes_written != uart3_bytes_sent)
    {
        if ((huart3.gState & 0x01) == 0x00)
        {
            uint32_t buffptr = uart3_bytes_sent % APP_TX_DATA_SIZE;
            uint32_t buffsize = uart3_bytes_written - uart3_bytes_sent;
            if (buffptr + buffsize > APP_TX_DATA_SIZE)
            {
                buffsize = APP_TX_DATA_SIZE - buffptr;
            }
            HAL_UART_Transmit_DMA(&huart3, &uart3TxBuffer[buffptr], buffsize);
            uart3_bytes_sent += buffsize;
        }
    }
}



