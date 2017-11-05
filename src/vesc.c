#include "vesc.h"
#include "stdio.h"
#include "string.h"
#include "serial_print.h"

uint8_t packet[20];
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;

void VESC_Output(uint16_t esc_output)
{
    int16_t ret = snprintf((char *) packet, sizeof(packet), " %i \r\n", esc_output);
    if (ret > 0)
    {
        HAL_UART_Transmit(&huart2, packet, ret, 10000);
        // PrintlnSerial_int(esc_output, EndPoint_USB);
    }
    ret = snprintf((char *) packet, sizeof(packet), "ABC");
    if (ret > 0)
    {
        HAL_UART_Transmit(&huart6, packet, ret, 10000);
        // PrintlnSerial_int(esc_output, EndPoint_USB);
    }

}


