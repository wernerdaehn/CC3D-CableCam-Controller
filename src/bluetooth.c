
#include "bluetooth.h"
#include "serial_print.h"
#include "stm32f4xx_hal.h"
#include "usbd_cdc_if.h"
#include "stdbool.h"
#include "protocol.h"

extern UART_HandleTypeDef huart3;


void wait_for_bt_module_response(uint8_t *btchar_string, uint8_t * btchar_string_length);
uint8_t is_ok(uint8_t *btchar_string, uint8_t * btchar_string_length);
void UARTInit(uint32_t baud);

uint8_t configure_bt_module()
{
    uint8_t btchar_string[10];
    uint8_t btchar_string_length = 0;
    uint32_t baudrates[7] = {4800, 9600, 14400, 19200, 38400, 57600, 115200};
    uint8_t current_baud_rate_index = 0;
    uint8_t config_error = 0;

    while (current_baud_rate_index < 7)
    {

        PrintlnSerial(EndPoint_USB);
        PrintSerial_string("Testing Bluetooth module communication with ", EndPoint_USB);
        PrintSerial_long(baudrates[current_baud_rate_index], EndPoint_USB);
        PrintlnSerial_string(" baud.", EndPoint_USB);

        UARTInit(baudrates[current_baud_rate_index]);
        PrintSerial_string("AT", EndPoint_UART3);
        wait_for_bt_module_response(btchar_string, &btchar_string_length);
        if (is_ok(btchar_string, &btchar_string_length))
        {
            break;
        }
        else
        {
            current_baud_rate_index++;
        }
    }
    if (current_baud_rate_index != 7)
    {
        PrintlnSerial(EndPoint_USB);
        PrintSerial_string("Bluetooth module is currently configured for ", EndPoint_USB);
        PrintSerial_long(baudrates[current_baud_rate_index], EndPoint_USB);
        PrintlnSerial_string(" baud.", EndPoint_USB);

        if (current_baud_rate_index != 4)
        {
            PrintlnSerial_string("Sending command AT+BAUD6 to change it to 38400 baud", EndPoint_USB);

            PrintSerial_string("AT+BAUD6", EndPoint_UART3);
            wait_for_bt_module_response(btchar_string, &btchar_string_length);

            UARTInit(38400);
            if (is_ok(btchar_string, &btchar_string_length))
            {
                PrintlnSerial_string("Baud rate changed to 38400.", EndPoint_USB);
            }
            else
            {
                config_error = 1;
            }
        }
        if (config_error == 0)
        {
            PrintlnSerial(EndPoint_USB);
            PrintlnSerial_string("Setting name to CableCam using the command AT+NAMECableCam", EndPoint_USB);

            PrintSerial_string("AT+NAMECableCam", EndPoint_UART3);
            wait_for_bt_module_response(btchar_string, &btchar_string_length);
            if (is_ok(btchar_string, &btchar_string_length))
            {
                PrintlnSerial_string("Bluetooth Device Name is CableCam.", EndPoint_USB);
                PrintlnSerial(EndPoint_USB);
                PrintlnSerial_string("Setting pairing secure pin to 1234 using the command AT+PIN1234", EndPoint_USB);

                PrintSerial_string("AT+PIN1234", EndPoint_UART3);
                wait_for_bt_module_response(btchar_string, &btchar_string_length);
                if (is_ok(btchar_string, &btchar_string_length))
                {
                    PrintlnSerial_string("Bluetooth Device configured successfully", EndPoint_USB);
                    return 1;
                }
            }
        }
    }
    PrintlnSerial_string("Command not correctly processed, no OK response", EndPoint_USB);
    return 0;
}

void wait_for_bt_module_response(uint8_t *btchar_string, uint8_t * btchar_string_length)
{
    uint8_t btchar;
    uint32_t tick = HAL_GetTick();
    *btchar_string_length = 0;
    uint16_t startpointrx = huart3.RxXferCount;

    while (HAL_GetTick() - tick < 3000)
    {
        USBPeriodElapsed();
        if (huart3.RxXferCount != startpointrx)
        {
            btchar = huart3.pRxBuffPtr[startpointrx % huart3.RxXferSize];
            if (*btchar_string_length < 10)
            {
                btchar_string[*btchar_string_length] = btchar;
                (*btchar_string_length)++;
            }
            startpointrx++;
        }
    }
    PrintlnSerial(EndPoint_USB);
}

uint8_t is_ok(uint8_t *btchar_string, uint8_t * btchar_string_length)
{
    if (*btchar_string_length >= 2 && btchar_string[0] == 'O' && btchar_string[1] == 'K')
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void UARTInit(uint32_t baud)
{
    controllerstatus.bluetooth_passthrough = false;
    huart3.Init.BaudRate = baud;
    // HAL_UART_DeInit(&huart3);
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        PrintlnSerial_string("Changing the baudrate failed??? HAL_UART_Init() raised and error.", EndPoint_USB);
    }
}
