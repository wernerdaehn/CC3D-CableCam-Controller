#include "serial_print.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include "stdio.h"
#include "usbd_cdc_if.h"

char printbuf[80];

extern UART_HandleTypeDef huart3;

void SendString(const char * ptr, Endpoints endpoint);


void PrintSerial_int(int16_t v, Endpoints endpoint)
{
    snprintf(printbuf, sizeof(printbuf), " %d", v);
    SendString(printbuf, endpoint);
}

void PrintSerial_char(char v, Endpoints endpoint)
{
    snprintf(printbuf, sizeof(printbuf), "%c", v);
    SendString(printbuf, endpoint);
}

void PrintSerial_string(const char * v, Endpoints endpoint)
{
    SendString(v, endpoint);
}

void PrintSerial_long(int32_t v, Endpoints endpoint)
{
    snprintf(printbuf, sizeof(printbuf), " %ld", v);
    SendString(printbuf, endpoint);
}

void PrintSerial_double(double v, Endpoints endpoint)
{
    if (v > 999999.0 || v < -999999.0)
    {
        SendString("????.???", endpoint);
    }
    else
    {
        snprintf(printbuf, sizeof(printbuf), " %1.3lf", v);
        SendString(printbuf, endpoint);
    }
}

void PrintSerial_float(float v, Endpoints endpoint)
{
    if (v > 999999.0f || v < -999999.0f)
    {
        SendString("????.???", endpoint);
    }
    else
    {
        snprintf(printbuf, sizeof(printbuf), " %1.3lf", (double) v);
        SendString(printbuf, endpoint);
    }
}

void PrintSerial_hexchar(char v, Endpoints endpoint)
{
    snprintf(printbuf, sizeof(printbuf), " %02x", v);
    SendString(printbuf, endpoint);
}

void PrintlnSerial_hexstring(uint8_t v[], uint8_t len, Endpoints endpoint)
{
    for (int i=0; i<len; i++)
    {
        snprintf(printbuf, sizeof(printbuf), " %02x", v[i]);
        SendString(printbuf, endpoint);
    }
    PrintlnSerial(endpoint);
}

void PrintlnSerial_int(int16_t v, Endpoints endpoint)
{
    snprintf(printbuf, sizeof(printbuf), " %d\r\n", v);
    SendString(printbuf, endpoint);
}

void PrintlnSerial_char(char v, Endpoints endpoint)
{
    snprintf(printbuf, sizeof(printbuf), " %c\r\n", v);
    SendString(printbuf, endpoint);
}

void PrintlnSerial_string(const char * v, Endpoints endpoint)
{
    SendString(v, endpoint);
    PrintlnSerial(endpoint);
}

void PrintlnSerial_long(int32_t v, Endpoints endpoint)
{
    snprintf(printbuf, sizeof(printbuf), " %ld\r\n", v);
    SendString(printbuf, endpoint);
}

void PrintlnSerial_double(double v, Endpoints endpoint)
{
    if (v > 999999.0 || v < -999999.0)
    {
        SendString("????.??? \r\n", endpoint);
    }
    else
    {
        snprintf(printbuf, sizeof(printbuf), " %1.3lf\r\n", v);
        SendString(printbuf, endpoint);
    }
}

void PrintlnSerial_float(float v, Endpoints endpoint)
{
    if (v > 999999.0f || v < -999999.0f)
    {
        SendString("????.??? \r\n", endpoint);
    }
    else
    {
        snprintf(printbuf, sizeof(printbuf), " %1.3lf\r\n", (double) v);
        SendString(printbuf, endpoint);
    }
}

void PrintlnSerial(Endpoints endpoint)
{
    SendString("\r\n", endpoint);
}

void SendString(const char * ptr, Endpoints endpoint)
{
    if (endpoint == EndPoint_UART3 || endpoint == EndPoint_All)
    {
        HAL_UART_Transmit(&huart3, (uint8_t *)ptr, strlen(ptr), 1000);
    }
    if (endpoint == EndPoint_USB || endpoint == EndPoint_All)
    {
        CDC_TransmitString(ptr);
    }
}
