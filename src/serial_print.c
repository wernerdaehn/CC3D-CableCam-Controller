#include "serial_print.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include "stdio.h"
#include "usbd_cdc_if.h"
#include "uart_callback.h"

char printbuf[80];

extern UART_HandleTypeDef huart3;

void SendString(const char * ptr, uint16_t len, Endpoints endpoint);

void PrintSerial_int(int16_t v, Endpoints endpoint)
{
    uint16_t l = snprintf(printbuf, sizeof(printbuf), " %d", v);
    SendString(printbuf, l, endpoint);
}

void PrintSerial_char(char v, Endpoints endpoint)
{
    uint16_t l = snprintf(printbuf, sizeof(printbuf), "%c", v);
    SendString(printbuf, l, endpoint);
}

void PrintSerial_string(const char * v, Endpoints endpoint)
{
    SendString(v, strlen(v), endpoint);
}

void PrintSerial_stringptr(const char * v, uint16_t len, Endpoints endpoint)
{
    SendString(v, len, endpoint);
}

void PrintSerial_long(int32_t v, Endpoints endpoint)
{
    uint16_t l = snprintf(printbuf, sizeof(printbuf), " %ld", v);
    SendString(printbuf, l, endpoint);
}

void PrintSerial_double(double v, Endpoints endpoint)
{
    if (v > 999999.0 || v < -999999.0)
    {
        SendString("????.???", 8, endpoint);
    }
    else
    {
        uint16_t l = snprintf(printbuf, sizeof(printbuf), " %1.3lf", v);
        SendString(printbuf, l, endpoint);
    }
}

void PrintSerial_float(float v, Endpoints endpoint)
{
    if (v > 999999.0f || v < -999999.0f)
    {
        SendString("????.???", 8, endpoint);
    }
    else
    {
        uint16_t l = snprintf(printbuf, sizeof(printbuf), " %1.3lf", (double) v);
        SendString(printbuf, l, endpoint);
    }
}

void PrintSerial_hexchar(char v, Endpoints endpoint)
{
    uint16_t l = snprintf(printbuf, sizeof(printbuf), " %02x", v);
    SendString(printbuf, l, endpoint);
}

void PrintlnSerial_hexstring(uint8_t v[], uint8_t len, Endpoints endpoint)
{
    for (int i=0; i<len; i++)
    {
        uint16_t l = snprintf(printbuf, sizeof(printbuf), " %02x", v[i]);
        SendString(printbuf, l, endpoint);
    }
    PrintlnSerial(endpoint);
}

void PrintlnSerial_int(int16_t v, Endpoints endpoint)
{
    uint16_t l = snprintf(printbuf, sizeof(printbuf), " %d\r\n", v);
    SendString(printbuf, l, endpoint);
}

void PrintlnSerial_char(char v, Endpoints endpoint)
{
    uint16_t l = snprintf(printbuf, sizeof(printbuf), " %c\r\n", v);
    SendString(printbuf, l, endpoint);
}

void PrintlnSerial_string(const char * v, Endpoints endpoint)
{
    SendString(v, strlen(v), endpoint);
    PrintlnSerial(endpoint);
}

void PrintlnSerial_stringptr(const char * v, uint16_t len, Endpoints endpoint)
{
    SendString(v, len, endpoint);
    PrintlnSerial(endpoint);
}

void PrintlnSerial_long(int32_t v, Endpoints endpoint)
{
    uint16_t l = snprintf(printbuf, sizeof(printbuf), " %ld\r\n", v);
    SendString(printbuf, l, endpoint);
}

void PrintlnSerial_double(double v, Endpoints endpoint)
{
    if (v > 999999.0 || v < -999999.0)
    {
        SendString("????.??? \r\n", 11, endpoint);
    }
    else
    {
        uint16_t l = snprintf(printbuf, sizeof(printbuf), " %1.3lf\r\n", v);
        SendString(printbuf, l, endpoint);
    }
}

void PrintlnSerial_float(float v, Endpoints endpoint)
{
    if (v > 999999.0f || v < -999999.0f)
    {
        SendString("????.??? \r\n", 11, endpoint);
    }
    else
    {
        uint16_t l = snprintf(printbuf, sizeof(printbuf), " %1.3lf\r\n", (double) v);
        SendString(printbuf, l, endpoint);
    }
}

void PrintlnSerial(Endpoints endpoint)
{
    SendString("\r\n", 2, endpoint);
}

void SendString(const char * ptr, uint16_t len, Endpoints endpoint)
{

    if (endpoint == EndPoint_UART3 || endpoint == EndPoint_All)
    {
        UART3Append((uint8_t*) ptr, len);
    }
    if (endpoint == EndPoint_USB || endpoint == EndPoint_All)
    {
        CDC_TransmitBuffer((uint8_t*) ptr, len);
    }
}
