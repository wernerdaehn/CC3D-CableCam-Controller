#ifndef SERIAL_PRINT_H_
#define SERIAL_PRINT_H_

#include "main.h"


typedef enum
{
  EndPoint_UART3                   = 0x00,
  EndPoint_USB,
  EndPoint_All,
} Endpoints;


void PrintSerial_int(int16_t v, Endpoints endpoint);
void PrintSerial_char(char v, Endpoints endpoint);
void PrintSerial_string(char * v, Endpoints endpoint);
void PrintSerial_long(int32_t v, Endpoints endpoint);
void PrintSerial_double(double v, Endpoints endpoint);
void PrintlnSerial_int(int16_t v, Endpoints endpoint);
void PrintlnSerial_char(char v, Endpoints endpoint);
void PrintlnSerial_string(char * v, Endpoints endpoint);
void PrintlnSerial_long(int32_t v, Endpoints endpoint);
void PrintlnSerial_double(double v, Endpoints endpoint);
void PrintlnSerial(Endpoints endpoint);
#endif
