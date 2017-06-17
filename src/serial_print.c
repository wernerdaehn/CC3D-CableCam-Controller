#include "serial_print.h"
#include "stm32f4xx_hal.h"
#include "string.h"
#include "stdio.h"
#include "usbd_cdc_if.h"

char printbuf[80];

void SendString(char * ptr, Endpoints endpoint);


void PrintSerial_int(int v, Endpoints endpoint) {
	snprintf(printbuf, sizeof(printbuf), " %d ", v);
	SendString(printbuf, endpoint);
}

void PrintSerial_char(char v, Endpoints endpoint){
	snprintf(printbuf, sizeof(printbuf), "%c", v);
	SendString(printbuf, endpoint);
}
void PrintSerial_string(char * v, Endpoints endpoint){
	SendString(v, endpoint);
}
void PrintSerial_long(long v, Endpoints endpoint){
	snprintf(printbuf, sizeof(printbuf), " %ld ", v);
	SendString(printbuf, endpoint);
}
void PrintSerial_double(double v, Endpoints endpoint){
	if (v > 999999.0 || v < -999999.0) {
		SendString("????.???", endpoint);
	} else {
		snprintf(printbuf, sizeof(printbuf), " %7.3f ", v);
		SendString(printbuf, endpoint);
	}
}


void PrintlnSerial_int(int v, Endpoints endpoint) {
	snprintf(printbuf, sizeof(printbuf), " %d \r\n", v);
	SendString(printbuf, endpoint);
}

void PrintlnSerial_char(char v, Endpoints endpoint){
	snprintf(printbuf, sizeof(printbuf), " %c \r\n", v);
	SendString(printbuf, endpoint);
}
void PrintlnSerial_string(char * v, Endpoints endpoint){
	SendString(v, endpoint);
	PrintlnSerial(endpoint);
}
void PrintlnSerial_long(long v, Endpoints endpoint){
	snprintf(printbuf, sizeof(printbuf), " %ld \r\n", v);
	SendString(printbuf, endpoint);
}
void PrintlnSerial_double(double v, Endpoints endpoint){
	if (v > 999999.0 || v < -999999.0) {
		SendString("????.??? \r\n", endpoint);
	} else {
		snprintf(printbuf, sizeof(printbuf), " %7.3f \r\n", v);
		SendString(printbuf, endpoint);
	}
}
void PrintlnSerial(Endpoints endpoint){
	SendString("\r\n", endpoint);
}

void SendString(char * ptr, Endpoints endpoint) {
	if (endpoint == EndPoint_UART3 || endpoint == EndPoint_All) {
		// Uart_SendString(ptr);
	}
	if (endpoint == EndPoint_USB || endpoint == EndPoint_All) {
		CDC_TransmitString(ptr);
	}
}
