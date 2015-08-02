#ifndef DEBUG_H_
#define DEBUG_H_

#include "stm32f1xx.h"
#include "serial_print.h"



#define DEBUG_LEVEL_ALWAYS 		  0
#define DEBUG_LEVEL_VERYHIGH	  1
#define DEBUG_LEVEL_HIGH 		  2
#define DEBUG_LEVEL_MID 		  3
#define DEBUG_LEVEL_LOW 		  4
#define DEBUG_LEVEL_VERYLOW		  5
#define DEBUG_LEVEL_OFF 	 	  0


void setDebugLevel(uint8_t level, Endpoints endpoint);
uint8_t getDebugLevel(void);

void printDebug_int(int v, uint8_t debuglevel);
void printDebug_char(char v, uint8_t debuglevel);
void printDebug_string(char * v, uint8_t debuglevel);
void printDebug_long(long v, uint8_t debuglevel);
void printDebug_double(double v, uint8_t debuglevel);

void printlnDebug_int(int v, uint8_t debuglevel);
void printlnDebug_char(char v, uint8_t debuglevel);
void printlnDebug_string(char * v, uint8_t debuglevel);
void printlnDebug_long(long v, uint8_t debuglevel);
void printlnDebug_double(double v, uint8_t debuglevel);


#endif /* DEBUG_H_ */
