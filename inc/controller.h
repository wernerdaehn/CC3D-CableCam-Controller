#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "stm32f4xx.h"

#define APP_TX_BUF_SIZE 512

void setServoNeutralRange(uint16_t forward, uint16_t reverse);
void initController(void);
void controllercycle(void);

void setPIDValues(double, double, double);
void setPValue(double);
void setIValue(double);
void setDValue(double);
int32_t getTargetPos(void);
int32_t getPos(void);
int32_t getSpeed(void);

void resetThrottle(void);

uint16_t getProgrammingSwitch(void);
uint16_t getEndPointSwitch(void);

#endif
