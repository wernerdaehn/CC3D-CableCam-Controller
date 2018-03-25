#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "stm32f4xx.h"

#define APP_TX_BUF_SIZE 512


void initController(void);
void controllercycle(void);

int32_t getPos(void);
float getStick(void);

float getProgrammingSwitch(void);
float getEndPointSwitch(void);
float getMaxAccelPoti(void);
float getMaxSpeedPoti(void);
float getModeSwitch(void);
float getAuxInput(void);

float getSpeedPosSensor(void);
float getSpeedPosDifference(void);

#endif
