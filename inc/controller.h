#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "stm32f4xx.h"

#define APP_TX_BUF_SIZE 512


void initController(void);
void controllercycle(void);

void setPos(void);
float getStick(void);

float getProgrammingSwitch(void);
float getEndPointSwitch(void);
float getMaxAccelPoti(void);
float getMaxSpeedPoti(void);
float getModeSwitch(void);
float getPlaySwitch(void);
float getAuxInput(void);

#endif
