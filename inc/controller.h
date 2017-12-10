#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "stm32f4xx.h"

#define APP_TX_BUF_SIZE 512
#define ESC_STICK_SCALE 50


void setServoNeutralRange(uint16_t forward, uint16_t reverse);
void initController(void);
void controllercycle(void);

void setPIDValues(double, double, double);
void setPValue(double);
void setIValue(double);
void setDValue(double);
int32_t getTargetPos(void);
int32_t getPos(void);
int16_t getStick(void);

void resetThrottle(void);
void resetPosTarget(void);

uint16_t getProgrammingSwitch(void);
uint16_t getEndPointSwitch(void);
uint16_t getMaxAccelPoti(void);
uint16_t getMaxSpeedPoti(void);
uint16_t getModeSwitch(void);

double getSpeedPosSensor(void);
double getSpeedPosDifference(void);

#endif
