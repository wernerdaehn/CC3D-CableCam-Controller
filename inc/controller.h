#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "stm32f4xx.h"

#define APP_TX_BUF_SIZE 512

typedef enum {
	CONTROLLER_INIT = 0,  			// we just did turn on the controller, ESC signal has to be idle
	CONTROLLER_FIND_END_POS,		// controller allows to slowly travel to the end pos
	CONTROLLER_RUN 					// full operation
} CONTROLLER_STATUS_t;

extern int16_t servo[];
extern int16_t rcData[];

void setServoNeutralRange(uint16_t forward, uint16_t reverse);
void initController(void);
void controllercycle(void);
void setRCData(uint16_t rcsignal1, uint16_t rcsignal2, uint16_t rcsignal3, uint16_t rcsignal4);
uint16_t getRCDataForChannel(uint8_t channel);
void setServoValueForChannel(uint8_t channel, uint16_t rcsignal);

void setPIDValues(double, double, double);
void setPValue(double);
void setIValue(double);
void setDValue(double);
void getPIDValues(double *kp, double *ki, double *kd);
double getPValue(void);
double getIValue(void);
double getDValue(void);
void setMaxAcceleration(double maxaccel);
double getMaxAcceleration(void);
void setMaxAccelerationRCStick(double maxaccel);
double getMaxAccelerationRCStick(void);
void setToleratedPositionalErrorLimit(double distance);
double getToleratedPositionalErrorLimit(void);
void setMaxAllowedPositionalError(double distance);
double getMaxAllowedPositionalError(void);
void setPosStart(double p);
double getPosStart(void);
void setPosEnd(double p);
double getPosEnd(void);
void setMaxSpeed(double p);
double getMaxSpeed(void);
void setTargetPos(double p);
double getTargetPos(void);
void setPitch(double p);
double getPitch(void);
void setYaw(double p);
double getYaw(void);
void resetThrottle(void);
double getThrottle(void);
double getSpeed(void);
double getPos(void);

#endif
