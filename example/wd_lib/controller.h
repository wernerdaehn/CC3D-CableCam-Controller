#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "stm32f1xx.h"

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
double getPValue();
double getIValue();
double getDValue();
void setMaxAcceleration(double maxaccel);
double getMaxAcceleration();
void setMaxAccelerationRCStick(double maxaccel);
double getMaxAccelerationRCStick();
void setToleratedPositionalErrorLimit(double distance);
double getToleratedPositionalErrorLimit();
void setMaxAllowedPositionalError(double distance);
double getMaxAllowedPositionalError();
void setPosStart(double p);
double getPosStart();
void setPosEnd(double p);
double getPosEnd();
void setMaxSpeed(double p);
double getMaxSpeed();
void setTargetPos(double p);
double getTargetPos();
void setPitch(double p);
double getPitch();
void setYaw(double p);
double getYaw();
void resetThrottle();
double getThrottle();
double getSpeed();
double getPos();

#endif
