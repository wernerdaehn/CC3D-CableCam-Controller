#ifndef CONTROLLER_H_
#define CONTROLLER_H_

#include "stm32f4xx.h"

#define APP_TX_BUF_SIZE 512

typedef enum {
	CONTROLLER_INIT = 0,  			// we just did turn on the controller, ESC signal has to be idle
	CONTROLLER_FIND_END_POS,		// controller allows to slowly travel to the end pos
	CONTROLLER_RUN 					// full operation
} CONTROLLER_STATUS_t;

typedef enum {
	OPERATIONAL = 0,  			// we can move as freely as we want, limiters enabled
	PROGRAMMING,		        // reduced speed and ready to set end points via the RC
	INVALID_RC 					// the starting point, when either no RC signal had been received at all or there is a speed != 0 at the beginning
} SAFE_MODE_t;

typedef enum {
	FREE = 0,
	EMERGENCYBRAKE,
	ENDPOINTBRAKE
} CONTROLLER_MONITOR_t;


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
