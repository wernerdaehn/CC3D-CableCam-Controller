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


/*
 * This is the Play state machine. Valid changes are:
 * OFF -> ON
 * ON -> OFF, TEMPORARY_OFF, FORCE_OFF
 * TEMPORARY_OFF -> OFF, ON
 * FORCE_OFF -> OFF
 */
typedef enum {
	OFF = 0,  			// State machine shows that Play has not bee requested
	ON,		            // State machine shows that Play is on
	TEMPORARY_OFF,		// The current state asks Play to be turned Off, condition might change
	FORCE_OFF           // A condition requires Play to set Off, need to switch Play Off to continue
} PLAY_RUNNING_t;

#endif
