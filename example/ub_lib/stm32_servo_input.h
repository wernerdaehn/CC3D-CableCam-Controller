
//--------------------------------------------------------------
#ifndef __STM32_SERVO_INPUT_H
#define __STM32_SERVO_INPUT_H


//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "stm32f1xx.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_tim.h"

#define  ICPWM_PRESCALE  71     // prescaler for 72MHz -> 1 MHz
#define MAX_TIMING_SAMPLES 40



typedef struct {
	  uint16_t duty;
	  uint16_t pause;
	  uint32_t last_update;
	  uint16_t last_falling;
	  uint16_t last_rising;
}ICPWM_Var_t;


void     ICPWM_Init(void);
uint16_t ICPWM_ReadDUTY(uint8_t channel);
uint16_t ICPWM_ReadPause(uint8_t channel);

uint16_t getTimingTableValue(int channel, int pos);


//--------------------------------------------------------------
#endif
