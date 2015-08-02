/*
 * TIM4 Ch3 on PB7
 */
#ifndef __STM32_SERVO_OUTPUT_H
#define __STM32_SERVO_OUTPUT_H


#include "stm32f1xx.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_tim.h"


#define  PWM_PERIOD   19999 // periode
#define  PWM_PRESCALE   71 // prescaler ( => 50Hz)

#define SERVO_1_OUT	1
#define SERVO_2_OUT	2
#define SERVO_3_OUT	3
#define SERVO_5_OUT 4

#define  PWM_POLARITY  TIM_OCPolarity_High


void PWM_Init(void);
void PWM_SetPWM(uint8_t servo, uint16_t wert);



#endif
