/*
 * Encoder is using TIM4 Ch3&4 one PB8 and PB9
 */
#ifndef __STM32_ENCODER_H
#define __STM32_ENCODER_H


#include "stm32f1xx.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_tim.h"

#define   ENC_FILTER              6


void ENCODER_Init();
long ENCODER_ReadPos(void);




//--------------------------------------------------------------
#endif
