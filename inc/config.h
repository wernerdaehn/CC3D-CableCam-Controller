#ifndef CONFIG_H_
#define CONFIG_H_

#include "stm32f4xx_hal.h"

#define ENCODER_VALUE TIM5->CNT
#define CONTROLLERLOOPTIME_FLOAT 0.02f
#define CONTROLLERLOOPTIME_MS 20
#define CONTROLLERLOOPTIME_FREQ 50

#define UART_CYCLIC_RXBUFFER_SIZE 512

uint16_t mod16(int16_t number, int16_t base);


#endif /* CONFIG_H_ */
