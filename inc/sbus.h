#ifndef SBUS_H_
#define SBUS_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "serial_print.h"

#define SBUS_MAX_CHANNEL 16
#define SBUS_FRAME_SIZE 25

struct sbusFrame_s {
    uint8_t syncByte;
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
    uint8_t flags;
    uint8_t endByte;
} __attribute__ ((__packed__));

typedef union {
    uint8_t bytes[SBUS_FRAME_SIZE];
    struct sbusFrame_s frame;
} sbusFrame_t;


typedef struct {
	  uint16_t duty;
} servo_t;

typedef struct {
	uint32_t sbusFrameStartTime;
	uint32_t sbusLastValidFrame;
	uint32_t counter_sbus_frames;
	uint32_t counter_sbus_errors;
	uint32_t counter_sbus_valid_data;
	uint32_t counter_sbus_frame_errors;
	uint32_t counter_parity_errors;
	uint32_t counter_noise_errors;
	uint32_t counter_frame_errors;
	uint32_t counter_overrun_errors;
	servo_t servovalues[SBUS_MAX_CHANNEL];
	uint8_t channel16;
	uint8_t channel17;
	uint8_t signalloss;
	uint8_t failsafeactive;
	uint8_t receivertype;
}sbusData_t;

extern sbusData_t sbusdata;


void SBUS_IRQHandler(UART_HandleTypeDef *huart);
void printSBUSChannels(Endpoints endpoint);
int16_t getDuty(uint8_t channel);
uint8_t* getSBUSFrameAddress(void);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void initSBusData(uint8_t receivertype);

#endif
