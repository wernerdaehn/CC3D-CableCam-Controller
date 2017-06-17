
#include "stm32f4xx_hal.h"
#include "clock_50Hz.h"
#include "config.h"
#include "sbus.h"


/*
 * Observations
 *
 * FrSky X8R
 * time between frames: 6ms.
 * time to send frame: 3ms.
*
 * Futaba R6208SB/R6303SB
 * time between frames: 11ms.
 * time to send frame: 3ms.
 */

#define SBUS_TIME_NEEDED_PER_FRAME 3500


#define SBUS_STATE_FAILSAFE (1 << 0)
#define SBUS_STATE_SIGNALLOSS (1 << 1)

#define SBUS_FRAME_BEGIN_BYTE 0x0F

#define SBUS_DIGITAL_CHANNEL_MIN 173
#define SBUS_DIGITAL_CHANNEL_MAX 1812

uint16_t calculate_duty(unsigned int sbus_val);


#define SBUS_FLAG_CHANNEL_17        (1 << 0)
#define SBUS_FLAG_CHANNEL_18        (1 << 1)
#define SBUS_FLAG_SIGNAL_LOSS       (1 << 2)
#define SBUS_FLAG_FAILSAFE_ACTIVE   (1 << 3)



void setServoValues(void);

static sbusFrame_t sbusFrame;

static sbusData_t sbusdata;

uint8_t* getSBUSFrameAddress(void) {
	return &sbusFrame.bytes[0];
}

sbusData_t* getSBUSFrame() {
	return &sbusdata;
}

int getDuty(int channel) {
	if (channel >= 0 && channel <= 17) {
		if (HAL_GetTick() - sbusdata.servovalues[channel].last_update > 3000) {
			return 0;
		} else {
			return sbusdata.servovalues[channel].duty;
		}
	} else {
		return 0;
	}
}


void setServoValues() {
	if (sbusFrame.frame.syncByte == SBUS_FRAME_BEGIN_BYTE) {
		sbusdata.sbusLastValidFrame = HAL_GetTick();
		sbusdata.servovalues[0].duty = calculate_duty(sbusFrame.frame.chan0);
		sbusdata.servovalues[0].last_update = sbusdata.sbusLastValidFrame;
		sbusdata.servovalues[1].duty = calculate_duty(sbusFrame.frame.chan1);
		sbusdata.servovalues[1].last_update = sbusdata.sbusLastValidFrame;
		sbusdata.servovalues[2].duty = calculate_duty(sbusFrame.frame.chan2);
		sbusdata.servovalues[2].last_update = sbusdata.sbusLastValidFrame;
		sbusdata.servovalues[3].duty = calculate_duty(sbusFrame.frame.chan3);
		sbusdata.servovalues[3].last_update = sbusdata.sbusLastValidFrame;
		sbusdata.servovalues[4].duty = calculate_duty(sbusFrame.frame.chan4);
		sbusdata.servovalues[4].last_update = sbusdata.sbusLastValidFrame;
		sbusdata.servovalues[5].duty = calculate_duty(sbusFrame.frame.chan5);
		sbusdata.servovalues[5].last_update = sbusdata.sbusLastValidFrame;
		sbusdata.servovalues[6].duty = calculate_duty(sbusFrame.frame.chan6);
		sbusdata.servovalues[6].last_update = sbusdata.sbusLastValidFrame;
		sbusdata.servovalues[7].duty = calculate_duty(sbusFrame.frame.chan7);
		sbusdata.servovalues[7].last_update = sbusdata.sbusLastValidFrame;
		sbusdata.servovalues[8].duty = calculate_duty(sbusFrame.frame.chan8);
		sbusdata.servovalues[8].last_update = sbusdata.sbusLastValidFrame;
		sbusdata.servovalues[9].duty = calculate_duty(sbusFrame.frame.chan9);
		sbusdata.servovalues[9].last_update = sbusdata.sbusLastValidFrame;
		sbusdata.servovalues[10].duty = calculate_duty(sbusFrame.frame.chan10);
		sbusdata.servovalues[10].last_update = sbusdata.sbusLastValidFrame;
		sbusdata.servovalues[11].duty = calculate_duty(sbusFrame.frame.chan11);
		sbusdata.servovalues[11].last_update = sbusdata.sbusLastValidFrame;
		sbusdata.servovalues[12].duty = calculate_duty(sbusFrame.frame.chan12);
		sbusdata.servovalues[12].last_update = sbusdata.sbusLastValidFrame;
		sbusdata.servovalues[13].duty = calculate_duty(sbusFrame.frame.chan13);
		sbusdata.servovalues[13].last_update = sbusdata.sbusLastValidFrame;
		sbusdata.servovalues[14].duty = calculate_duty(sbusFrame.frame.chan14);
		sbusdata.servovalues[14].last_update = sbusdata.sbusLastValidFrame;
		sbusdata.servovalues[15].duty = calculate_duty(sbusFrame.frame.chan15);
		sbusdata.servovalues[15].last_update = sbusdata.sbusLastValidFrame;
		if (sbusFrame.frame.flags & SBUS_FLAG_CHANNEL_17) {
			sbusdata.servovalues[16].duty = SERVO_MAX;
		} else {
			sbusdata.servovalues[16].duty = SERVO_MIN;
		}
		sbusdata.servovalues[16].last_update = sbusdata.sbusLastValidFrame;
		if (sbusFrame.frame.flags & SBUS_FLAG_CHANNEL_18) {
			sbusdata.servovalues[17].duty = SERVO_MAX;
		} else {
			sbusdata.servovalues[17].duty = SERVO_MIN;
		}
		sbusdata.servovalues[17].last_update = sbusdata.sbusLastValidFrame;
		
		sbusdata.signalloss = sbusFrame.frame.flags & SBUS_FLAG_SIGNAL_LOSS;
		sbusdata.failsafeactive = sbusFrame.frame.flags & SBUS_FLAG_FAILSAFE_ACTIVE;
	}
}

uint32_t getInvalidFrameCount(void) {
	return sbusdata.counter_sbus_frame_errors;
}


uint16_t calculate_duty(unsigned int sbus_val) {
	return (0.625f * sbus_val) + 880;
}

void SBUS_IRQHandler(UART_HandleTypeDef *huart)
{
  uint32_t isrflags   = READ_REG(huart->Instance->SR);
  uint32_t cr1its     = READ_REG(huart->Instance->CR1);
  uint32_t cr3its     = READ_REG(huart->Instance->CR3);
  uint32_t errorflags = 0x00U;
	
	uint8_t byteReceived = (uint8_t)(huart->Instance->DR & (uint16_t)0x00FF);

  /* If no error occurs */
  errorflags = (isrflags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
  if(errorflags == RESET)
  {
    /* UART in mode Receiver -------------------------------------------------*/
    if(((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
    {
			uint32_t now = HAL_GetTick();
			uint32_t sbusFrameTime = now - sbusdata.sbusFrameStartTime;

			if (sbusFrameTime > SBUS_TIME_NEEDED_PER_FRAME) {
				huart->pRxBuffPtr = &sbusFrame.bytes[0];
				huart->RxXferCount = SBUS_FRAME_SIZE;
				sbusdata.sbusFrameStartTime = now;
				sbusdata.counter_sbus_frame_errors++;
			}

			if (huart->RxXferCount > 0) {
				*huart->pRxBuffPtr = byteReceived;
				
				if (huart->RxXferCount == SBUS_FRAME_SIZE && *huart->pRxBuffPtr != SBUS_FRAME_BEGIN_BYTE) {
					/* It is a first byte and it is not a valid start-byte, so we are in the middle of a frame.
						 Hence do not forward the pointer but wait for a valid start byte.
					*/
				} else {
					if (huart->RxXferCount == SBUS_FRAME_SIZE) {
						sbusdata.sbusFrameStartTime = now;
					}
					huart->RxXferCount--;
					huart->pRxBuffPtr += 1U;
					if (huart->RxXferCount == 0) {
						/* Last byte of the frame */
						setServoValues();
						huart->pRxBuffPtr = &sbusFrame.bytes[0];
						huart->RxXferCount = SBUS_FRAME_SIZE;
						sbusdata.counter_sbus_frames++;
					}
				}
			} // else overflow happened
    }
  } else {
    /* UART parity error interrupt occurred ----------------------------------*/
    if(((isrflags & USART_SR_PE) != RESET) && ((cr1its & USART_CR1_PEIE) != RESET))
    {
      huart->ErrorCode |= HAL_UART_ERROR_PE;
			__HAL_UART_CLEAR_PEFLAG(huart);
			sbusdata.counter_parity_errors++;
    }
    
    /* UART noise error interrupt occurred -----------------------------------*/
    if(((isrflags & USART_SR_NE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
    {
      huart->ErrorCode |= HAL_UART_ERROR_NE;
			__HAL_UART_CLEAR_NEFLAG(huart);
			sbusdata.counter_noise_errors++;
    }
    
    /* UART frame error interrupt occurred -----------------------------------*/
    if(((isrflags & USART_SR_FE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
    {
      huart->ErrorCode |= HAL_UART_ERROR_FE;
			__HAL_UART_CLEAR_FEFLAG(huart);
			sbusdata.counter_frame_errors++;
    }
    
    /* UART Over-Run interrupt occurred --------------------------------------*/
    if(((isrflags & USART_SR_ORE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
    { 
      huart->ErrorCode |= HAL_UART_ERROR_ORE;
			__HAL_UART_CLEAR_OREFLAG(huart);
			sbusdata.counter_overrun_errors++;
    }

  } /* End if some error occurs */

}

void printSBUSChannels(Endpoints endpoint) {
	PrintSerial_string("Valid SBUS Frames: ", endpoint);
	PrintlnSerial_long(sbusdata.counter_sbus_frames, endpoint);
	PrintSerial_string("SBUS Frameerrors: ", endpoint);
	PrintlnSerial_long(sbusdata.counter_sbus_frame_errors, endpoint);
	uint32_t age = HAL_GetTick() - sbusdata.sbusLastValidFrame;
	if (age > 30000) {
		if (sbusdata.counter_sbus_frames == 0) {
			PrintlnSerial_string("Never received any SBUS data", endpoint);
		} else {
			PrintSerial_string("Data too old: ", endpoint);
			PrintlnSerial_long(age, endpoint);
		}
	} else {
		for (int i = 0; i < SBUS_MAX_CHANNEL; i++) {
			PrintSerial_string("Channel", endpoint);
			PrintSerial_int(i, endpoint);
			PrintSerial_string(": ", endpoint);
			age = HAL_GetTick() - sbusdata.servovalues[i].last_update;
			if (age > 30000) {
				PrintlnSerial_string("--outdated--", endpoint);
			} else {
				PrintlnSerial_int(sbusdata.servovalues[i].duty, endpoint);
			}
		}
	}
}

	
