
#include "main.h"
#include "stm32_uart.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32f1xx_hal_uart.h"
#include "clock_50Hz.h"
#include "config.h"
#include "stm32_servo_input.h"


UART_HandleTypeDef Uart1Handle;
extern ICPWM_Var_t ICPWM_Var[NO_CHANNELS];

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

#define SBUS_TIME_NEEDED_PER_FRAME 3000


#define SBUS_STATE_FAILSAFE (1 << 0)
#define SBUS_STATE_SIGNALLOSS (1 << 1)

#define SBUS_MAX_CHANNEL 18
#define SBUS_FRAME_SIZE 25

#define SBUS_FRAME_BEGIN_BYTE 0x0F

#define SBUS_DIGITAL_CHANNEL_MIN 173
#define SBUS_DIGITAL_CHANNEL_MAX 1812

static uint8_t sbusFrameDone = 0;
static void sbusReceive(void *huart, uint8_t *pBufferChar, uint32_t character);
uint16_t calculate_duty(unsigned int sbus_val);

uint8_t charbuffer[SBUS_FRAME_SIZE*2];

void sbusInit()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	__GPIOA_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
	__HAL_RCC_AFIO_CLK_ENABLE();
	__USART1_CLK_ENABLE();

	//PA9 TX
	GPIO_InitStructure.Pin = GPIO_PIN_9;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	// HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); We don't need TX at the moment

	// PB2 Inverter turned high
	GPIO_InitStructure.Pin = GPIO_PIN_2;
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);

	// PA10 RX
	GPIO_InitStructure.Pin = GPIO_PIN_10;
	GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

	HAL_NVIC_SetPriority(USART1_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(USART1_IRQn);

	// One Byte = 1 startbit + 8 databit + 1 paritybit + 2 stopbit (8E2), baudrate = 100'000 bit/s
	Uart1Handle.Instance        = USART1;
	Uart1Handle.Init.BaudRate   = 100000;
	Uart1Handle.Init.WordLength = UART_WORDLENGTH_9B;
	Uart1Handle.Init.StopBits   = UART_STOPBITS_2;
	Uart1Handle.Init.Parity     = UART_PARITY_EVEN;
	Uart1Handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	Uart1Handle.Init.Mode       = UART_MODE_RX;

	if(HAL_UART_DeInit(&Uart1Handle) != HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_UART_Init(&Uart1Handle) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_UART_Char_Listener(&Uart1Handle, charbuffer, SBUS_FRAME_SIZE*2, &sbusReceive) != HAL_OK)
	{
		Error_Handler();
	}


}

#define SBUS_FLAG_CHANNEL_17        (1 << 0)
#define SBUS_FLAG_CHANNEL_18        (1 << 1)
#define SBUS_FLAG_SIGNAL_LOSS       (1 << 2)
#define SBUS_FLAG_FAILSAFE_ACTIVE   (1 << 3)

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

static sbusFrame_t sbusFrame;

static void sbusReceive(void *huart, uint8_t *pBufferChar, uint32_t character)
{
    static uint8_t sbusFramePosition = 0;
    static uint32_t sbusFrameStartAt = 0;
    uint32_t now = getCounter(); // 20ms increment
    uint8_t i;

    int32_t sbusFrameTime = (now - sbusFrameStartAt) * 20;

    if (sbusFrameTime > (long)(SBUS_TIME_NEEDED_PER_FRAME + 500)) {
        sbusFramePosition = 0;
    }

    sbusFrame.bytes[sbusFramePosition] = (uint8_t)character;

    if (sbusFramePosition == 0) {
        if (sbusFrame.bytes[sbusFramePosition] != SBUS_FRAME_BEGIN_BYTE) {
            return;
        }
        sbusFrameStartAt = now;
    }

    sbusFramePosition++;

    if (sbusFramePosition == SBUS_FRAME_SIZE) {
        // endByte currently ignored
        sbusFrameDone = 1;
        sbusFramePosition = 0;
        for (i=0; i<NO_CHANNELS-SUM_CHANNEL_START; i++) {
        	switch (i) {
        	case 0:
        		ICPWM_Var[SUM_CHANNEL_START+i].duty = calculate_duty(sbusFrame.frame.chan0);
                ICPWM_Var[SUM_CHANNEL_START+i].last_update = now;
        		break;
        	case 1:
        		ICPWM_Var[SUM_CHANNEL_START+i].duty = calculate_duty(sbusFrame.frame.chan1);
                ICPWM_Var[SUM_CHANNEL_START+i].last_update = now;
        		break;
        	case 2:
        		ICPWM_Var[SUM_CHANNEL_START+i].duty = calculate_duty(sbusFrame.frame.chan2);
                ICPWM_Var[SUM_CHANNEL_START+i].last_update = now;
        		break;
        	case 3:
        		ICPWM_Var[SUM_CHANNEL_START+i].duty = calculate_duty(sbusFrame.frame.chan3);
                ICPWM_Var[SUM_CHANNEL_START+i].last_update = now;
        		break;
        	case 4:
        		ICPWM_Var[SUM_CHANNEL_START+i].duty = calculate_duty(sbusFrame.frame.chan4);
                ICPWM_Var[SUM_CHANNEL_START+i].last_update = now;
        		break;
        	case 5:
        		ICPWM_Var[SUM_CHANNEL_START+i].duty = calculate_duty(sbusFrame.frame.chan5);
                ICPWM_Var[SUM_CHANNEL_START+i].last_update = now;
        		break;
        	case 6:
        		ICPWM_Var[SUM_CHANNEL_START+i].duty = calculate_duty(sbusFrame.frame.chan6);
                ICPWM_Var[SUM_CHANNEL_START+i].last_update = now;
        		break;
        	case 7:
        		ICPWM_Var[SUM_CHANNEL_START+i].duty = calculate_duty(sbusFrame.frame.chan7);
                ICPWM_Var[SUM_CHANNEL_START+i].last_update = now;
        		break;
        	case 8:
        		ICPWM_Var[SUM_CHANNEL_START+i].duty = calculate_duty(sbusFrame.frame.chan8);
                ICPWM_Var[SUM_CHANNEL_START+i].last_update = now;
        		break;
        	case 9:
        		ICPWM_Var[SUM_CHANNEL_START+i].duty = calculate_duty(sbusFrame.frame.chan9);
                ICPWM_Var[SUM_CHANNEL_START+i].last_update = now;
        		break;
        	case 10:
        		ICPWM_Var[SUM_CHANNEL_START+i].duty = calculate_duty(sbusFrame.frame.chan10);
                ICPWM_Var[SUM_CHANNEL_START+i].last_update = now;
        		break;
        	case 11:
        		ICPWM_Var[SUM_CHANNEL_START+i].duty = calculate_duty(sbusFrame.frame.chan11);
                ICPWM_Var[SUM_CHANNEL_START+i].last_update = now;
        		break;
        	case 12:
        		ICPWM_Var[SUM_CHANNEL_START+i].duty = calculate_duty(sbusFrame.frame.chan12);
                ICPWM_Var[SUM_CHANNEL_START+i].last_update = now;
        		break;
        	case 13:
        		ICPWM_Var[SUM_CHANNEL_START+i].duty = calculate_duty(sbusFrame.frame.chan13);
                ICPWM_Var[SUM_CHANNEL_START+i].last_update = now;
        		break;
        	case 14:
        		ICPWM_Var[SUM_CHANNEL_START+i].duty = calculate_duty(sbusFrame.frame.chan14);
                ICPWM_Var[SUM_CHANNEL_START+i].last_update = now;
        		break;
        	case 15:
        		ICPWM_Var[SUM_CHANNEL_START+i].duty = calculate_duty(sbusFrame.frame.chan15);
                ICPWM_Var[SUM_CHANNEL_START+i].last_update = now;
        		break;
        	case 16:
        	    if (sbusFrame.frame.flags & SBUS_FLAG_CHANNEL_17) {
        	    	ICPWM_Var[SUM_CHANNEL_START+i].duty = SERVO_MAX;
        	    } else {
        	    	ICPWM_Var[SUM_CHANNEL_START+i].duty = SERVO_MIN;
        	    }
                ICPWM_Var[SUM_CHANNEL_START+i].last_update = now;
        		break;
        	case 17:
        	    if (sbusFrame.frame.flags & SBUS_FLAG_CHANNEL_18) {
        	    	ICPWM_Var[SUM_CHANNEL_START+i].duty = SERVO_MAX;
        	    } else {
        	    	ICPWM_Var[SUM_CHANNEL_START+i].duty = SERVO_MIN;
        	    }
                ICPWM_Var[SUM_CHANNEL_START+i].last_update = now;
        		break;
        	default:
        		break;
        	}
        }
        if (sbusFrame.frame.flags & SBUS_FLAG_SIGNAL_LOSS) {

        }
        if (sbusFrame.frame.flags & SBUS_FLAG_FAILSAFE_ACTIVE) {

        }
    } else {
        sbusFrameDone = 0;
    }
}

uint16_t calculate_duty(unsigned int sbus_val) {
	return (0.625f * sbus_val) + 880;
}



