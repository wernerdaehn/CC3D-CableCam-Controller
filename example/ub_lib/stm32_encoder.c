

//--------------------------------------------------------------
// Includes
//--------------------------------------------------------------
#include "stm32_encoder.h"
#include "stm32f1xx.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#include "main.h"


TIM_HandleTypeDef    Tim2Handle;


void P_ENCODER_InitIO(void);
void P_ENCODER_InitTIM();

void ENCODER_Init()
{
	// init der Funktionen
	P_ENCODER_InitIO();
	P_ENCODER_InitTIM();
}


long ENCODER_ReadPos(void)
{
	long ret_wert=0;
	ret_wert=(long) __HAL_TIM_GetCounter(&Tim2Handle);
	if (ret_wert > 32767) { // turn uint16_t with 0...65535 into a long with -32767...+32767
		ret_wert-= 65536;
	}
	return(ret_wert);
}



void P_ENCODER_InitIO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	__GPIOA_CLK_ENABLE();
	__AFIO_CLK_ENABLE();
	GPIO_InitStructure.Pin = GPIO_PIN_0 | GPIO_PIN_1;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_INPUT;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
}


void P_ENCODER_InitTIM()
{
	TIM_Encoder_InitTypeDef     sConfig;

	Tim2Handle.Instance = TIM2;
	Tim2Handle.Init.Period            = 0xFFFF;
	Tim2Handle.Init.Prescaler         = 0;
	Tim2Handle.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
	Tim2Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	Tim2Handle.Init.RepetitionCounter = 0;

	HAL_TIM_Encoder_DeInit(&Tim2Handle);

	__TIM2_CLK_ENABLE();
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 1; // Not sure what filter value to use. Seems there is some eddy current in the aluminum
	sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 1;

	if(HAL_TIM_Encoder_Init(&Tim2Handle, &sConfig) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}

	__HAL_TIM_SetCounter(&Tim2Handle, 0);

	if(HAL_TIM_Encoder_Start(&Tim2Handle, TIM_CHANNEL_1 | TIM_CHANNEL_2) != HAL_OK)
	{
		/* Configuration Error */
		Error_Handler();
	}


}

