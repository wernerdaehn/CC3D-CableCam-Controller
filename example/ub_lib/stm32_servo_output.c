
#include "stm32f1xx.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#include "stm32_servo_output.h"
#include "main.h"

TIM_HandleTypeDef    Tim4Handle;
TIM_HandleTypeDef    Tim3Handle;


void P_PWM_InitIO(void);
void P_PWM_InitTIM(void);

/*
 * SERVO_1_OUT = PB9 = TIM4_CH4
 * SERVO_2_OUT = PB8 = TIM4_CH3
 * SERVO_3_OUT = PB7 = TIM4_CH2
 * SERVO_5_OUT = PB4 = TIM3_CH1
 */


void PWM_Init(void)
{
	P_PWM_InitIO();
	P_PWM_InitTIM();
}


void PWM_SetPWM(uint8_t servo, uint16_t pwmvalue)
{
	if(pwmvalue>PWM_PERIOD) pwmvalue=PWM_PERIOD;

	if (servo == SERVO_1_OUT) {
		TIM4->CCR4 = pwmvalue;
	} else if (servo == SERVO_2_OUT) {
		TIM4->CCR3 = pwmvalue;
	} else if (servo == SERVO_3_OUT) {
		TIM4->CCR2 = pwmvalue;
	} else if (servo == SERVO_5_OUT) {
		TIM3->CCR1 = pwmvalue;
	}
}

void P_PWM_InitIO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	__GPIOB_CLK_ENABLE();
	GPIO_InitStructure.Pin = GPIO_PIN_4 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
	__HAL_AFIO_REMAP_TIM3_PARTIAL();

}


void P_PWM_InitTIM(void)
{
	TIM_OC_InitTypeDef sConfig;

	__TIM4_CLK_ENABLE();
	__TIM3_CLK_ENABLE();

	Tim4Handle.Instance = TIM4;
	Tim4Handle.Init.Prescaler         = PWM_PRESCALE;
	Tim4Handle.Init.Period            = PWM_PERIOD;
	Tim4Handle.Init.ClockDivision     = 0;
	Tim4Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	Tim4Handle.Init.RepetitionCounter = 0;
	if (HAL_TIM_PWM_Init(&Tim4Handle) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}


	sConfig.OCMode       = TIM_OCMODE_PWM1;
	sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
	sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
	sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
	sConfig.Pulse = 1500;




	if (HAL_TIM_PWM_ConfigChannel(&Tim4Handle, &sConfig, TIM_CHANNEL_2) != HAL_OK)
	{
		/* Configuration Error */
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&Tim4Handle, &sConfig, TIM_CHANNEL_3) != HAL_OK)
	{
		/* Configuration Error */
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&Tim4Handle, &sConfig, TIM_CHANNEL_4) != HAL_OK)
	{
		/* Configuration Error */
		Error_Handler();
	}

	/* Start channel 2 */
	if (HAL_TIM_PWM_Start(&Tim4Handle, TIM_CHANNEL_2) != HAL_OK)
	{
		/* PWM Generation Error */
		Error_Handler();
	}
	/* Start channel 3 */
	if (HAL_TIM_PWM_Start(&Tim4Handle, TIM_CHANNEL_3) != HAL_OK)
	{
		/* PWM generation Error */
		Error_Handler();
	}
	/* Start channel 4 */
	if (HAL_TIM_PWM_Start(&Tim4Handle, TIM_CHANNEL_4) != HAL_OK)
	{
		/* PWM generation Error */
		Error_Handler();
	}

	Tim3Handle.Instance = TIM3;
	Tim3Handle.Init.Prescaler         = PWM_PRESCALE;
	Tim3Handle.Init.Period            = PWM_PERIOD;
	Tim3Handle.Init.ClockDivision     = 0;
	Tim3Handle.Init.CounterMode       = TIM_COUNTERMODE_UP;
	Tim3Handle.Init.RepetitionCounter = 0;
	if (HAL_TIM_PWM_Init(&Tim3Handle) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler();
	}

	if (HAL_TIM_PWM_ConfigChannel(&Tim3Handle, &sConfig, TIM_CHANNEL_1) != HAL_OK)
	{
		/* Configuration Error */
		Error_Handler();
	}

	/* Start channel 2 */
	if (HAL_TIM_PWM_Start(&Tim3Handle, TIM_CHANNEL_1) != HAL_OK)
	{
		/* PWM Generation Error */
		Error_Handler();
	}

}
