
#include "stm32_servo_input.h"
#include "config.h"
#include "clock_50Hz.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_tim.h"
#include "main.h"

static int current_virtual_channel = -1;

void P_ICPWM_InitIO(void);
void P_ICPWM_InitTIM(void); 

ICPWM_Var_t ICPWM_Var[NO_CHANNELS];
static uint16_t timing[SUM_CHANNEL_START][MAX_TIMING_SAMPLES];

TIM_HandleTypeDef    Tim1HandleServoIn;



/*
 * Channel 0 = TIM1_CH1 = PA8 = Servo_4_Out
 * Channel <SUM_CHANNEL_START> = First pause of SumPPM
 * Channel <SUM_CHANNEL_START + n> = SumPPM channel n
 *
 */


uint16_t getTimingTableValue(int channel, int pos) {
	if (channel < SUM_CHANNEL_START && pos < MAX_TIMING_SAMPLES) {
		return timing[channel][pos];
	} else {
		return -1;
	}
}

void ICPWM_Init(void)
{
	uint8_t i;
	for(i=0; i<NO_CHANNELS; i++) {
		ICPWM_Var[i].duty=0;
		ICPWM_Var[i].pause=0;
	}
	P_ICPWM_InitIO();
	P_ICPWM_InitTIM();
}


uint16_t ICPWM_ReadDUTY(uint8_t channel)
{
	uint16_t ret_wert = 0;

	if (channel < NO_CHANNELS) {
		ret_wert = ICPWM_Var[channel].duty;
		// last value was written at 1400, currently it is 1460 -> value is less than 4 seconds old
		if (getCounter() - ICPWM_Var[channel].last_update < 200) {
			return ret_wert;
		} else {
			// value is older than 4 seconds -> invalid
			return 0;
		}
	} else {
		return 0;
	}
}

uint16_t ICPWM_ReadPause(uint8_t channel)
{
	uint16_t ret_wert = 0;

	if (channel < NO_CHANNELS) {
		ret_wert = ICPWM_Var[channel].pause;
		return ret_wert;
	} else {
		return 0;
	}
}

void P_ICPWM_InitIO(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	__GPIOA_CLK_ENABLE();
	__GPIOB_CLK_ENABLE();
	__TIM1_CLK_ENABLE();
	__AFIO_CLK_ENABLE();

	GPIO_InitStructure.Pull = GPIO_PULLDOWN;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Mode = GPIO_MODE_AF_INPUT;
	GPIO_InitStructure.Pin = GPIO_PIN_8;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure); // PA8 = S4out = SumPPMS input

	HAL_NVIC_SetPriority(TIM1_CC_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);

}

//--------------------------------------------------------------
// interne Funktion
// Init vom Timer
//--------------------------------------------------------------
void P_ICPWM_InitTIM(void)
{
	TIM_IC_InitTypeDef     sICConfig;

	// Timer1
	Tim1HandleServoIn.Instance = TIM1;
	Tim1HandleServoIn.Init.Period            = 0xFFFF;
	Tim1HandleServoIn.Init.Prescaler         = 72-1;
	Tim1HandleServoIn.Init.ClockDivision     = 0;
	Tim1HandleServoIn.Init.CounterMode       = TIM_COUNTERMODE_UP;
	Tim1HandleServoIn.Init.RepetitionCounter = 0;
	if(HAL_TIM_IC_Init(&Tim1HandleServoIn) != HAL_OK)
	{
		Error_Handler();
	}

	sICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
	sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
	sICConfig.ICFilter    = 0x3;

	if(HAL_TIM_IC_ConfigChannel(&Tim1HandleServoIn, &sICConfig, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	if(HAL_TIM_IC_Start_IT(&Tim1HandleServoIn, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint8_t timerindex = 99;
	uint16_t input_capture_value;
	uint16_t captureflag;
	uint16_t duty;
	uint16_t pause;

	if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
		timerindex = 0;
		input_capture_value = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		captureflag = TIM_CCER_CC1P;
	}
	if (timerindex != 99) {
		if (timing[timerindex][0] < 1 || timing[timerindex][0] >= MAX_TIMING_SAMPLES) {
			timing[timerindex][0] = 1;
		}
		timing[timerindex][ timing[timerindex][0] ] = TIM1->CCR1;
		timing[timerindex][0]++;
		/*
		 * This is an example of a Sum-PPM Signal with 6 virtual channels
		 *  ______________       ______________       _ ... _       ______________       ______________
		 * | 1: 1000+-400 |_500_| 2: 1000+-400 |_500_|  ...  |_500_| 6: 1000+-400 |_500_| Pause: 6000  |_....
		 *
		 *
		 * And this a regular single channel servo signal
		 *  ______________
		 * | 1: 1500+-700 |_______________________18500-+700_______________________....
		 */
		if (htim->Instance->CCER & captureflag) {		/* captured falling edge? */
			ICPWM_Var[timerindex].last_falling = input_capture_value;
			duty = ICPWM_Var[timerindex].last_falling - ICPWM_Var[timerindex].last_rising;	/* Get the Input Capture value, calculate high pulse length */
			htim->Instance->CCER &= ~captureflag;		/* select rising edge as capture event */
			if (duty > 4000) { // a long duty signal is the packet end of a sum ppm
				ICPWM_Var[timerindex].pause = duty; // it is a pause signal
				current_virtual_channel = SUM_CHANNEL_START;
			} else { // everything else is the duty signal
				if (current_virtual_channel != -1) {
					ICPWM_Var[current_virtual_channel].duty = duty + 500;
					ICPWM_Var[current_virtual_channel].last_update = getCounter();
				}
				ICPWM_Var[timerindex].duty = duty;
				ICPWM_Var[timerindex].last_update = getCounter();
			}
		} else {									/* captured rising edge? */
			ICPWM_Var[timerindex].last_rising = input_capture_value;
			htim->Instance->CCER |= captureflag;		/* select falling edge as capture event */
			pause = ICPWM_Var[timerindex].last_rising - ICPWM_Var[timerindex].last_falling;	/* Get the Input Capture value, calculate low pulse length */
			if (pause < 550 && current_virtual_channel != -1) { // a pause of length less than 550us is a sum-ppm channel-pause
				ICPWM_Var[current_virtual_channel].pause = pause;
				current_virtual_channel++;
				if (current_virtual_channel >= NO_CHANNELS) { // failsafe, in case the sum ppm has more channels as we can cope with, then we keep writing into the last channel
					current_virtual_channel = NO_CHANNELS-1;
				}
			} else if (pause > 16000) { // no sum ppm
				current_virtual_channel = -1;
			}
			ICPWM_Var[timerindex].pause = pause;
		}
	}
}



