
#include "main.h"
#include "sbus.h"

static int current_virtual_channel = -1;

extern sbusData_t sbusdata;


/** \brief This function is called by the Input Capture interrupt and returns the receiver servo input values
 *
 * It can handle two kinds of inputs, a SumPPM input or a regular servo signal input. The result is written into
 * the sbusdata structure, hence the actual values can have different meanings. Neutral in sbus and SumPPM is
 * around the value 1000 whereas with a standard servo input it is 1500.
 * There is no testing if SBus is active at the same time which would screw up the values.
 *
 * \param htim TIM_HandleTypeDef*
 * \return void
 *
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint32_t duty;
	uint32_t period;


	if (htim->Instance == TIM1 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {
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
		if (htim->Instance->CCER & TIM_CCER_CC3P) {		/* captured falling edge? */
            period = TIM_GetCapture1(htim->Instance);
			duty = TIM_GetCapture2(htim->Instance);	/* Get the Input Capture value, calculate high pulse length */
            // If the period is >600 (3ms), it is a new sequence
            if (periode > 600 ) {
                current_virtual_channel = 0;
            }
            // Put the value into the next field of the array
            if (current_virtual_channel < SBUS_MAX_CHANNEL) {
                sbusdata.servovalues[current_virtual_channel].duty = (uint16_t) duty;
                current_virtual_channel++;
            }



			if (duty > 4000) { // a long duty signal is the packet end of a sum ppm
				current_virtual_channel = 0;
				sbusdata.sbusLastValidFrame = HAL_GetTick();
			} else { // everything else is the duty signal
				if (current_virtual_channel != -1) {
					sbusdata.servovalues[current_virtual_channel].duty = (uint16_t) duty;
					sbusdata.counter_sbus_valid_data++;
				}
				else
                {
					sbusdata.servovalues[0].duty = (uint16_t) duty;
					sbusdata.counter_sbus_valid_data++;
                }
			}
		} else {									/* captured rising edge? */
			last_rising = input_capture_value;
			htim->Instance->CCER |= TIM_CCER_CC3P;		/* select falling edge as capture event */
			pause = last_rising - last_falling;	/* Get the Input Capture value, calculate low pulse length */
			if (pause < 550 && current_virtual_channel != -1) { // a pause of length less than 550us is a sum-ppm channel-pause
				current_virtual_channel++;
				if (current_virtual_channel >= SBUS_MAX_CHANNEL) { // failsafe, in case the sum ppm has more channels as we can cope with, then we keep writing into the last channel
					current_virtual_channel = SBUS_MAX_CHANNEL-1;
				}
			} else if (pause > 16000) { // no sum ppm
				current_virtual_channel = -1;
				sbusdata.sbusLastValidFrame = HAL_GetTick();
			}
		}
	}
}


