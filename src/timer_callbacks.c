
#include "stm32f4xx_hal.h"
#include "clock_50Hz.h"
#include "config.h"
#include "sbus.h"
#include "protocol.h"

static int8_t current_virtual_channel = 0;
static uint16_t lastrising = 0;

extern uint32_t possensorduration;
uint32_t possensortick_old = 0;
extern uint32_t last_possensortick;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    /*
     * This is an example of a Sum-PPM Signal with 6 virtual channels
     *  ______________       ______________       _ ... _       ______________       _______________________
     * | 1: 1000+-400 |_500_| 2: 1000+-400 |_500_|  ...  |_500_| 6: 1000+-400 |_500_|       End: 6000       |_....
     *
     *
     * And this a regular single channel servo signal
     *  ______________
     * | 1: 1500+-700 |_______________________18500-+700_______________________....
     *
     * Special care has to be taken with incorrect signals, e.g. when the receiver was turned off and creates the first output.
     * In this case there could be a long pause looking like a servo signal when in reality it is just the start of a SumPPM signal
     * and the such.
     *
     * Unfortunately the logic requires to know the pause length after the signal to differentiate between a SumPPM and regular
     * servo input. Only if the duty is followed by a long pause, this value is to be written into servodata[0]. Hence there is
     * a small delay of up to 20ms in the servo mode, 5ms in SumPPM input.
     * For that reason the TIM1_CH3 fires on the rising flank and an interrupt on the slave's falling edge is not triggered.
     */

    if (htim->Instance == TIM1)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
        {
            /*
             * The Channel3 Interrupt fires at the falling edge, thus we know the duty
             * l______________4               3___
             * |              |_______________|   ......
             */

            // It is important to use a uint16 here as the timer returns a 16 value, although it is defined as 32 bit uint
            uint16_t duty = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4) - lastrising;
            setNextDuty(duty);

            if (duty > 4000)   // a long duty signal is the packet-end of a sum ppm
            {
                current_virtual_channel = 0; // Hence the next duty signal will be for the first channel
                sbusdata.sbusLastValidFrame = HAL_GetTick(); // and we got a valid frame, so set the timestamp to now
                sbusdata.counter_sbus_valid_data++;
                sbusdata.receivertype = RECEIVER_TYPE_SUMPPM;
            }
            else     // everything else is the duty signal
            {
                if (current_virtual_channel < SBUS_MAX_CHANNEL) // What if the SumPPM sends more than 16 values? Ignore those.
                {
                    sbusdata.servovalues[current_virtual_channel].duty = duty; // any other duty signal sets the input value
                }
            }

            /*
             *  ______________4               3______________
             * |              |_______________|              |___________
             */
            uint16_t pause = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3) - HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
            setNextDuty(pause);
            if (pause < 700)   // a pause of length less than 700us is a sum-ppm channel-pause
            {
                current_virtual_channel++;
            }
            else if (pause > 10000)     // no sum ppm
            {
                current_virtual_channel = 0;
                sbusdata.sbusLastValidFrame = HAL_GetTick();
                sbusdata.counter_sbus_valid_data++;
                if (sbusdata.receivertype != RECEIVER_TYPE_SERVO)
                {
                    /*
                     * Set all values to zero except for the first channel. This makes sure changes in the receiver type setting
                     * or the receiver itself do not leave a value over. Obviously that has to be done only if the receiver type changed.
                     */
                    for (int i=1; i<SBUS_MAX_CHANNEL; i++)
                    {
                        sbusdata.servovalues[i].duty = 0;
                    }
                }
                sbusdata.receivertype = RECEIVER_TYPE_SERVO;
            }

            lastrising = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
        }
    }
    else if (htim->Instance == TIM5)
    {
        last_possensortick = HAL_GetTick();
        possensorduration = last_possensortick - possensortick_old;
        possensortick_old = last_possensortick;
    }
}
