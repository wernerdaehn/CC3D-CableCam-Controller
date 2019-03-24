
#include "stm32f4xx_hal.h"
#include "clock_50Hz.h"
#include "config.h"
#include "sbus.h"
#include "protocol.h"

static int8_t current_virtual_channel = 0;

uint32_t possensortick_old = 0;
uint16_t servovalues[SBUS_MAX_CHANNEL];

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

    /*
     * 404   1107   404   1107   404
     * 7460   405   1107   404   1107   404   1107   405   1107   404   1107   405   1675   405   1107   404   1107   405
     * 7461   404   1107   405   1106   405   1108   404   1108   404   1108   405   1676   405   1107   404   1108   404
     * 7461   404   1107   405   1107   405   1110   405   1106   405   1106   405   1675   404   1107   405   1106   405
     * 7457   404   1107   405   1107
     *
     *
     * 1095   398   597   399   1094   398   1094   399   1094   398   1094   399   1124   398   1179   398   1094   398
     * 1094   398   597   398   597   398   597   398   597   398   597   398   597   398
     * 4020   398   1094   399   596   399   1094   398   1095   398   1095   398   1094   399   1124   398   1180   398
     * 1094   398   1095   398   597   398   596   399   596   399   596   398   597   399
     *
     *
     * Duty & Pause Timing for SumPPM:
     *                                        399   1105   398   1105   398   1105   399   1104   399   1617   399   8094
     * 401   1104   399   1116   399   1104   399   1106   399   1105   398   1105   399   1105   399   1618   399   8093
     * 401   1102   399   1116   399   1104   399   1106   398   1106   398   1105   399   1105   398   1617   399   8094
     * 401   1102   399   1117   399   1104   398   1106   399   1105   398   1105   399   1105   398   1617
     *
     * Duty & Pause Timing for SumPPM:
     *                           398   1105   398   1105   398   1104   399   1104   399   1105   398   591   398   9121
     * 401   1102   398   1117   398   1105   399   1106   398   1105   398   1105   399   1104   399   591   398   9124
     * 401   1102   398   1117   399   1105   398   1106   398   1105   399   1105   398   1105   398   591   399   9120
     *
     */
    if (htim->Instance == TIM1)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3 || htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) // changed to be rising/falling independent
        {
            /*
             * The channel3 or 4 interrupt fires at the falling edge, thus we know the duty
             * 3______________4               3___
             * |              |_______________|   ......
             */

            // It is important to use a uint16 here as the timer returns a 16 bit value, although it is defined as 32 bit uint
            uint16_t diff;
            if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
            {
                diff = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4) - HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
            }
            else
            {
                diff = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3) - HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
            }
            setNextDuty(diff);

            if (diff > 10000)   // a very long signal is the packet-end of a servo
            {
                if (current_virtual_channel == 1)
                {
                    sbusdata.servovalues[0].duty = servovalues[0];
                    sbusdata.sbusLastValidFrame = HAL_GetTick(); // and we got a valid frame, so set the timestamp to now
                    sbusdata.counter_sbus_valid_data++;
                    sbusdata.receivertype = RECEIVER_TYPE_SERVO;
                }
                current_virtual_channel = 0; // Hence the next duty signal will be for the first channel
            }
            else if (diff > 3500)   // a long signal is the packet-end of a sum ppm
            {
                if (current_virtual_channel == 8 || current_virtual_channel == 16)
                {
                    for (uint16_t i = 0; i < current_virtual_channel; i++)
                    {
                        sbusdata.servovalues[i].duty = servovalues[i];
                    }
                    sbusdata.sbusLastValidFrame = HAL_GetTick(); // and we got a valid frame, so set the timestamp to now
                    sbusdata.counter_sbus_valid_data++;
                    sbusdata.receivertype = RECEIVER_TYPE_SUMPPM;
                }
                current_virtual_channel = 0; // Hence the next duty signal will be for the first channel
            }
            else if (diff < 550) // a sumppm pause
            {

            }
            else if (current_virtual_channel < SBUS_MAX_CHANNEL) // What if the SumPPM sends more than 16 values? Ignore those.
            {
                servovalues[current_virtual_channel++] = diff;
            }
        }
    }
    else if (htim->Instance == TIM5)
    {
        controllerstatus.last_possensortick = HAL_GetTick();
        controllerstatus.possensorduration = controllerstatus.last_possensortick - possensortick_old;
        /*
         * Need to deal with the situation that the first pulse after a long stand still would have very high duration.
         * Limit to a minimum speed
         */
        if (controllerstatus.possensorduration > 1000)
        {
            controllerstatus.possensorduration = 0;
        }
        possensortick_old = controllerstatus.last_possensortick;
    }

}
