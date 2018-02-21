#include "config.h"
#include "protocol.h"
#include "controller.h"
#include "protocol.h"
#include "clock_50Hz.h"
#include "sbus.h"
#include "vesc.h"
#include "math.h"

extern sbusData_t sbusdata;

void printControlLoop(int16_t input, float speed, float pos, float brakedistance, uint16_t esc, Endpoints endpoint);
float stickCycle(float pos, float brakedistance);

/*
 * Preserve the previous filtered stick value to calculate the acceleration
 */
float stick_last_value = 0.0f;

uint32_t possensorduration = 0;
uint32_t last_possensortick = 0;

float speed_current = 0.0f;


int32_t pos_current_old = 0L;
float pos_target = 0.0f, pos_target_old = 0.0f;

uint8_t endpointclicks = 0;
float lastendpointswitch = 0.0f;
uint8_t lastmodeswitchsetting = 255;


float getSpeedPosSensor(void)
{
    if (HAL_GetTick() - last_possensortick > 2000)
    {
        return 0.0f;
    }
    else if (possensorduration == 0)
    {
        return 0.0f;
    }
    else
    {
        /* Speed is the number of signals per 0.02 (=Ta) secs.
         * Hence refactor Ta to millisenconds to match the HAL_GetTick() millisecond time scale.
         * But the encoder is set to by an X4 type, so it updates the position on rising and falling flanks
         * of both channels whereas the interrupt fires on one pin and rising flank only - hence times 4.
         */
        return CONTROLLERLOOPTIME_FLOAT * 1000.0f * 4.0f / ((float) possensorduration);
    }
}

float getSpeedPosDifference(void)
{
    return speed_current;
}

int32_t getTargetPos(void)
{
    return pos_target;
}


float getStick(void)
{
    return stick_last_value;
}

int32_t getPos(void)
{
    return ENCODER_VALUE;
}

void initController(void)
{
    controllerstatus.safemode = INVALID_RC;
}

float abs_d(float v)
{
    if (v < 0.0f)
    {
        return -v;
    }
    else
    {
        return v;
    }
}

float getProgrammingSwitch()
{
    return getDuty(activesettings.rc_channel_programming);
}

float getEndPointSwitch()
{
    return getDuty(activesettings.rc_channel_endpoint);
}

float getMaxAccelPoti()
{
    return getDuty(activesettings.rc_channel_max_accel);
}

float getMaxSpeedPoti()
{
    return getDuty(activesettings.rc_channel_max_speed);
}

float getModeSwitch()
{
    return getDuty(activesettings.rc_channel_mode);
}

/*
 * getStickPositionRaw() returns the position in percent from -1.0 to 0 to +1.0 of the stick considering the neutral range.
 * Everything within the neutral range means a stick position of zero, the first value outside the neutral range would be 1.
 * Examples:
 *   sbus reading: 885; neutral: 870..890 --> return 0;
 *   sbus reading: 860; neutral: 870..890 --> return 860-870=-10;
 *
 * Therefore the output is a linear range from min to max with the neutral range eliminated from the value.
 * Also, when the reading is too old, meaning older than 3 seconds, the value is reset to NAN as emergency precaution.
 *
 */
float getStickPositionRaw()
{
    /*
     * "value" is the duty signal rebased from the stick_neutral_pos to zero.
     * So when getDuty() returns 900 and the stick_neutral_pos is 1000, the value would be -100.
     *
     * Remember that getDuty() itself can return a value of 0 as well in case no valid RC signal was received either
     * or non received for a while.
     */
    float value = getDuty(activesettings.rc_channel_speed);

    if (isnan(value)) // in case getDuty() returned 0 meaning no valid stick position was received...
    {
        // No valid value for that, use Neutral
        return 0.0f;
    }
    else
    {
        /*
         * At the beginning the safemode == INVALID_RC, meaning no valid signal was received ever.
         * If a valid rc signal is received, then the safemode is changed according to the programming switch.
         * There is one more case, we got a valid speed signal but the stick is not in neutral. This is considered
         * an invalid signal as well. At startup the speed signal has to be in idle.
         */
        if ((controllerstatus.safemode == INVALID_RC || controllerstatus.safemode == NOT_NEUTRAL_AT_STARTUP) && (value != 0.0f))
        {
            if (controllerstatus.safemode == INVALID_RC)
            {
                PrintSerial_string("A valid RC signal with value ", EndPoint_All);
                PrintSerial_float(value, EndPoint_All);
                PrintSerial_string("% received on channel ", EndPoint_All);
                PrintSerial_int(value, EndPoint_All);
                PrintSerial_string(".\r\nCheck the RC, the channel assignments $i and the input neutral point settings $n", EndPoint_All);

                controllerstatus.safemode = NOT_NEUTRAL_AT_STARTUP; // Statemachine to the next level
            }
            return 0.0f;
        }
    }
    return value;
}


/** \brief The receiver provides a stick position and this function applies filters. Returns the filtered value.
 *
 * Depending on the CableCam mode and the distance to brake various filters are applied.
 * In short these filters are acceleration limiter, speed limiter, engage brake for endpoints.
 *
 * In MODE_PASSTHROUGH none of the filters are used, stick output = input (actually output = input*10).
 *
 * In all other modes at least the acceleration and speed filters are active. These should avoid that the
 * user cranks the stick forward, wheels spinning. Instead the filter applies a ramp, increasing the
 * speed constantly by the defined acceleration factor until the output matches the input or the max speed
 * has been reached. What the values for acceleration and speed actually are depends on the programming switch position.
 * In case the endpoints should be programmed the values are stick_max_accel_safemode and stick_max_speed_safemode.
 * In case of OPERATIONAL it is stick_max_accel and stick_max_speed. (see $a and $v commands).
 *
 * The second type of filter is the endpoint limiter. If the CableCam is in danger to hit the start- or endpoint at the
 * current speed with the stick_max_accel as deceleration, the stick is slowly moved into neutral. The endpoint limiter
 * is obviously engaged in OPERATIONAL mode only, otherwise the end points could not be set to further outside.
 *
 * \param pos float Current position
 * \param brakedistance float Brake distance at current speed
 * \return stick_requested_value float
 *
 */
float stickCycle(float pos, float brakedistance)
{
    float stickpercent = getStickPositionRaw();
    // output = ( (1 - factor) x input^3 ) + ( factor x input )     with input and output between -1 and +1

    float value = ((1.0f - (float) activesettings.expo_factor) * stickpercent * stickpercent * stickpercent) + (activesettings.expo_factor * stickpercent);

    int32_t speed = ENCODER_VALUE - pos_current_old; // When the current pos is greater than the previous, speed is positive

    // In passthrough mode the returned value is the raw value
    if (activesettings.mode != MODE_PASSTHROUGH)
    {
        /*
         * In all other modes than Passthroug the accel and speed limiters are turned on
         */
        float maxaccel = activesettings.stick_max_accel;
        float maxspeed = activesettings.stick_max_speed;

        if (controllerstatus.safemode != OPERATIONAL)
        {
            maxaccel = activesettings.stick_max_accel_safemode;
            maxspeed = activesettings.stick_max_speed_safemode;
        }

        float diff = value - stick_last_value;

        // First handle the controller status state machine to print out a warning
        if (diff > maxaccel || diff < -maxaccel)
        {
            controllerstatus.accel_limiter = true;
        }
        else
        {
            controllerstatus.accel_limiter = false;
        }

        // Now limit to max accel
        if (diff > maxaccel)
        {
            // Example: last time: +0.20; now stick: +1.00 --> diff: +0.80; max change is 0.01 per cycle --> new value: +0.20+0.01=+0.21 as the limiter kicks in
            value = stick_last_value + maxaccel;
        }
        else if (diff < -maxaccel)
        {
            // Example: last time: +1.00; now stick: +0.20 --> diff: -0.80; max change is 0.01 per cycle --> new value: +1.00-0.01=+0.99 as the limiter kicks in
            value = stick_last_value - maxaccel;
        }

        /*
         * It is important to calculate the new value based on the acceleration first, then we have the new target speed,
         * now it is limited to an absolute value.
         */
        if (value > maxspeed || value < -maxspeed)
        {
            controllerstatus.speed_limiter = true;
        }
        else
        {
            controllerstatus.speed_limiter = false;
        }

        if (value > maxspeed)
        {
            value = maxspeed;
        }
        else if (value < -maxspeed)
        {
            value = -maxspeed;
        }



        if (controllerstatus.safemode == OPERATIONAL && activesettings.mode != MODE_PASSTHROUGH && activesettings.mode != MODE_LIMITER)
        {
            /*
             * The current status is OPERATIONAL, hence the endpoints are honored.
             *
             */


            /*
             * The pos_start has to be smaller than pos_end always. This is checked in the end_point set logic.
             * However there is a cases where this might not be so:
             * Start and end point had been set but then only the start point is moved.
             */
            if (activesettings.pos_start > activesettings.pos_end)
            {
                float tmp = activesettings.pos_start;
                activesettings.pos_start = activesettings.pos_end;
                activesettings.pos_end = tmp;
            }

            /*
             * One problem is the direction. Once the endpoint was overshot, a stick position driving the cablecam even further over
             * the limit is not allowed. But driving it back between start and end point is fine. But what value of the stick
             * is further over the limit? activesettings.esc_direction tells that.
             *
             * Case 1: start pos_start = 0, stick forward drove cablecam to pos_end +3000. Hence esc_direction = +1
             *         Condition: if value > 0 && pos+brakedistance <= pos_end.
             *                Or: if value > 0 && pos+brakedistance <= pos_end.
             * Case 2: start pos_start = 0, stick forward drove cablecam to pos_end -3000. Hence esc_direction = -1.
             *         Condition: if value < 0 && pos+brakedistance <= pos_end.
             *                Or: if -value > 0 && pos+brakedistance <= pos_end.
             * Case 3: start pos_start = 0, stick reverse drove cablecam to pos_end +3000. Hence esc_direction = -1.
             *         Condition: if value < 0 && pos+brakedistance <= pos_end.
             *                Or: if -value > 0 && pos+brakedistance <= pos_end.
             * Case 4: start pos_start = 0, stick reverse drove cablecam to pos_end -3000. Hence esc_direction = +1.
             *         Condition: if value > 0 && pos+brakedistance <= pos_end.
             *                Or: if value > 0 && pos+brakedistance <= pos_end.
             *
             *
             */

            if (pos + brakedistance < activesettings.pos_end && pos - brakedistance > activesettings.pos_start)
            {
                controllerstatus.endpointbrake = false;
                controllerstatus.emergencybrake = false;
            }
            else
            {
                if (pos + brakedistance >= activesettings.pos_end)
                {
                    /*
                     * In case the CableCam will overshoot the end point reduce the speed to zero.
                     * Only if the stick is in the reverse direction already, accept the value. Else you cannot maneuver
                     * back into the safe zone.
                     */
                    if (((float) activesettings.esc_direction) * value > 0.0f)
                    {
                        value = stick_last_value - (maxaccel * ((float) activesettings.esc_direction)); // reduce speed at full allowed acceleration
                        if (value * ((float) activesettings.esc_direction) < 0.0f) // watch out to not get into reverse.
                        {
                            value = 0.0f;
                        }
                        if (controllerstatus.endpointbrake == false)
                        {
                            PrintlnSerial_string("Endpoint brake on ", EndPoint_All);
                        }
                        controllerstatus.endpointbrake = true;
                        LED_WARN_ON;
                        /*
                         * Failsafe: In case the endpoint itself had been overshot and stick says to do so further, stop immediately.
                         */
                        if (pos >= activesettings.pos_end)
                        {
                            stick_last_value = 0.0f;
                            return 0.0f;
                        }
                    }
                    else
                    {
                        controllerstatus.endpointbrake = false;
                        controllerstatus.emergencybrake = false;
                        LED_WARN_OFF;
                    }

                    /*
                     * Failsafe: No matter what the user did going way over the endpoint at speed causes this function to return a speed=0 request.
                     */
                    if (pos + brakedistance >= activesettings.pos_end + activesettings.max_position_error && speed > 0)
                    {
                        /*
                         * We are in danger to overshoot the end point by max_position_error because we are moving with a speed > 0 towards
                         * the end point. Hence kick in the emergency brake.
                         */
                        if (controllerstatus.emergencybrake == false)
                        {
                            PrintlnSerial_string("Emergency brake on ", EndPoint_All);
                        }
                        controllerstatus.emergencybrake = true;
                        LED_WARN_ON;
                        stick_last_value = value;
                        return 0.0f;
                    }
                }


                if (pos - brakedistance <= activesettings.pos_start)
                {
                    /*
                     * In case the CableCam will overshoot the start point reduce the speed to zero.
                     * Only if the stick is in the reverse direction already, accept the value. Else you cannot maneuver
                     * back into the safe zone.
                     */
                    if (((float) activesettings.esc_direction) * value < 0.0f)
                    {
                        value = stick_last_value + (maxaccel * ((float) activesettings.esc_direction)); // reduce speed at full allowed acceleration
                        if (value * ((float) activesettings.esc_direction) > 0.0f) // watch out to not get into reverse.
                        {
                            value = 0.0f;
                        }
                        if (controllerstatus.endpointbrake == false)
                        {
                            PrintlnSerial_string("Endpoint brake on ", EndPoint_All);
                        }
                        controllerstatus.endpointbrake = true;
                        LED_WARN_ON;
                        /*
                         * Failsafe: In case the endpoint itself had been overshot and stick says to do so further, stop immediately.
                         */
                        if (pos <= activesettings.pos_start)
                        {
                            stick_last_value = 0.0f;
                            return 0.0f;
                        }
                    }
                    else
                    {
                        controllerstatus.endpointbrake = false;
                        controllerstatus.emergencybrake = false;
                        LED_WARN_OFF;
                    }

                    /*
                     * Failsafe: No matter what the user did going way over the endpoint at speed causes this function to return a speed=0 request.
                     */
                    if (pos - brakedistance <= activesettings.pos_start - activesettings.max_position_error && speed < 0)
                    {
                        /*
                         * We are in danger to overshoot the end point by max_position_error because we are moving with a speed > 0 towards
                         * the end point. Hence kick in the emergency brake.
                         */
                        if (controllerstatus.emergencybrake == false)
                        {
                            PrintlnSerial_string("Emergency brake on ", EndPoint_All);
                        }
                        controllerstatus.emergencybrake = true;
                        LED_WARN_ON;
                        stick_last_value = value;
                        return 0.0f;
                    }
                }
            }
        }

    }
    stick_last_value = value; // store the current effective stick position as value for the next cycle







    /*
     * Evaluate the programming mode switch.
     *
     * A HIGH value is considered normal operation
     * Everything else
     *                  LOW value
     *                  value of neutral
     *                  if the channel is not set at all (value == 0)
     * is considered the programming mode with reduced speeds.
     *
     * The reason for including NEUTRAL as non-operational is because that would indicate this channel is not set properly.
     */
    if (getProgrammingSwitch() > 0.8f)
    {
        if (controllerstatus.safemode != OPERATIONAL)
        {
            PrintlnSerial_string("Entered OPERATIONAL mode", EndPoint_All);
        }
        controllerstatus.safemode = OPERATIONAL;
    }
    else
    {
        if (controllerstatus.safemode != PROGRAMMING)
        {
            /* We just entered the programing mode, hence wait for the first end point
             * First endpoint is the first click, Second endpoint the second click.
             */
            endpointclicks = 0;
            PrintlnSerial_string("Entered Endpoint Programming mode", EndPoint_All);
        }
        controllerstatus.safemode = PROGRAMMING;
    }




    /*
     * Evaluate the End Point Programming switch
     */
    float currentendpointswitch = getEndPointSwitch();
    if (currentendpointswitch > 0.8f && controllerstatus.safemode == PROGRAMMING && lastendpointswitch <= 0.8f && !isnan(lastendpointswitch))
    {
        int32_t pos = ENCODER_VALUE;
        if (endpointclicks == 0)
        {
            activesettings.pos_start = pos;
            endpointclicks = 1;
            PrintlnSerial_string("Point 1 set", EndPoint_All);
        }
        else
        {
            /*
             * Subsequent selections of the endpoint just moves the 2nd Endpoint around. Else there
             * is the danger the user selects point 1 at zero, then travels to 1000 and selects that as endpoint. Then clicks again
             * and therefore the range is from 1000 to 1000, so no range at all. To avoid that, force to leave the programming mode
             * temporarily for setting both points again.
             */
            PrintlnSerial_string("Point 2 set", EndPoint_All);
            if (activesettings.pos_start < pos)
            {
                activesettings.pos_end = pos;
            }
            else
            {
                activesettings.pos_end = activesettings.pos_start;
                activesettings.pos_start = pos;
            }
        }
    }
    lastendpointswitch = currentendpointswitch; // Needed to identify a raising flank on the tip switch


    /*
     * Evaluate the Max Acceleration Potentiometer
     */
    float max_accel = getMaxAccelPoti();
    if (!isnan(max_accel))
    {
        /*
         * The value for max_accel coming from the dial can be anything between -1.0 and +1.0. As we want the full range
         * this is shifted to 0.0 to 2.0. But how quickly should the stick move? In 5 seconds from 0 to +1.0? That would be an acceleration value of 1/5/50=0.004.
         * Hence rescaling it by dividing by 500.
         * And a minimum value of 0.00 does not make sense either as it would mean no stick movement at all. The absolute minimum is 0.001 which would represent a 1/(0.001*50) = 20sec time from 0 to 1.0
          */
        activesettings.stick_max_accel = (1.0f + max_accel)/500.0f+0.001f;
    }

    /*
     * Evaluate the Max Speed Potentiometer
     */
    float max_speed = getMaxSpeedPoti();
    if (!isnan(max_speed))
    {
        // Again, the full range of the poti from -1.0 to +1.0 should result in a value range of 0.1 to 1.0
        activesettings.stick_max_speed = (1.0f + max_speed)/2.223f + 0.1f;
    }

    /*
     * Evaluate the Mode Switch
     */
    float modeswitch = getModeSwitch();
    if (!isnan(modeswitch))
    {
        if (modeswitch < 0.0f)
        {
            /*
             * Passthrough
             */
            activesettings.mode = MODE_PASSTHROUGH;
        }
        else if (modeswitch > 0.0f)
        {
            /*
             * Full Limiters
             */
            activesettings.mode = MODE_LIMITER_ENDPOINTS;
        }
        else
        {
            activesettings.mode = MODE_LIMITER;
        }

        if (lastmodeswitchsetting != activesettings.mode)
        {
            lastmodeswitchsetting = activesettings.mode;
            PrintlnSerial_string(getCurrentModeLabel(activesettings.mode), EndPoint_All);
        }

    }

    return value;
}


// ******** Main Loop *********
void controllercycle()
{
    /*
     * speed = change in position per cycle. Speed is always positive.
     * speed = abs(pos_old - pos)
     *
     * time_to_stop = number of cycles it takes to get the current stick value down to neutral with the given activesettings.stick_max_accel
     * time_to_stop = abs(getStick()/max_accel)
     *
     *
     * brake_distance = v²/(2*a) = speed²/(2*a)
     * Problem is, the value of a, the deceleration rate needed is unknown. All we know is the current speed and the stick position.
     * But we know that the current speed should be zero after time_to_stop cycles, hence we know the required
     * deceleration in speed-units: a = speed/time_to_Stop
     *
     * Hence:
     * brake_distance = speed²/(2*a) = speed²/(2*speed/time_to_Stop) = speed * time_to_stop / 2
     *
     * Or in other words: distance = velocity * time; hence brake_distance = velocity * time / 2
     *   |
     * s |--
     * p |  \
     * e |   \
     * e |    \
     * d |     \
     *   |      \
     *   |       \
     *    ----------------------------------
     *      time_to_stop
     */
    int32_t pos_current = ENCODER_VALUE;
    speed_current = abs_d((float) (pos_current_old - pos_current));
    float pos = (float) pos_current;
    float speed_timer = getSpeedPosSensor();
    float speed = speed_current;

    /* If the pos difference is less than 5.0f per 20ms, then the resolution is not high enough. In that
     * case use the time between to pos sensor interrupts to calculate a more precise speed.
     */
    if (speed_current <= 5.0f)
    {
        speed = speed_timer;
    }

    float time_to_stop = abs_d(getStick()/activesettings.stick_max_accel);

    float distance_to_stop = speed * time_to_stop / 2.0f;
    float stick_filtered_value = stickCycle(pos, distance_to_stop); // go through the stick position calculation with its limiters, max accel etc

    /*
     * PPM output is rescaled to +-700 and adding the ESC's neutral range to get an immediate output.
     */
    if (stick_filtered_value > 0.0f)
    {
        TIM3->CCR3 = activesettings.esc_neutral_pos + activesettings.esc_neutral_range + ((int16_t) (stick_filtered_value * 700.0f));
    }
    else if (stick_filtered_value < 0.0f)
    {
        TIM3->CCR3 = activesettings.esc_neutral_pos - activesettings.esc_neutral_range + ((int16_t) (stick_filtered_value * 700.0f));
    }
    else
    {
        TIM3->CCR3 = activesettings.esc_neutral_pos;
    }
    VESC_Output(stick_filtered_value);



    /*
     * Log the last CYCLEMONITOR_SAMPLE_COUNT events in memory.
     * If neither the cablecam moves nor should move (esc_output == 0), then there is nothing interesting to log
     */
    if (speed != 0.0f || stick_filtered_value != 0.0f)
    {
        cyclemonitor_t * sample = &controllerstatus.cyclemonitor[controllerstatus.cyclemonitor_position];
        sample->distance_to_stop = distance_to_stop;
        sample->esc = TIM3->CCR3;
        sample->pos = pos;
        sample->speed = speed;
        sample->stick = getStick();
        sample->tick = HAL_GetTick();
        controllerstatus.cyclemonitor_position++;
        if (controllerstatus.cyclemonitor_position > CYCLEMONITOR_SAMPLE_COUNT)
        {
            controllerstatus.cyclemonitor_position = 0;
        }
    }


    pos_current_old = pos_current; // required for the actual speed calculation

    if (is1Hz())
    {
        // printControlLoop(stick_filtered_value, speed, pos, distance_to_stop, controllerstatus.monitor, TIM3->CCR3, EndPoint_USB);
    }
}

void printControlLoop(int16_t input, float speed, float pos, float brakedistance, uint16_t esc, Endpoints endpoint)
{
    PrintSerial_string("LastValidFrame: ", endpoint);
    PrintSerial_long(sbusdata.sbusLastValidFrame, endpoint);
    PrintSerial_string("  Duty: ", endpoint);
    PrintSerial_int(getDuty(activesettings.rc_channel_speed), endpoint);
    PrintSerial_string("  Raw: ", endpoint);
    PrintSerial_int(sbusdata.servovalues[activesettings.rc_channel_speed].duty, endpoint);
    PrintSerial_string("  ESC Out: ", endpoint);
    PrintSerial_int(esc, endpoint);
    PrintSerial_string("  ", endpoint);
    PrintSerial_string(getSafeModeLabel(), endpoint);
    PrintSerial_string("  ESC Input: ", endpoint);
    PrintSerial_int(input, endpoint);
    PrintSerial_string("  Speed: ", endpoint);
    PrintSerial_float(speed, endpoint);
    PrintSerial_string("  Brakedistance: ", endpoint);
    PrintSerial_float(brakedistance, endpoint);

    PrintSerial_string("  Calculated stop: ", endpoint);
    if (input > 0)
    {
        if (pos + brakedistance > activesettings.pos_end)
        {
            PrintSerial_float(pos + brakedistance, endpoint);
        }
        else
        {
            PrintSerial_string("         ", endpoint);
        }
    }
    else
    {
        if (pos - brakedistance < activesettings.pos_start)
        {
            PrintSerial_float(pos - brakedistance, endpoint);
        }
        else
        {
            PrintSerial_string("         ", endpoint);
        }
    }

    PrintSerial_string("  Pos: ", endpoint);
    PrintlnSerial_float(pos, endpoint);
}
