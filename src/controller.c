#include "config.h"
#include "protocol.h"
#include "controller.h"
#include "protocol.h"
#include "clock_50Hz.h"
#include "sbus.h"
#include "vesc.h"
#include "math.h"
#include "eeprom.h"

void printControlLoop(int16_t input, float speed, float pos, float brakedistance, uint16_t esc, Endpoints endpoint);
float stickCycle(float brakedistance);
float getStickPositionRaw(void);
float abs_d(float v);
float getGimbalDuty(uint8_t channel);

/*
 * Preserve the previous filtered stick value to calculate the acceleration
 */
float stick_last_value = 0.0f;

uint32_t possensorduration = 0;
uint32_t last_possensortick = 0;

float speed_current = 0.0f;
static float stickintegral = 0.0f;


int32_t pos_current_old = 0L;

uint8_t endpointclicks = 0;
float lastendpointswitch = 0.0f;
uint8_t lastmodeswitchsetting = 255;

float gimbalout[8];


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
         * The encoder is set to by an X4 type and increases on falling and raising flanks
         * whereas the interrupt fires on one pin and rising flank only - hence times 4.
         */
        return CONTROLLERLOOPTIME_FLOAT * 1000.0f * 4.0f / ((float) possensorduration);
    }
}

float getSpeedPosDifference(void)
{
    return speed_current;
}


float getStick(void)
{
    return stick_last_value;
}

float getPos(void)
{
    int32_t v = ENCODER_VALUE; // Convert uint to int

    if (activesettings.esc_direction == 0.0f)
    {
        return (float) v;
    }
    else
    {
        return ((float) v) * activesettings.esc_direction;
    }
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

float getPlaySwitch()
{
    return getDuty(activesettings.rc_channel_play);
}

float getAuxInput()
{
    return getDuty(activesettings.rc_channel_aux);
}

/*
 * getStickPositionRaw() returns the position in percent from -1.0 to 0 to +1.0 of the stick considering the neutral range.
 * Everything within the neutral range means a stick position of zero, the first value outside the neutral range would be 1.
 * Examples:
 *   sbus reading: 885; neutral: 870..890 --> return 0;
 *   sbus reading: 860; neutral: 870..890 --> return 860-870=-10;
 *
 * Therefore the output is a linear range from min to max with the neutral range eliminated from the value.
 * Also, when the reading is too old, meaning older than 3 seconds, the value is reset to NAN as ency precaution.
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
        else
        {
            float pos_current = getPos();
            if (controllerstatus.play_running == 1)
            {
                if (controllerstatus.safemode != OPERATIONAL)
                {
                    controllerstatus.play_running = 2;
                    PrintlnSerial_string("Cannot Play, not in operational mode but endpoint programming mode", EndPoint_All);
                }
                else if (activesettings.mode != MODE_LIMITER_ENDPOINTS)
                {
                    controllerstatus.play_running = 2;
                    PrintlnSerial_string("Cannot Play, not limiter with endpoints mode", EndPoint_All);
                }
                else if (value != 0.0f)
                {
                    controllerstatus.play_running = 2;
                    PrintlnSerial_string("Cannot Play, stick is not in neutral", EndPoint_All);
                }
                else if (semipermanentsettings.pos_start == -POS_END_NOT_SET)
                {
                    controllerstatus.play_running = 2;
                    PrintlnSerial_string("Cannot Play, pos_end is not set", EndPoint_All);
                }
                else if (semipermanentsettings.pos_end == POS_END_NOT_SET )
                {
                    controllerstatus.play_running = 2;
                    PrintlnSerial_string("Cannot Play, pos_start is not set", EndPoint_All);
                }
                else if (activesettings.esc_direction == 0.0f)
                {
                    controllerstatus.play_running = 2;
                    PrintlnSerial_string("Cannot Play, esc direction is not set. See $r", EndPoint_All);
                } else
                {
                    /*
                     * The condition to play a preprogrammed movement is very narrow. It requires that
                     * - end points are set
                     * - stick is in neutral
                     * - mode is the limiter with endpoints mode
                     * - not in the programming mode
                     * - and of course the play command was triggered
                     */
                    if (HAL_GetTick() - controllerstatus.play_time_lastsignal < 2000L)
                    {
                        /*
                         * Only when the play signal is sent continuously, we actually play the program. If the RC or app is disconnected, we don't.
                         */
                        if (controllerstatus.play_direction == 0)
                        {
                            if (pos_current - semipermanentsettings.pos_start < semipermanentsettings.pos_end - pos_current)
                            {
                                /*
                                 * Example: pos_current = 400; pos_start = 0; pos_end = 1000;
                                 * 400 - 0 < 1000-400
                                 *
                                 * --> Closer to the start point, so drive to pos_end first
                                 */
                                controllerstatus.play_direction = 1;
                            }
                            else
                            {
                                controllerstatus.play_direction = -1;
                            }
                        }

                        if (HAL_GetTick() - controllerstatus.play_endpoint_reached_at < 5000L)  // wait for 5 seconds at the start/end point
                        {
                            value = 0.0f;
                        }
                        else
                        {
                            if (controllerstatus.play_direction == 1) // Drive towards the end point
                            {
                                if (pos_current < semipermanentsettings.pos_end - 44.0f)
                                {
                                    // Continue driving towards the end point
                                    value = activesettings.stick_max_speed;
                                }
                                else
                                {
                                    // End point reached (almost), hence revert the direction and restart the countdown
                                    controllerstatus.play_endpoint_reached_at = HAL_GetTick();
                                    controllerstatus.play_direction = -1;
                                }
                            }
                            else
                            {
                                // Driving towards the start point
                                if (pos_current > semipermanentsettings.pos_start + 44.0f)
                                {
                                    // current position is far away from the start point, hence continue driving towards it
                                    value = -activesettings.stick_max_speed;
                                }
                                else
                                {
                                    // Got in reach of the start point, hence reverse direction and restart the wait countdown
                                    controllerstatus.play_endpoint_reached_at = HAL_GetTick();
                                    controllerstatus.play_direction = 1;
                                }
                            }
                        }
                    }
                }
            }
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
float stickCycle(float brakedistance)
{
    float stickpercent = getStickPositionRaw();
    float pos = getPos();


    // output = ( (1 - factor) x input^3 ) + ( factor x input )     with input and output between -1 and +1
    float value = ((1.0f - (float) activesettings.expo_factor) * stickpercent * stickpercent * stickpercent) + (activesettings.expo_factor * stickpercent);

    // In passthrough mode the returned value is the raw value
    if (activesettings.mode != MODE_PASSTHROUGH)
    {
        /*
         * In all other modes than Passthroug the accel and speed limiters are turned on
         */
        float maxaccel = controllerstatus.stick_max_accel;

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
            if (semipermanentsettings.pos_start > semipermanentsettings.pos_end)
            {
                float tmp = semipermanentsettings.pos_start;
                semipermanentsettings.pos_start = semipermanentsettings.pos_end;
                semipermanentsettings.pos_end = tmp;
            }

            /*
             * One problem is the direction. Once the endpoint was overshot, a stick position driving the cablecam even further over
             * the limit is not allowed. But driving it back between start and end point is fine. But what value of the stick
             * is further over the limit?
             * The rule is that pos_start < pos_end and a positive speed means driving towards the endpoint. Since that cannot be
             * guaranteed to be the case, the activesettings.esc_direction ($r command) controls that. In case this value is -1 the
             * getPos() funtion returns the mirrored values, makes them negative.
             *
             * Case 1: start pos_start = 0, stick forward drove cablecam to pos_end +3000. Hence esc_direction = +1
             *         Condition: if value > 0 && pos+brakedistance <= pos_end.
             *                Or: if value > 0 && pos+brakedistance <= pos_end.
             * Case 4: start pos_start = 0, stick reverse drove cablecam to pos_end -3000. Hence esc_direction = +1.
             *         Condition: if value > 0 && pos+brakedistance <= pos_end.
             *                Or: if value > 0 && pos+brakedistance <= pos_end.
             *
             *
             */

            /*
             * First check if the position is safe, both end points cannot be overshot at the current speed. In that case no braking calculation is needed.
             */
            if (pos + brakedistance < semipermanentsettings.pos_end && pos - brakedistance > semipermanentsettings.pos_start)
            {
                controllerstatus.endpointbrake = false;
                controllerstatus.emergencybrake = false;
            }
            else
            {
                /*
                 * Which end point is in danger? The cablecam might be very close to the start point but is driving towards the other end point -> no problem.
                 */
                if (pos + brakedistance >= semipermanentsettings.pos_end)
                {
                    /*
                     * In case the CableCam will overshoot the end point reduce the speed to zero.
                     * Only if the stick is in the reverse direction already, accept the value. Without that condition you cannot move the cablecam
                     * back into the safe zone.
                     */
                    if (value > 0.0f)
                    {
                        LED_WARN_ON;
                        /*
                         * Failsafe: In case the endpoint itself had been overshot and stick says to do so further, stop immediately.
                         * As there is some elasticity between motor and actual position, add a value here.
                         */
                        if (pos >= semipermanentsettings.pos_end)
                        {
                            stick_last_value = 0.0f;
                            return stick_last_value;
                        }
                        else
                        {
                            value = stick_last_value - maxaccel; // reduce speed at full allowed acceleration
                            if (value < 0.0f) // watch out to not get into reverse.
                            {
                                value = 0.0f;
                            }
                            if (controllerstatus.endpointbrake == false)
                            {
                                PrintSerial_string("Endpoint brake on. Distance=", EndPoint_All);
                                PrintSerial_float(semipermanentsettings.pos_end-pos, EndPoint_All);
                                PrintSerial_string(", calculated braking distance=", EndPoint_All);
                                PrintlnSerial_float(brakedistance, EndPoint_All);
                            }
                            controllerstatus.endpointbrake = true;

                            /*
                             * Failsafe: If, at the current speed the end point will be overshot significantly, ignore the ramp logic and request a full brake by setting speed=0.
                             * Because at high speeds the calculation might not be that accurate, add a factor depending on the brake distance.
                             *
                             * Example: pos = 800; max_position error = 100; pos_end = 1000
                             * at brakedistance = 200: 800+200 >= 1000+100+100*200/100 = 1000+100+200--> No emergency brake, we will stop nicely at the end point
                             * at brakedistance = 400: 800+400 >= 1000+100+100*400/100 = 1000+100+400
                             *
                             * Example: pos = 800; max_position error = 10; pos_end = 1000
                             * at brakedistance = 200: 800+200 >= 1000+10+10*200/100 = 1000+10+20--> No emergency brake, we will stop nicely at the end point
                             * at brakedistance = 400: 800+400 >= 1000+10+10*400/100 = 1000+10+40
                             */
                            if (activesettings.max_position_error != 0.0f &&
                                (pos + brakedistance >= semipermanentsettings.pos_end + activesettings.max_position_error + activesettings.max_position_error*brakedistance/100.0f))
                            {
                                /*
                                 * We are in danger to overshoot the end point by max_position_error. Hence kick in the emergency brake.
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
                    else // moving away from the end point into the safe zone
                    {
                        controllerstatus.endpointbrake = false;
                        controllerstatus.emergencybrake = false;
                        LED_WARN_OFF;
                    }

                }


                if (pos - brakedistance <= semipermanentsettings.pos_start)
                {
                    /*
                     * In case the CableCam will overshoot the start point reduce the speed to zero.
                     * Only if the stick is in the reverse direction already, accept the value. Else you cannot maneuver
                     * back into the safe zone.
                     */
                    if (value < 0.0f)
                    {
                        LED_WARN_ON;
                        /*
                         * Failsafe: In case the endpoint itself had been overshot and stick says to do so further, stop immediately.
                         */
                        if (pos <= semipermanentsettings.pos_start)
                        {
                            stick_last_value = 0.0f;
                            return 0.0f;
                        }
                        else
                        {
                            value = stick_last_value + maxaccel; // reduce speed at full allowed acceleration from a negative value up to zero
                            if (value > 0.0f) // watch out to not get into reverse.
                            {
                                value = 0.0f;
                            }
                            if (controllerstatus.endpointbrake == false)
                            {
                                PrintSerial_string("Endpoint brake on. Distance=", EndPoint_All);
                                PrintSerial_float(pos - semipermanentsettings.pos_start, EndPoint_All);
                                PrintSerial_string(", calculated braking distance=", EndPoint_All);
                                PrintlnSerial_float(brakedistance, EndPoint_All);
                            }
                            controllerstatus.endpointbrake = true;

                            /*
                             * Failsafe: If, at the current speed the end point will be overshot significantly, ignore the ramp logic and request a full brake by setting speed=0.
                             * Because at high speeds the calculation might not be that accurate, add a factor depending on the brake distance
                             */
                            if (activesettings.max_position_error != 0.0f &&
                                (pos - brakedistance <= semipermanentsettings.pos_start - activesettings.max_position_error - activesettings.max_position_error*brakedistance/100.0f))
                            {
                                /*
                                 * We are in danger to overshoot the start point by max_position_error. Hence kick in the emergency brake.
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
                    else
                    {
                        controllerstatus.endpointbrake = false;
                        controllerstatus.emergencybrake = false;
                        LED_WARN_OFF;
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
        stickintegral += value;
    }




    /*
     * Evaluate the End Point Programming switch
     */
    float currentendpointswitch = getEndPointSwitch();
    if (currentendpointswitch > 0.8f && controllerstatus.safemode == PROGRAMMING && lastendpointswitch <= 0.8f && !isnan(lastendpointswitch))
    {
        if (endpointclicks == 0)
        {
            semipermanentsettings.pos_start = pos;
            endpointclicks = 1;
            PrintlnSerial_string("Point 1 set", EndPoint_All);
            stickintegral = 0.0f;
        }
        else
        {
            /*
             * Subsequent selections of the endpoint just move the 2nd Endpoint around. Else there
             * is the danger the user selects point 1 at zero, then travels to 1000 and selects that as endpoint. Then clicks again
             * for the third time. This logic therefore just updates the 2nd point but never the first.
             * If the first point has to be changed, then the programming mode needs to be left and reengaged.
             *
             * Below logic has multiple loop holes that might occur.
             * 1. Possensor not working or setting start and endpoint without moving the cablecam. In that case the range is zero and the
             *    cablecam would stop moving. It is better this way than assuming everything was in order and the end points are unset.
             * 2. When the esc direction is unset the code tries to guess the direction. It sums up the stick input values and if the value is
             *    positive and the possensor did count up (or it is negative and the possensor did count down), then the direction is +1, else
             *    it is -1. But what if the cablecam did roll downhill and the stick was used to slow down its movement? Then the values would
             *    be with the wrong sign. Let's hope this never happens during the initial setup. That is also the reason why the esc direction
             *    is calculated only if the esc direction is unset yet.
             */
            semipermanentsettings.pos_end = pos;
            if (stickintegral > -3.0f && stickintegral < 3.0f)
            {
                PrintlnSerial_string("Endpoints too close together based on the stick signals. Move the cablecam via the RC.", EndPoint_All);
            }
            else
            {

                PrintlnSerial_string("Point 2 set", EndPoint_All);

                if (activesettings.esc_direction == 0.0f)
                {
                    /*
                     * The esc_direction has never been set. Hence set it together with the end points.
                     */
                    if ((semipermanentsettings.pos_start < semipermanentsettings.pos_end && stickintegral > 0.0f) ||
                        (semipermanentsettings.pos_start > semipermanentsettings.pos_end && stickintegral < 0.0f))
                    {
                        activesettings.esc_direction = +1.0f;
                        PrintlnSerial_string("ESC direction not set yet. Correct value seems to be $r 1", EndPoint_All);
                    }
                    else
                    {
                        activesettings.esc_direction = -1.0f;
                        semipermanentsettings.pos_start = -semipermanentsettings.pos_start;
                        semipermanentsettings.pos_end = -semipermanentsettings.pos_end;
                        PrintlnSerial_string("ESC direction not set yet. Correct value seems to be $r -1", EndPoint_All);
                    }
                    PrintlnSerial_string("Saving all settings ($w) to store the $r parameter permanently", EndPoint_All);
                    uint32_t write_errors = eeprom_write_sector_safe((uint8_t*) &activesettings, sizeof(activesettings), EEPROM_SECTOR_FOR_SETTINGS);
                    if (write_errors != 0)
                    {
                        PrintlnSerial_string("Saving the settings failed", EndPoint_All);
                    }
                }
                else
                {
                    if ((((semipermanentsettings.pos_start < semipermanentsettings.pos_end && stickintegral > 0.0f) ||
                        (semipermanentsettings.pos_start > semipermanentsettings.pos_end && stickintegral < 0.0f)) && activesettings.esc_direction == -1.0f) ||
                        (((semipermanentsettings.pos_start > semipermanentsettings.pos_end && stickintegral < 0.0f) ||
                        (semipermanentsettings.pos_start < semipermanentsettings.pos_end && stickintegral > 0.0f)) && activesettings.esc_direction == 1.0f))
                    {
                        PrintlnSerial_string("Is $r really correct? Does not look like it.", EndPoint_All);
                    }
                }
                PrintSerial_string("Drove from", EndPoint_All);
                PrintSerial_float(semipermanentsettings.pos_start, EndPoint_All);
                PrintSerial_string(" to", EndPoint_All);
                PrintlnSerial_float(semipermanentsettings.pos_end, EndPoint_All);


                if (semipermanentsettings.pos_start > semipermanentsettings.pos_end)
                {
                    // Rule is that pos_start < pos_end. Hence swap the two.
                    float tmp = semipermanentsettings.pos_start;
                    semipermanentsettings.pos_start = semipermanentsettings.pos_end;
                    semipermanentsettings.pos_end = tmp;
                }
                eeprom_write_sector_async((uint8_t*) &semipermanentsettings, sizeof(semipermanentsettings), EEPROM_SECTOR_FOR_SEMIPERMANENTSETTINGS);
            }
        }
    }
    lastendpointswitch = currentendpointswitch; // Needed to identify a raising flank on the tip switch


    float max_accel_scale;
    float max_speed_scale;
    if (controllerstatus.safemode != OPERATIONAL)
    {
        max_accel_scale = activesettings.stick_max_accel_safemode;
        max_speed_scale = activesettings.stick_max_speed_safemode;
    }
    else
    {
        max_accel_scale = activesettings.stick_max_accel;
        max_speed_scale = activesettings.stick_max_speed;
    }

    /*
     * Evaluate the Max Acceleration Potentiometer
     */
    float max_accel_poti = getMaxAccelPoti();
    if (!isnan(max_accel_poti))
    {
        /*
         * The value for max_accel coming from the dial can be anything between -1.0 and +1.0. As we want the full range
         * this is shifted to 0.0 to 2.0. But how quickly should the stick move? In 1 second from 0 to +1.0? That would
         * be an acceleration value of 1/1/50=0.02 (Assuming 50Hz controller cycles).
         * And a minimum value of 0.00 does not make sense either as it would mean no stick movement at all.
         * The absolute minimum shall be 20secs, so a value of 1/20/50 = 0.001
         * The maximum value should be the activesettings.stick_max_accel value or stick_max_accel_safemode - see above
         *
         * Example: poti reads +1 -> (1.0+1.0)/2 * max_accel_scale + 0.001 = max_accel_scale + 0.001
         */
        controllerstatus.stick_max_accel = (1.0f + max_accel_poti)/2.0f * (max_accel_scale - 0.001f) + 0.001f;
    }
    else
    {
        // If the poti is not providing a value value, the max_accel is the absolute maximum as defined.
        controllerstatus.stick_max_accel = max_accel_scale;
    }

    /*
     * Evaluate the Max Speed Potentiometer
     */
    float max_speed_poti = getMaxSpeedPoti();
    if (!isnan(max_speed_poti))
    {
        // Again, the full range of the poti from -1.0 to +1.0 should result in a value range of 0.1 to 1.0
        controllerstatus.stick_max_speed = (1.0f + max_speed_poti)/2.0f * (max_speed_scale - 0.1f) + 0.1f;
    }
    else
    {
        controllerstatus.stick_max_speed = max_speed_scale;
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

    float playswitch = getPlaySwitch();
    if (!isnan(playswitch) && playswitch > 0.8f)
    {
        controllerstatus.play_running = 1;
        controllerstatus.play_time_lastsignal = HAL_GetTick();
    }
    else
    {
        controllerstatus.play_running = 0;
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
     * brake_distance = v�/(2*a) = speed�/(2*a)
     * Problem is, the value of a, the deceleration rate needed is unknown. All we know is the current speed and the stick position.
     * But we know that the current speed should be zero after time_to_stop cycles, hence we know the required
     * deceleration in speed-units: a = speed/time_to_Stop
     *
     * Hence:
     * brake_distance = speed�/(2*a) = speed�/(2*speed/time_to_Stop) = speed * time_to_stop / 2
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
    int32_t pos_current = getPos();
    speed_current = abs_d((float) (pos_current_old - pos_current));
    float speed_timer = getSpeedPosSensor();
    float speed = speed_current;

    /* If the pos difference is less than 5.0f per 20ms, then the resolution is not high enough. In that
     * case use the time between to pos sensor interrupts to calculate a more precise speed.
     */
    if (speed_current <= 5.0f)
    {
        speed = speed_timer;
    }

    float time_to_stop = abs_d(getStick()/controllerstatus.stick_max_accel);

    float distance_to_stop = speed * time_to_stop / 2.0f;
    float stick_filtered_value = stickCycle(distance_to_stop); // go through the stick position calculation with its limiters, max accel etc

    /*
     * PPM output is rescaled to +-700 and adding the ESC's neutral range to get an immediate output.
     * However the controllerstatus.stick_max_speed plays a role as well. If that is a value of 50%, then the stick_filtered_value is rescaled to this.
     * Example: stick: 100%; max_speed: 50% --> esc_valuerange * 1.00 * 0.5 --> Stick of 100% means 50% output
     */
    if (stick_filtered_value > 0.0f)
    {
        TIM3->CCR3 = activesettings.esc_neutral_pos + activesettings.esc_neutral_range + ((int16_t) (stick_filtered_value * controllerstatus.stick_max_speed *((float) activesettings.esc_value_range)));
    }
    else if (stick_filtered_value < 0.0f)
    {
        TIM3->CCR3 = activesettings.esc_neutral_pos - activesettings.esc_neutral_range + ((int16_t) (stick_filtered_value * controllerstatus.stick_max_speed * ((float) activesettings.esc_value_range)));
    }
    else
    {
        TIM3->CCR3 = activesettings.esc_neutral_pos;
    }
    VESC_Output(stick_filtered_value); // controllerstatus.stick_max_speed is handled inside


    float aux = getAuxInput();
    if (aux > 0.0f)
    {
        TIM3->CCR4 = activesettings.esc_neutral_pos + activesettings.esc_neutral_range + ((int16_t) (aux * ((float) activesettings.esc_value_range)));
    }
    else if (aux < 0.0f)
    {
        TIM3->CCR4 = activesettings.esc_neutral_pos - activesettings.esc_neutral_range + ((int16_t) (aux * ((float) activesettings.esc_value_range)));
    }
    else
    {
        TIM3->CCR4 = activesettings.esc_neutral_pos;
    }

    for (int ch=0; ch<8 ; ch++)
    {
        gimbalout[ch] = getGimbalDuty(ch);
    }
    setGimbalValues(gimbalout);


    pos_current_old = pos_current; // required for the actual speed calculation

}

float getGimbalDuty(uint8_t channel)
{
    if (activesettings.rc_channel_sbus_out_mapping[channel] == 255)
    {
        return NAN;
    }
    else
    {
        return sbusdata.servovalues[activesettings.rc_channel_sbus_out_mapping[channel]].duty;
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
        if (pos + brakedistance > semipermanentsettings.pos_end)
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
        if (pos - brakedistance < semipermanentsettings.pos_start)
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
