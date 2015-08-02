#include "config.h"
#include "protocol.h"
#include "controller.h"
#include "stm32_servo_input.h"
#include "stm32_servo_output.h"
#include "debug.h"
#include "stm32_encoder.h"
#include "protocol.h"
#include "clock_50Hz.h"

void calculateQx();
void setPosStartEnd(double p1, double p2);


int16_t servo[4] = {1500,1500,1500,1500};
int16_t rcData[4] = {1500,1500,1500,1500};

int16_t esc_neutral_range_forward = 30; // overall neutral range in us, e.g. 80us means neutral from -1460...+1540
int16_t esc_neutral_range_reverse = 30; // overall neutral range in us, e.g. 80us means neutral from -1460...+1540
int16_t esc_neutral_range_center = SERVO_CENTER; // overall neutral range in us, e.g. 80us means neutral from -1460...+1540

static uint16_t number_of_idle_cycles = 0;
static double speed_old = 0;
static double yalt = 0.0f, ealt = 0.0f, ealt2 = 0.0f;

uint8_t resetthrottle = 0;

/*
 *  Putting the stick in neutral can mean two different things
 *  a) The stick was providing input and went back into the neutral range -> output a servo signal of neutral, else it would stay at the last stick position
 *  b) The stick was in neutral the entire time and the servo output set using the serial protocol
 *
 *  Hence there is a trigger:
 *  When the RC stick is used the in-neutral is set to 0. If the stick is moved back to the neutral position the servo output
 *  is set to neutral and in-neutral set to 1. As long as the stick remains in the neutral range the output will not be changed again, hence
 *  allows the serial command to change it.
 */
uint8_t in_neutral_pitch = 0;
uint8_t in_neutral_yaw = 0;
uint8_t in_neutral_aux = 0;

static CONTROLLER_STATUS_t controller_status = CONTROLLER_INIT;


static double pos_target = 0.0f, pos_current_alt = 0.0f;

double max_acceleration = MAX_ACCELERATION_INIT; // current max acceleration allowed; measured by the actual speed changes
double max_speed = MAX_SPEED_INIT; // speed limit measured by the speed sensor
double pos_start = -POS_END_NOT_SET;
double pos_end = POS_END_NOT_SET;

double rc_speed_signal_old = 0.0f;

#define Ta  0.02

double Q0, Q1, Q2;

int8_t moving_direction = 0;


void setServoNeutralRange(uint16_t reverse, uint16_t forward) {
	esc_neutral_range_center = (forward+reverse)/2;
	esc_neutral_range_forward = forward - esc_neutral_range_center;
	esc_neutral_range_reverse = esc_neutral_range_center - reverse;
}

void calculateQx() {
	Q0 = activesettings.P+activesettings.I*Ta+activesettings.D/Ta;
	Q1 = -activesettings.P-2*activesettings.D/Ta;
	Q2 = activesettings.D/Ta;
}

void setRCData(uint16_t rcsignal1, uint16_t rcsignal2, uint16_t rcsignal3, uint16_t rcsignal4) {
	rcData[SERVO_YAW] = rcsignal1;
	rcData[SERVO_PITCH] = rcsignal2;
	rcData[SERVO_AUX] = rcsignal3;
	rcData[SERVO_ESC] = rcsignal4;
}


void setServoValueForChannel(uint8_t channel, uint16_t rcsignal) {
	switch (channel) {
	case SERVO_YAW:
		servo[SERVO_YAW] = rcsignal;
		break;
	case SERVO_PITCH:
		servo[SERVO_PITCH] = rcsignal;
		break;
	case SERVO_AUX:
		servo[SERVO_AUX] = rcsignal;
		break;
	case SERVO_ESC:
		servo[SERVO_ESC] = rcsignal;
		break;
	}
}

uint16_t getRCDataForChannel(uint8_t channel) {
	switch (channel) {
	case SERVO_YAW:
		return rcData[SERVO_YAW];
		break;
	case SERVO_PITCH:
		return rcData[SERVO_PITCH];
		break;
	case SERVO_AUX:
		return rcData[SERVO_AUX];
		break;
	case SERVO_ESC:
		return rcData[SERVO_ESC];
		break;
	}
	return 0;
}

void setPIDValues(double kp, double ki, double kd) {
	activesettings.P = kp;
	activesettings.I = ki;
	activesettings.D = kd;
	calculateQx();
}

void getPIDValues(double *kp, double *ki, double *kd) {
	*kp = activesettings.P;
	*ki = activesettings.I;
	*kd = activesettings.D;
}

void setPValue(double v) {
	activesettings.P = v;
	calculateQx();
}
void setIValue(double v) {
	activesettings.I = v;
	calculateQx();
}
void setDValue(double v) {
	activesettings.D = v;
	calculateQx();
}
double getPValue() {
	return activesettings.P;
}
double getIValue() {
	return activesettings.I;
}
double getDValue() {
	return activesettings.D;
}

void setMaxAccelerationRCStick(double maxaccel) {
	activesettings.rs_speed_signal_max_acceleration = maxaccel;
}

double getMaxAccelerationRCStick() {
	return activesettings.rs_speed_signal_max_acceleration;
}

void setToleratedPositionalErrorLimit(double distance) {
	activesettings.tolerated_positional_error_limit = distance;
}

double getToleratedPositionalErrorLimit() {
	return activesettings.tolerated_positional_error_limit;
}

void setMaxAllowedPositionalError(double distance) {
	activesettings.tolerated_positional_error_exceeded = distance;
}

double getMaxAllowedPositionalError() {
	return activesettings.tolerated_positional_error_exceeded;
}

void setMaxAcceleration(double maxaccel) {
	activesettings.max_accel = maxaccel;
	if (pos_end != POS_END_NOT_SET) {
		max_acceleration = maxaccel;
	}
}

double getMaxAcceleration() {
	return activesettings.max_accel;
}

void setPosStartEnd(double p1, double p2) {
	if (p1 < p2) {
		pos_start = p1;
		pos_end = p2;
	} else {
		pos_start = p2;
		pos_end = p1;
	}
	printDebug_string("Position limits set: start_pos=", DEBUG_LEVEL_ALWAYS);
	printDebug_double(pos_start, DEBUG_LEVEL_ALWAYS);
	printDebug_string(" end_pos=", DEBUG_LEVEL_ALWAYS);
	printlnDebug_double(pos_end, DEBUG_LEVEL_ALWAYS);
	max_acceleration = activesettings.max_accel;
	max_speed = activesettings.max_speed;
	controller_status = CONTROLLER_RUN;
}

void setPosStart(double p) {
	if (pos_end == POS_END_NOT_SET) {
		setPosStartEnd(p, 0);
	} else {
		setPosStartEnd(p, pos_end);
	}
}

double getPosStart() {
	return pos_start;
}

void setPosEnd(double p) {
	if (pos_start == POS_END_NOT_SET) {
		setPosStartEnd(0, p);
	} else {
		setPosStartEnd(pos_start, p);
	}
}

double getPosEnd() {
	return pos_end;
}

void setMaxSpeed(double p) {
	activesettings.max_speed = p;
	if (pos_end != POS_END_NOT_SET) {
		max_speed = p;
	}
}

double getMaxSpeed() {
	return activesettings.max_speed;
}

void setTargetPos(double p) {
	pos_target = p;
}

double getTargetPos() {
	return pos_target;
}

void setPitch(double p) {
	servo[SERVO_PITCH] = (int16_t) p;
}

double getPitch() {
	return (double) servo[SERVO_PITCH];
}

void setYaw(double p) {
	servo[SERVO_YAW] = (int16_t) p;
}

double getYaw() {
	return (double) servo[SERVO_YAW];
}

void resetThrottle() {
	resetthrottle = 1;
}

double getThrottle() {
	return (double) servo[SERVO_ESC];
}

double getSpeed() {
	return speed_old;
}
double getPos() {
	return (double) ENCODER_ReadPos();
}

void initController() {
	calculateQx();
	setServoNeutralRange(activesettings.neutrallow, activesettings.neutralhigh);
}

double abs_d(double v) {
	if (v<0.0) {
		return -v;
	} else {
		return v;
	}
}

// ******** Main Loop *********
void controllercycle() {

	double pos_current = (double) ENCODER_ReadPos();
	double speed;
	double rc_speed_signal = (double) (rcData[SERVO_ESC] - SERVO_CENTER);

	/*
	 * Servo AUX is handled always, independent from the mode
	 */
	if (rcData[SERVO_AUX] >= SERVO_MIN && rcData[SERVO_AUX] <= SERVO_MAX) {
		double diff_aux = (double) (rcData[SERVO_AUX] - SERVO_CENTER);
		if (diff_aux < -NEUTRAL_RANGE || diff_aux > +NEUTRAL_RANGE) {     // As long as it remains in the neutral zone input values from the serial protocol take precedence, else we overwrite with rc inpput
			servo[SERVO_AUX] = rcData[SERVO_AUX];
			in_neutral_aux = 0;
		} else if (in_neutral_aux == 0) {
			servo[SERVO_AUX] = SERVO_CENTER;
			in_neutral_aux = 1;
		}
	}


	if (activesettings.mode == MODE_PASSTHROUGH) {
		/*
		 * In passthrough mode we send the input values directly to the output
		 */
		PWM_SetPWM(SERVO_1_OUT, rcData[SERVO_ESC]);
		PWM_SetPWM(SERVO_2_OUT, rcData[SERVO_PITCH]);
		PWM_SetPWM(SERVO_3_OUT, rcData[SERVO_YAW]);
		// We need that to make sure when enabling the MODE_ABSOLUTE, we start slowly
		pos_target = pos_current;
		speed = 0;
	} else {

		/*
		 * Step 1 is to check the pitch and yaw values.
		 * There are multiple cases:
		 * 1. the input is outside the allowed range, e.g. no RC connected or the input mapping is wrong
		 * 2. the stick was moved from outside neutral into neutral, hence the value should be neutral
		 * 3. the stick was in neutral all the time and the value was overwritten using the serial protocol - then we should not change anything here
		 *
		 * The neutral ranges of the stick are compensated, e.g. the stick might say
		 * rc=1521us
		 * center=1500us
		 * NEUTRAL_RANGE=20us
		 * ==> output=1501us
		 *
		 */
		if (rcData[SERVO_PITCH] >= SERVO_MIN && rcData[SERVO_PITCH] <= SERVO_MAX) { // maybe we never got an input because no receiver is connected
			double diff_pitch = (double) (rcData[SERVO_PITCH] - SERVO_CENTER);
			if (diff_pitch < -NEUTRAL_RANGE) {     // As long as it remains in the neutral zone input values from the serial protocol take precedence, else we overwrite with rc inpput
				servo[SERVO_PITCH] = rcData[SERVO_PITCH] + (int16_t)NEUTRAL_RANGE; // Pitch below the neutral zone hence we start to give signal slowly
				in_neutral_pitch = 0;
			} else if (diff_pitch > +NEUTRAL_RANGE) {     // As long as it remains in the neutral zone input values from the serial protocol take precedence, else we overwrite with rc inpput
				servo[SERVO_PITCH] = rcData[SERVO_PITCH] - (int16_t)NEUTRAL_RANGE; // Pitch above the neutral zone hence we start to give signal slowly
				in_neutral_pitch = 0;
			} else if (in_neutral_pitch == 0) {
				servo[SERVO_PITCH] = SERVO_CENTER;
				in_neutral_pitch = 1;
			}
		}

		if (rcData[SERVO_YAW] >= SERVO_MIN && rcData[SERVO_YAW] <= SERVO_MAX) { // maybe we never got an input because no receiver is connected
			double diff_yaw = (double) (rcData[SERVO_YAW] - SERVO_CENTER);
			if (diff_yaw < -NEUTRAL_RANGE) {      // As long as it remains in the neutral zone input values from the serial protocol take precedence, else we overwrite with rc inpput
				servo[SERVO_YAW] = rcData[SERVO_YAW] + (int16_t)NEUTRAL_RANGE; // Yaw below the neutral zone hence we start to give signal slowly
				in_neutral_yaw = 0;
			} else if (diff_yaw > +NEUTRAL_RANGE) {      // As long as it remains in the neutral zone input values from the serial protocol take precedence, else we overwrite with rc inpput
				servo[SERVO_YAW] = rcData[SERVO_YAW] - (int16_t)NEUTRAL_RANGE; // Yaw above the neutral zone hence we start to give signal slowly
				in_neutral_yaw = 0;
			} else if (in_neutral_yaw == 0) {
				servo[SERVO_YAW] = SERVO_CENTER;
				in_neutral_yaw = 1;
			}
		}

		/*
		 * ESC speed is similar with the additional requirement that the stick has to be in neutral at start, else we stay in neutral
		 */
		if (rcData[SERVO_ESC] >= SERVO_MIN && rcData[SERVO_ESC] <= SERVO_MAX) {
			if (rc_speed_signal < NEUTRAL_RANGE && rc_speed_signal > -NEUTRAL_RANGE) { // limit the noise -> everything around the zero point is zero as requested speed
				rc_speed_signal = 0.0;
				if (controller_status == CONTROLLER_INIT) {
					controller_status = CONTROLLER_FIND_END_POS;
				}
			} else if (controller_status == CONTROLLER_INIT ) {
				if (is1Hz()) {
					printDebug_string("We have a speed signal at start! ", DEBUG_LEVEL_ALWAYS);
					printlnDebug_double(rc_speed_signal, DEBUG_LEVEL_ALWAYS);
				}
				rc_speed_signal = 0.0;
			} else if (rc_speed_signal >= NEUTRAL_RANGE) {
				rc_speed_signal -= NEUTRAL_RANGE; // speed signal was e.g. 55ms, we consider +-20ms the NEUTRAL_RANGE hence the stick has been 35ms above neutral
			} else if (rc_speed_signal <= -NEUTRAL_RANGE) {
				rc_speed_signal += NEUTRAL_RANGE;
			}
		} else {
			rc_speed_signal = 0.0;
		}



		/*
		 * The speed is the number of sensor signals per cycle. In MODE_ABSOLUTE_POSITION we define the target speed by increasing the target position.
		 * But in MODE_LIMITER the RC speed signal sets the ESC signal, there is no speed as such. All we know is the current speed by checking the current position.
		 */
		if (activesettings.mode == MODE_LIMITER || activesettings.mode == MODE_LIMITER_ENDPOINTS) {
			speed = pos_current - pos_current_alt;
			pos_target = pos_current;
			if (activesettings.mode == MODE_LIMITER) {
				/*
				 * In this mode some hacking takes place. Imagine the motor stands still. Hence the measured speed from the sensor is zero and
				 * the speed signal from the rc stick is zero.
				 * Now the user cranks the speed signal to the max at once. The ESC starts to accelerate the motor as quickly it can, resulting in a higher
				 * acceleration than allowed. Therefore the speed signal needs to be reduced but to what value? In the absolute mode the error is constantly
				 * calculated and the PID loop tries to adjust the speed signal. But in this mode there is no error value.
				 * Hence two limiters are combined. First the speed signal is not allowed to change abruptly, it is limited to up
				 * to rs_speed_signal_max_acceleration. This allows the ESC to reach terminal velocity for the current speed signal. Now, if that
				 * terminal velocity is too high for the acceleration limiter based on the actual speed, the speed signal is kept at that level for another cycle.
				 * Since there is a relationship between those two acceleration limiters, setting the values needs some care. Best would be a low rc speed signal
				 * acceleration so that the actual acceleration limiter never kicks in. The more aggressive the speed signal is allowed to be changed, the less precise
				 * the actual acceleration can be controlled.
				 */


				// For slow accelerations in this mode, the speed signal has its own acceleration limiter
				if (rc_speed_signal > rc_speed_signal_old + activesettings.rs_speed_signal_max_acceleration) {
					rc_speed_signal = rc_speed_signal_old + activesettings.rs_speed_signal_max_acceleration;
				} else if (rc_speed_signal < rc_speed_signal_old - activesettings.rs_speed_signal_max_acceleration) {
					rc_speed_signal = rc_speed_signal_old - activesettings.rs_speed_signal_max_acceleration;
				}

				// In mode limiter all we can do is keeping the speed signal at the same level as in the previous cycle in case we are too fast
				// we have a speed limiter
				if (speed > max_speed && rc_speed_signal > rc_speed_signal_old) {
					rc_speed_signal = rc_speed_signal_old;
				}
				if (speed < -max_speed && rc_speed_signal < rc_speed_signal_old) {
					rc_speed_signal = rc_speed_signal_old;
				}

				// and an acceleration limiter
				if ((speed - speed_old) > max_acceleration && rc_speed_signal > rc_speed_signal_old) {
					rc_speed_signal = rc_speed_signal_old;
				} else if ((speed - speed_old) < -max_acceleration && rc_speed_signal < rc_speed_signal_old) {
					rc_speed_signal = rc_speed_signal_old;
				}
			}

			/*
			 * One thing might happen here: You drive directly into the end point but the speed sensor got disconnected. Hence speed remains zero and
			 * controller believes it is standing still. In this case we do an emergency brake.
			 */
			if (speed == 0.0 && (rc_speed_signal > 200.0 || rc_speed_signal < -200.0)) {
				rc_speed_signal = 0.0;
				if (is5s()) {
					printlnDebug_string("ERMEGENCY: speed=0 and rc signal is high - there is something severly wrong", DEBUG_LEVEL_VERYHIGH);
				}
			}
		} else if (activesettings.mode == MODE_ABSOLUTE_POSITION) {
			speed = rc_speed_signal / 5.0;
			// we have a speed limiter
			if (speed > max_speed) {
				speed = max_speed;
			}
			if (speed < -max_speed) {
				speed = -max_speed;
			}

			// and an acceleration limiter
			if ((speed - speed_old) > max_acceleration) {
				speed = speed_old + max_acceleration;
			} else if ((speed - speed_old) < -max_acceleration) {
				speed = speed_old - max_acceleration;
			}
		} else {
			// Impossible to reach this code but...
			speed = 0.0;
			pos_target = pos_current;
		}


		pos_target += speed;

		double distance_to_stop = abs_d(speed_old) * (abs_d(speed_old)-max_acceleration) / 2.0f / max_acceleration;
		/*
		   We need to work with the average speed, and that is the motor speed and the new speed at max deceleration

		   speed_old	accel	new_speed	distance_to_stop		old_pos_target	    new_pos_target	old_pos_target+distance_to_stop
				 100	1		99	        4950	                  0	                 99	     	    4950
				  99	1		98	        4851	                 99	                197	     	    4950
				  98	1		97	        4753	                197	                294	      	    4950
				  97	1		96	        4656	                294	                390	      	    4950
				  96	1		95	        4560	                390	                485	      	    4950
				  95	1		94	        4465	                485	                579	     	    4950
				  94	1		93	        4371	                579	                672	     	    4950
				  93	1		92	        4278	                672	                764				4950
				  92	1		91	        4186	                764	                855				4950
				  91	1		90	        4095	                855	                945				4950
				  ..
				  ..
				  ..
		 */

		if (pos_end == POS_END_NOT_SET) {
			if (rc_speed_signal == 0.0 || speed == 0.0) { // either the stick is in neutral or the cablecam does not move by itself
				if (pos_target > 1000.0 || pos_target < -1000.0) {
					if (number_of_idle_cycles > 500) {
						// initially the end_pos is not set (very high) but if the stick was in neutral pos for 10 seconds and we traveled at least 1000 steps forward, then we take this position as final end pos
						setPosStartEnd(pos_target, 0.0);
					} else if (is1Hz()) {
						printDebug_string("If we stay there 10 seconds we have an end pos: pos=", DEBUG_LEVEL_ALWAYS);
						printDebug_double(pos_target, DEBUG_LEVEL_ALWAYS);
						printDebug_string(" wait time=", DEBUG_LEVEL_ALWAYS);
						printlnDebug_int((int) (number_of_idle_cycles/50), DEBUG_LEVEL_ALWAYS);
					}
				} else if (pos_target != 0.0) {
					if (is1Hz()) {
						printDebug_string("This is too close to the start position for being a valid end position: pos=", DEBUG_LEVEL_VERYLOW);
						printlnDebug_double(pos_target, DEBUG_LEVEL_VERYLOW);
					}
				}
				number_of_idle_cycles++;
			} else {
				number_of_idle_cycles = 0;
			}
		} else {
			if (pos_target + distance_to_stop >= pos_end && rc_speed_signal >= 0.0) {
				speed = speed_old - max_acceleration;
				if (speed < 0.0) speed = 0.0; // do brake but do not drive reverse
				rc_speed_signal = 0.0; // in MODE_LIMITER activate the brakes
			} else if (pos_target - distance_to_stop <= pos_start && rc_speed_signal <= 0.0) {
				speed = speed_old + max_acceleration;
				if (speed > 0.0) speed = 0.0;
				rc_speed_signal = 0.0; // in MODE_LIMITER activate the brakes
			}
			if (pos_target > pos_end) {
				pos_target = pos_end;
				if (rc_speed_signal > 0.0) {
					rc_speed_signal = 0.0;
				}
			} else if (pos_target < pos_start) {
				pos_target = pos_start;
				if (rc_speed_signal < 0.0) {
					rc_speed_signal = 0.0;
				}
			}
		}

		if (activesettings.mode == MODE_LIMITER || activesettings.mode == MODE_LIMITER_ENDPOINTS) {
			int16_t s = SERVO_CENTER + (int16_t) (rc_speed_signal);
			/* if (rc_speed_signal < 0.0f) {
				s = (int16_t) (rc_speed_signal - NEUTRAL_RANGE);
			} else if (rc_speed_signal > 0.0f) {
				s = (int16_t) (rc_speed_signal + NEUTRAL_RANGE);
			} else {
				s = 0;
			}
			s += SERVO_CENTER; */
			PWM_SetPWM(SERVO_1_OUT, s);

			if (is5s() && activesettings.debuglevel >= DEBUG_LEVEL_LOW) {
				printDebug_string("target pos=", DEBUG_LEVEL_LOW);
				printDebug_double(pos_target, DEBUG_LEVEL_LOW);
				printDebug_string(" current pos=", DEBUG_LEVEL_LOW);
				printDebug_double(pos_current, DEBUG_LEVEL_LOW);

				printDebug_string("    speed=", DEBUG_LEVEL_LOW);
				printDebug_double(speed, DEBUG_LEVEL_LOW);
				printDebug_string(" rc_out=", DEBUG_LEVEL_LOW);
				printlnDebug_int(s, DEBUG_LEVEL_LOW);
			}
		} else {
			/* The target position calculation does not care about the physics, all it said was "In an ideal world the target pos should be here,
				 the requested (filtered with max speed and max accel and end point limiters) speed is that, hence the new target pos = target pos + speed."
				 Example:
				   target_pos = 0m;
				   speed = 0m/s
				   speed as provided by the stick is 400/10 = 40m/s
				   --> kick in the accel limiter, 40m/s² is way too high, limited accel to 2m/s²
				   new_speed = 0m/s + 1s*2m/s² = 2m/s
				   target_pos = target_pos + 1s * 2m/s = 0m + 2m

				 In the code below is the PID logic to keep the motor following the target position value.
			 */
			double e, y = 0.0f;


			e = pos_target - pos_current;     // This is the amount of steps the target pos does not match the reality

		    // let's not be overly precise. If the error is below a threshold we are good enough
			// Not sure that is a good idea though, as it turns the control loop function into an even more unsteady curve around neutral.
			if (e >= -activesettings.tolerated_positional_error_limit && e <= activesettings.tolerated_positional_error_limit) {
				e = 0.0;
			}

			y = yalt + Q0*e + Q1*ealt + Q2*ealt2;         // PID loop calculation

			if (is5s() && activesettings.debuglevel >= DEBUG_LEVEL_VERYLOW) {
				printDebug_string("target pos=", DEBUG_LEVEL_VERYLOW);
				printDebug_double(pos_target, DEBUG_LEVEL_VERYLOW);
				printDebug_string(" current pos=", DEBUG_LEVEL_VERYLOW);
				printDebug_double(pos_current, DEBUG_LEVEL_VERYLOW);

				printDebug_string("    esc=", DEBUG_LEVEL_VERYLOW);
				printDebug_double(y, DEBUG_LEVEL_VERYLOW);
				printDebug_string("=", DEBUG_LEVEL_VERYLOW);
				printDebug_double(yalt, DEBUG_LEVEL_VERYLOW);
				printDebug_string(" + ", DEBUG_LEVEL_VERYLOW);
				printDebug_double(Q0, DEBUG_LEVEL_VERYLOW);
				printDebug_string("*", DEBUG_LEVEL_VERYLOW);
				printDebug_double(e, DEBUG_LEVEL_VERYLOW);
				printDebug_string(" + ", DEBUG_LEVEL_VERYLOW);
				printDebug_double(Q1, DEBUG_LEVEL_VERYLOW);
				printDebug_string("*", DEBUG_LEVEL_VERYLOW);
				printDebug_double(ealt, DEBUG_LEVEL_VERYLOW);
				printDebug_string(" + ", DEBUG_LEVEL_VERYLOW);
				printDebug_double(Q2, DEBUG_LEVEL_VERYLOW);
				printDebug_string("*", DEBUG_LEVEL_VERYLOW);
				printlnDebug_double(ealt2, DEBUG_LEVEL_VERYLOW);
			}



			// remove the ESC neutral range
			int16_t esc_speed = (int16_t) y;

			if (e > activesettings.tolerated_positional_error_exceeded || e < -activesettings.tolerated_positional_error_exceeded) {
				// EMERGENCY there is something severely wrong, variable overflow, hall sensor counter, hall sensor direction, anything...
				// hence we stop. User can change the mode to passthrough to move the cablecam
				if (is5s()) {
					printDebug_string("ERMEGENCY: target pos=", DEBUG_LEVEL_VERYHIGH);
					printDebug_double(pos_target, DEBUG_LEVEL_VERYHIGH);
					printDebug_string(" current pos=", DEBUG_LEVEL_VERYHIGH);
					printlnDebug_double(pos_current, DEBUG_LEVEL_VERYHIGH);
				}
				resetThrottle();

			} else {
				servo[SERVO_ESC] = esc_neutral_range_center + (esc_speed * activesettings.esc_direction);
				if (esc_speed>0) {
					servo[SERVO_ESC]+=(esc_neutral_range_forward * activesettings.esc_direction); // a speed request of +1us means +40+1 as the ESC starts at +40us only -> ESC's neutral range
					if (is5s() && activesettings.debuglevel >= DEBUG_LEVEL_LOW) {
						printDebug_string(" dist=", DEBUG_LEVEL_LOW);
						printDebug_double(e, DEBUG_LEVEL_LOW);
						printDebug_string("target pos=", DEBUG_LEVEL_LOW);
						printDebug_double(pos_target, DEBUG_LEVEL_LOW);
						printDebug_string("current pos=", DEBUG_LEVEL_LOW);
						printDebug_double(pos_current, DEBUG_LEVEL_LOW);
						printDebug_string(" old pos=", DEBUG_LEVEL_LOW);
						printDebug_double(pos_current_alt, DEBUG_LEVEL_LOW);
						printDebug_string("esc=", DEBUG_LEVEL_LOW);
						printlnDebug_int(servo[SERVO_ESC], DEBUG_LEVEL_LOW);
					}
				} else if (esc_speed<0) {
					servo[SERVO_ESC]-= (esc_neutral_range_reverse * activesettings.esc_direction);
					if (is5s() && activesettings.debuglevel >= DEBUG_LEVEL_LOW) {
						printDebug_string(" dist=", DEBUG_LEVEL_HIGH);
						printDebug_double(e, DEBUG_LEVEL_LOW);
						printDebug_string("target pos=", DEBUG_LEVEL_LOW);
						printDebug_double(pos_target, DEBUG_LEVEL_LOW);
						printDebug_string("current pos=", DEBUG_LEVEL_LOW);
						printDebug_double(pos_current, DEBUG_LEVEL_LOW);
						printDebug_string(" old pos=", DEBUG_LEVEL_LOW);
						printDebug_double(pos_current_alt, DEBUG_LEVEL_LOW);
						printDebug_string("esc=", DEBUG_LEVEL_LOW);
						printlnDebug_int(servo[SERVO_ESC], DEBUG_LEVEL_LOW);
					}
				}
				if (servo[SERVO_ESC] < SERVO_MIN) servo[SERVO_ESC] = SERVO_MIN;
				if (servo[SERVO_ESC] > SERVO_MAX) servo[SERVO_ESC] = SERVO_MAX;
			}

			PWM_SetPWM(SERVO_1_OUT, servo[SERVO_ESC]);
			if (resetthrottle == 1) {
				y = 0.0f;
				yalt = 0.0f;
				e = 0.0f;
				ealt = 0.0f;
				ealt2 = 0.0f;
				resetthrottle = 0;
			} else {
				ealt2 = ealt;
				ealt = e;
				yalt = y;
			}
		}
		PWM_SetPWM(SERVO_2_OUT, servo[SERVO_PITCH]);
		PWM_SetPWM(SERVO_3_OUT, servo[SERVO_YAW]);
		PWM_SetPWM(SERVO_5_OUT, servo[SERVO_AUX]);
	}
	rc_speed_signal_old = rc_speed_signal;
	pos_current_alt = pos_current;
	speed_old = speed;
}

