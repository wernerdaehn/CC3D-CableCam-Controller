#ifndef CONFIG_H_
#define CONFIG_H_

#define ENCODER_VALUE TIM5->CNT

#define MAX_ACCELERATION_MOVEMENT    0.2f
#define MAX_SPEED_MOVEMENT         1000.0f
#define MAX_ACCELERATION_INIT        0.05f
#define MAX_SPEED_INIT              4.0f
#define SERVO_CENTER      1500
#define SERVO_MIN          900
#define SERVO_MAX         2100
#define NEUTRAL_RANGE     20.0f

#define SERVO_ESC    0
#define SERVO_PITCH  1
#define SERVO_YAW    2
#define SERVO_AUX    3


#define RC_INPUT_ESC	0
#define RC_INPUT_YAW	1
#define RC_INPUT_PITCH	2
#define RC_INPUT_AUX	3

#endif /* CONFIG_H_ */
