#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include "serial_print.h"
#include "controller.h"

#define PROTOCOL_P                '1'   // 1 float arguments for Kp
#define PROTOCOL_I                '2'   // 1 float arguments for Ki
#define PROTOCOL_D                '3'   // 1 float arguments for Kd
#define PROTOCOL_MAX_ACCEL        'a'   // 1 float argument
#define PROTOCOL_PID       		  'c'	// PIDs set 3 floats
#define PROTOCOL_SPEED_FACTOR     'f'	// Define Speed Factor, the conversion from RC Stick value to Speed based on Hall Encoder, used in positional mode only
#define PROTOCOL_MAX_ERROR_DIST   'g'   // 1 float argument
#define PROTOCOL_HELP		      'h'	// help
#define PROTOCOL_INPUT_CHANNELS   'i'   // 3-5 int arguments for speed, command switch, end point button, max acceleration poti, may speed poti
#define PROTOCOL_INPUT_SOURCE     'I'   // 1 int arguments for the input, SumPPM or SBus
#define PROTOCOL_MODE             'm'
#define PROTOCOL_NEUTRAL          'n'	// 2 int neutral microseconds, +-range microseconds
#define PROTOCOL_ESC_NEUTRAL      'N'	// 2 int neutral microseconds, +-range microseconds
#define PROTOCOL_POS              'p'
#define PROTOCOL_ROTATION_DIR     'r'   // 1 int argument
#define PROTOCOL_SETTINGS         'S'   // no argument
#define PROTOCOL_EEPROM_WRITE     'w'   // no argument
#define PROTOCOL_MAX_SPEED        'v'   // 1 float argument
#define PROTOCOL_D_CYCLES         'z'   // Hidden command to print the debug information about the values for each cycle

#define MODE_ABSOLUTE_POSITION	0
#define MODE_PASSTHROUGH		1
#define MODE_LIMITER			2
#define MODE_LIMITER_ENDPOINTS	3
#define MODE_LIMITER_HOLD		4

#define POS_END_NOT_SET         0x7FFFFFFF

#define RECEIVER_TYPE_SUMPPM    0
#define RECEIVER_TYPE_SBUS      1
#define RECEIVER_TYPE_SERVO      2

#define CYCLEMONITOR_SAMPLE_COUNT 1024

/** \brief Controller-Mode State Machine
 *
 * The controller has multiple states, mostly during boot time.
 * Initially it is set to INVALID_RC, meaning no RC signal was ever read.
 * Once a signal was received, the state changes to NOT_NEUTRAL_AT_STARTUP in case
 * the stick is not in neutral. This would usually indicate a misconfiguration, wrong stick, wrong receiver, wrong neutral point etc.
 * The mode remains at NOT_NEUTRAL_AT_STARTUP until the stick is brought into the neutral range, then
 * the mode is OPERATIONAL.
 * Using the programming switch on the RC sender (see $i command) the user toggles between OPERATIONAL and PROGRAMMING.
 */
typedef enum {
	OPERATIONAL = 0,  			// we can move as freely as we want, limiters enabled
	PROGRAMMING,		        // reduced speed and ready to set end points via the RC
	INVALID_RC,					// the starting point, when either no RC signal had been received at all or there is a speed != 0 at the beginning
	NOT_NEUTRAL_AT_STARTUP      // valid readings but the position is not in neutral - print the problem and wait
} SAFE_MODE_t;

/** \brief Monitoring info how the cablecam can be moved currently
 *
 * Normally the CableCam can be moved FREEly, meaning the stick fully controls the cable cam.
 * If the CableCam-mode is one where the endpoint switches are active and there is the danger to
 * overshoot the endpoints, the program does engage the brakes and this monitor reflects the state
 * by setting it to ENDPOINTBRAKE.
 * In case of an emergency, signal loss, CableCam runs away, software fault,... the ESC is set to neutral
 * as last operation and the monitor state is EMERGENCYBRAKE.
 */
typedef enum {
	FREE = 0,
	EMERGENCYBRAKE,
	ENDPOINTBRAKE
} CONTROLLER_MONITOR_t;

/** \brief Structure holding all permanent settings
 *
 * This structure is mem-copied to EEPROM and read from there at write (command $w)
 * and to set the defaults at boot time.
 * In order to preserve the values across firmware versions, it is paramount to
 * add new fields at the end of the structure and to never remove a field.
 * Also in the main.c the default values have to be set at two places.
 */
typedef struct
{
    char version[11];
    double P;
    double I;
    double D;
    uint8_t debuglevel;
    int8_t esc_direction;
    int16_t stick_neutral_pos;
    int16_t stick_neutral_range;
    int16_t stick_max_accel;
    int16_t stick_max_speed;
    int16_t stick_max_accel_safemode;
    int16_t stick_max_speed_safemode;
    uint8_t rc_channel_speed;
    uint8_t rc_channel_programming;
    uint8_t rc_channel_endpoint;
    uint8_t mode;
    double max_position_error;
    double pos_start;
    double pos_end;
    double stick_speed_factor;
    uint8_t receivertype;
    int16_t esc_neutral_pos;
    int16_t esc_neutral_range;
    int16_t esc_scale;
    uint8_t rc_channel_max_accel;
    uint8_t rc_channel_max_speed;
} settings_t;


typedef struct
{
    double pos;
    int16_t stick;
    double distance_to_stop;
    double speed;
    uint16_t esc;
    uint32_t tick;
} cyclemonitor_t;


extern settings_t activesettings;

/** \brief All status and monitoring info is set in this structure to keep them together
 *
 * Without this structure the code would have many global variables making it harder to understand the
 * dependencies. Hence there is a single global variable "controllerstatus" with the various fields for
 * the various aspects of states.
 */
typedef struct
{
    char boottext_eeprom[81];
    SAFE_MODE_t safemode;
    CONTROLLER_MONITOR_t monitor;
    cyclemonitor_t cyclemonitor[CYCLEMONITOR_SAMPLE_COUNT];
    int16_t cyclemonitor_position;
} controllerstatus_t;

extern controllerstatus_t controllerstatus;

void initProtocol(void);
void serialCom(Endpoints endpoint);
void printHelp(Endpoints endpoint);
void printActiveSettings(Endpoints endpoint);

char * getSafeModeLabel();
char * getCurrentModeLabel(uint8_t mode);


#endif /* PROTOCOL_H_ */
