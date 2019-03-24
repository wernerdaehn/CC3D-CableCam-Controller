#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include "serial_print.h"
#include "controller.h"
#include "stdbool.h"
#include "sbus.h"
#include "odrive.h"

#define PROTOCOL_MAX_ACCEL        'a'   // 1 float argument
#define PROTOCOL_AUX_NEUTRAL      'A'	// 3 int neutral microseconds, +-neutral range microseconds, +- 100% throttle
#define PROTOCOL_BINARY           'b'   // Hidden command to print the binary active settings or play them back (Useful to quickly transfer settings)
#define PROTOCOL_BLUETOOTH        'B'   // no argument
#define PROTOCOL_VESC_BRAKE       'c'
#define PROTOCOL_ESC              'C'   // 1 int argument, see ESC_ defines
#define PROTOCOL_DEBUG            'd'   // one char to identify what debug to print out
#define PROTOCOL_INPUT_GIMBAL_DEFAULT     'D'   // up to 8 int arguments, the gimbal channel default output via SBus TX
#define PROTOCOL_ESC_MAX_SPEED    'e'   // 1 int argument, the maximum eRPM value set in the VESC. Goal is that 100% throttle = this eRPM value
#define PROTOCOL_ESC_STATUS      'E'   // no argument
#define PROTOCOL_MAX_ERROR_DIST   'g'   // 1 float argument
#define PROTOCOL_INPUT_GIMBAL     'G'   // up to 8 int arguments, the gimbal channel assignments used for output via SBus TX
#define PROTOCOL_HELP		      'h'	// help
#define PROTOCOL_INPUT_CHANNELS   'i'   // 3-5 int arguments for speed, command switch, end point button, max acceleration poti, may speed poti
#define PROTOCOL_INPUT_SOURCE     'I'   // 1 int arguments for the input, SumPPM or SBus
#define PROTOCOL_INPUT_SINGLE     'j'   // 1 char and 1 int argument, the channel assignment for individual functions
#define PROTOCOL_MODE             'm'
#define PROTOCOL_NEUTRAL          'n'	// 3 int neutral microseconds, +-neutral range microseconds, +- 100% throttle
#define PROTOCOL_ESC_NEUTRAL      'N'	// 3 int neutral microseconds, +-neutral range microseconds, +- 100% throttle
#define PROTOCOL_OFFSET_ENDPOINT  'o'   // 1 float argument
#define PROTOCOL_POS              'p'
#define PROTOCOL_PLAY             'P'   // 1 int argument, play=1 or don't play
#define PROTOCOL_ROTATION_DIR     'r'   // 1 int argument
#define PROTOCOL_REVERSE_WAIT     'R'   // 1 int argument
#define PROTOCOL_SETTINGS         'S'   // no argument
#define PROTOCOL_EEPROM_WRITE     'w'   // no argument
#define PROTOCOL_EEPROM_VOID      'W'   // no argument
#define PROTOCOL_MAX_SPEED        'v'   // 1 float argument
#define PROTOCOL_VERSION          'V'   // no argument
#define PROTOCOL_EXPO_FACTOR      'x'   // 1 float argument
#define PROTOCOL_POS_SOURCE       'Z'   // 1 int argument
#define PROTOCOL_SETUP            '1'
#define PROTOCOL_READ_ERRORHANDLER 'H'  // Error handler hidden command
#define PROTOCOL_BOOT             '_'   // Boot to Dfu Bootloader

#define MODE_PASSTHROUGH		1
#define MODE_LIMITER			2
#define MODE_LIMITER_ENDPOINTS	3

#define POS_END_NOT_SET         0x7FFFFFFF

#define RECEIVER_TYPE_SUMPPM    0
#define RECEIVER_TYPE_SBUS      1
#define RECEIVER_TYPE_SERVO     2

#define ESC_RC_ONLY             0
#define ESC_VESC                2
#define ESC_ODRIVE              3

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
	NOT_NEUTRAL_AT_STARTUP,     // valid readings but the position is not in neutral - print the problem and wait
	DISABLE_RC
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
 /*
typedef enum {
	FREE = 0,
	EMERGENCYBRAKE,
	ENDPOINTBRAKE
} CONTROLLER_MONITOR_t; */

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
    uint8_t debuglevel;
    float esc_direction;
    int16_t stick_neutral_pos;
    int16_t stick_neutral_range;
    int16_t stick_value_range;
    uint8_t rc_channel_speed;
    uint8_t rc_channel_programming;
    uint8_t rc_channel_endpoint;
    uint8_t rc_channel_max_accel;
    uint8_t rc_channel_max_speed;
    uint8_t rc_channel_mode;
    uint8_t mode;
    float max_position_error;
    float ignore1;
    float ignore2;
    uint8_t receivertype;
    int16_t esc_neutral_pos;
    int16_t esc_neutral_range;
    int16_t esc_value_range;
    int32_t esc_max_speed;
    float expo_factor;
    float stick_max_accel;
    float stick_max_speed;
    float stick_max_accel_safemode;
    float stick_max_speed_safemode;
    int16_t vesc_brake_handbrake_min;
    int16_t vesc_brake_handbrake_max;
    int16_t vesc_brake_min_speed;
    uint8_t rc_channel_aux;
    uint8_t rc_channel_yaw;
    uint8_t rc_channel_pitch;
    uint8_t rc_channel_roll;
    uint8_t rc_channel_play;
    uint8_t rc_channel_sbus_out_mapping[8];
    uint16_t structure_length;
    uint8_t pos_source;
    uint8_t esc_type;
    int16_t aux_neutral_pos;
    int16_t aux_neutral_range;
    int16_t aux_value_range;
    uint8_t noop_padding_2[2]; // Needed to align the structure to 32bit
    uint16_t rc_channel_sbus_out_default[8];
    uint32_t play_reverse_waiting_time;
    float offset_endpoint;
} settings_t;


extern settings_t activesettings;


typedef struct
{
    char version[11];
    float pos_start;
    float pos_end;
    uint16_t structure_length;
} settings2_t;

extern settings2_t semipermanentsettings;


/** \brief All status and monitoring info is set in this structure to keep them together
 *
 * Without this structure the code would have many global variables making it harder to understand the
 * dependencies. Hence there is a single global variable "controllerstatus" with the various fields for
 * the various aspects of states.
 */
typedef struct
{
    char boottext_eeprom[81];
    Endpoints debugrequester;
    SAFE_MODE_t safemode;
    int16_t cyclemonitor_position;
    bool accel_limiter;
    bool speed_limiter;
    bool endpointbrake;
    bool emergencybrake;
    PLAY_RUNNING_t play_running;
    int8_t play_direction;
    uint32_t play_time_lastsignal; // Failsafe. If the play signal is not received for a given amount of time, stop playing the program. RC or app seems to be disconnected.
    uint32_t play_endpoint_reached_at; // Used to calculate the pause before running into the other direction
    float stick_max_accel;
    float stick_max_speed;
    bool bluetooth_passthrough;
    bool debug_endpoint;
    float pos;
    float pos_diff;
    uint32_t last_pos_change;
    float speed_by_posdiff;
    float speed_by_time;
    uint32_t duart_pause[7];
    uint32_t duart_valid[7];
    uint32_t duart_last_data[7];
    uint32_t duart_errors[7];
    uint32_t duart_frame_errors[7];
    uint32_t duart_parity_errors[7];
    uint32_t duart_noise_errors[7];
    uint32_t duart_overrun_errors[7];
    uint32_t duart_dma_errors[7];
    uint32_t duart_last_packet_length[7];
    uint32_t dvesc_packetlength_error;
    uint32_t dvesc_crc_error;
    uint32_t dvesc_startbyte_error;
    uint32_t dvesc_endbyte_error;
    uint32_t tick_enter;
    uint32_t tick_leave;
    uint32_t tick_enter_previous;
    uint32_t possensorduration;
    uint32_t last_possensortick;
    int32_t esc_max_speed;
    ODRIVE_STATE_t odrivestate;
} controllerstatus_t;

extern controllerstatus_t controllerstatus;

void initProtocol(void);
void serialCom(Endpoints endpoint, char commandlinebuffer[]);
void printHelp(Endpoints endpoint);
void printActiveSettings(Endpoints endpoint);

const char * getSafeModeLabel(void);
const char * getCurrentModeLabel(uint8_t mode);


#endif /* PROTOCOL_H_ */
