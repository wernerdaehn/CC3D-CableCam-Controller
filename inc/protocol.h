#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include "serial_print.h"
#include "controller.h"

#define PROTOCOL_P                '1'   // 1 float arguments for Kp
#define PROTOCOL_I                '2'   // 1 float arguments for Ki
#define PROTOCOL_D                '3'   // 1 float arguments for Kd
#define PROTOCOL_MAX_ACCEL        'a'   // 1 float argument
#define PROTOCOL_PID       		  'c'	// PIDs set 3 floats
#define PROTOCOL_SPEED_FACTOR     'f'	// Define Speed Factor, the conversion from RC Stick value to Speed based on Hall Encoder
#define PROTOCOL_MAX_ERROR_DIST   'g'   // 1 float argument
#define PROTOCOL_HELP		      'h'	// help
#define PROTOCOL_INPUT_CHANNELS   'i'   // 3 int arguments for speed, pitch, yaw, aux
#define PROTOCOL_MODE             'm'
#define PROTOCOL_NEUTRAL          'n'	// 2 int low microseconds, high microseconds
#define PROTOCOL_POS              'p'
#define PROTOCOL_ROTATION_DIR     'r'   // 1 int argument
#define PROTOCOL_SETTINGS         'S'   // no argument
#define PROTOCOL_EEPROM_WRITE     'w'   // no argument
#define PROTOCOL_MAX_SPEED        'v'   // 1 float argument

#define MODE_ABSOLUTE_POSITION	0
#define MODE_LIMITER			1
#define MODE_PASSTHROUGH		2
#define MODE_LIMITER_ENDPOINTS	3
#define MODE_LIMITER_HOLD		4

#define POS_END_NOT_SET         0x7FFFFFFF


typedef struct {
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
} settings_t;

extern settings_t activesettings;


void initProtocol(void);
void serialCom(char * buf, Endpoints endpoint);
void printHelp(Endpoints endpoint);
void printActiveSettings(Endpoints endpoint);
SAFE_MODE_t getSafeMode(void);

#endif /* PROTOCOL_H_ */
