#ifndef PROTOCOL_H_
#define PROTOCOL_H_

#include "serial_print.h"

#define PROTOCOL_P                '1'   // 1 float arguments for Kp
#define PROTOCOL_I                '2'   // 1 float arguments for Ki
#define PROTOCOL_D                '3'   // 1 float arguments for Kd
#define PROTOCOL_AUX_VALUE        '5'   // 1 int argument for AUX output
#define PROTOCOL_MAX_ACCEL        'a'   // 1 float argument
#define PROTOCOL_MAX_STICK_ACCEL  'A'   // 1 float argument
#define PROTOCOL_START_POS        'b'   // 1 float argument
#define PROTOCOL_BLUETOOTH        'B'   // no argument
#define PROTOCOL_PID       		  'c'	// PIDs set 3 floats
#define PROTOCOL_DEBUG_LEVEL      'd'   // 1 int
#define PROTOCOL_END_POS          'e'   // 1 float argument
#define PROTOCOL_MIN_ERROR_DIST   'f'   // 1 float argument
#define PROTOCOL_MAX_ERROR_DIST   'g'   // 1 float argument
#define PROTOCOL_HELP		      'h'	// help
#define PROTOCOL_INPUT_CHANNELS   'i'   // 4 int arguments for speed, pitch, yaw, aux
#define PROTOCOL_POS              'l'
#define PROTOCOL_MODE             'm'
#define PROTOCOL_NEUTRAL          'n'	// 2 int low microseconds, high microseconds
#define PROTOCOL_PITCH            'p'   // 1 float argument
#define PROTOCOL_ROTATION_DIR     'r'   // 1 int argument
#define PROTOCOL_MAX_SPEED        's'   // 1 float argument
#define PROTOCOL_SETTINGS         'S'   // no argument
#define PROTOCOL_TARGET_POS       't'   // 1 float argument
#define PROTOCOL_SPEED            'v'
#define PROTOCOL_EEPROM_WRITE     'w'   // no argument
#define PROTOCOL_YAW              'y'   // 1 float argument
#define PROTOCOL_TIMING           '8'   // no argument
#define PROTOCOL_INPUT            '9'   // no argument

#define POS_END_NOT_SET  99999.0f

#define MODE_PASSTHROUGH		2
#define MODE_LIMITER			1
#define MODE_ABSOLUTE_POSITION	0
#define MODE_LIMITER_ENDPOINTS	3
#define MODE_LIMITER_HOLD		4



typedef struct {
  char version[11];
  double P;
  double I;
  double D;
  double max_accel;
  double max_speed;
  uint8_t debuglevel;
  int8_t esc_direction;
  uint16_t neutrallow;
  uint16_t neutralhigh;
  uint8_t speed_rc_channel;
  uint8_t yaw_rc_channel;
  uint8_t pitch_rc_channel;
  uint8_t aux_rc_channel;
  uint8_t mode;
  // end of Version 2015.01.01
  double rs_speed_signal_max_acceleration;
  double tolerated_positional_error_limit;
  double tolerated_positional_error_exceeded;
  uint16_t auxvalue;

} settings_t;

extern settings_t defaultsettings;
extern settings_t activesettings;


void initProtocol(void);
void serialCom(char * buf, Endpoints endpoint);
void printHelp(Endpoints endpoint);
void printActiveSettings(Endpoints endpoint);


#endif /* PROTOCOL_H_ */
