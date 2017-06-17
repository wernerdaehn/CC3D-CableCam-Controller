#include "config.h"
#include "protocol.h"
#include <errno.h>
#include "stdio.h"
#include "stdbool.h"
#include "stdlib.h"
#include "math.h"
#include "errno.h"
#include "serial_print.h"
#include "controller.h"
#include "sbus.h"

#define COMMAND_START  '$'
#define COMMAND_ARGUMENTS 'a'
#define COMMAND_CHECKSUM '*'
#define COMMAND_CHECKSUMBYTE1 '1'
#define COMMAND_CHECKSUMBYTE2 '2'
#define COMMAND_IDLE ' '

#define MAX_ARGUMENT_SIZE 20
#define MAX_ARGUMENTS  10
#define ERROR_OTHER                   0
#define ERROR_MAX_ARGUMENT_SIZE       1
#define ERROR_MAX_ARGUMENTS           2
#define ERROR_UNEXPECTED_LINEFEED     3
#define ERROR_CHECKSUM                4
#define ERROR_CHECKSUM_TOO_LONG       5
#define ERROR_UNKNOWN_COMMAND         6
#define ERROR_NUMBER_OF_ARGUMENTS     7
#define ERROR_EEPROM_SAVE             8
#define ERROR_NEUTRAL_RANGE			  9
#define ERROR_ESC_DIRECTION			 10
#define ERROR_MODE					 11
#define ERROR_BT_CONFIG_FAILED		 12
#define ERROR_BT_NOT_FROM_USB		 13

static char * error_string[] = {"other errors",
		"max size of argument exceeded",
		"max number of arguments exceeded",
		"linefeed char premature",
		"wrong checksum",
		"more than two chars in checksum",
		"unknown command",
		"wrong number of arguments",
		"eeprom save failed",
		"neutral values outside 1400..1600?!?",
		"esc direction is either +1 or -1",
		"allowed modes are 0..absolute, 1..braking at endpoints, 2..passthrough",
		"configuration of the bluetooth module failed",
		"cannot configure serial bluetooth module from its own serial line"
};

settings_t defaultsettings;
settings_t activesettings;

static char arguments[MAX_ARGUMENTS][MAX_ARGUMENT_SIZE+1];  // extra char for the trailing string 0
static uint8_t checksum;
static uint8_t checksum_received;
static uint8_t checksum_response;
static uint8_t command;
static uint8_t argument_size, argument_index;

void evaluateCommand(Endpoints endpoint);
void writeProtocolError(uint8_t, Endpoints endpoint);
void writeProtocolErrorText(char *, Endpoints endpoint);
void writeProtocolOK(Endpoints endpoint);
void writeProtocolInt(int v, Endpoints endpoint);

uint8_t is_ok(uint8_t *btchar_string, uint8_t * btchar_string_length);

void initProtocol() {
}

void writeProtocolError(uint8_t e, Endpoints endpoint) {
	PrintSerial_string("$ERROR: ", endpoint);
	PrintlnSerial_string(error_string[e], endpoint);
}

void writeProtocolErrorText(char * e, Endpoints endpoint) {
	PrintSerial_string("$ERROR: ", endpoint);
	PrintlnSerial_string(e, endpoint);
}

void writeProtocolOK(Endpoints endpoint) {
	checksum_response ^= ' ';
	checksum_response ^= 'O';
	checksum_response ^= 'K';
	char buf[5];
	snprintf(buf, 5, "%02x", checksum_response);
	PrintSerial_string(" OK*", endpoint);
	PrintlnSerial_string(buf, endpoint);
}

void writeProtocolHead(char command, Endpoints endpoint) {
	PrintlnSerial(endpoint);
	PrintSerial_string("$", endpoint);
	PrintSerial_char(command, endpoint);
	checksum_response = command;
}

void writeProtocolText(char * text, Endpoints endpoint) {
	int i = 0;
	while(i<40 && text[i] != 0) {
		checksum_response ^= text[i];
		i++;
	}
	PrintSerial_string(text, endpoint);
}

void writeProtocolChar(char c, Endpoints endpoint) {
	checksum_response ^= c;
	PrintSerial_char(c, endpoint);
}

void writeProtocolDouble(double v, Endpoints endpoint) {
	char bufpd[80];
	snprintf(bufpd, sizeof(bufpd), " %7.3f ", v);
	int k = 0;
	while(k < 40 && bufpd[k] != 0) {
		checksum_response ^= bufpd[k];
		k++;
	}
	PrintSerial_string(bufpd, endpoint);
}

void writeProtocolInt(int v, Endpoints endpoint) {
	char bufpd[80];
	snprintf(bufpd, sizeof(bufpd), " %d ", v);
	int k = 0;
	while(k < 40 && bufpd[k] != 0) {
		checksum_response ^= bufpd[k];
		k++;
	}
	PrintSerial_string(bufpd, endpoint);
}

void serialCom(char * buf, Endpoints endpoint) {
	uint8_t c;
	uint16_t pos = 0;
	uint8_t c_state = COMMAND_IDLE;

	while (pos < APP_TX_BUF_SIZE) {
		c = buf[pos++];
		// regular data handling to detect and handle MSP and other data
		if (c_state == COMMAND_IDLE) {
			if (c == '$' || c == '!') {
				c_state = COMMAND_START;
			} else if (c == 0) {
				// ignore text
				if (buf[0] != '\r' && buf[0] != '\n') {
					writeProtocolError(ERROR_UNKNOWN_COMMAND, endpoint);
				}
				return;
			}
		}
		else if (c_state == COMMAND_START) {
			command = c;
			argument_size = 0;
			argument_index = 0;
			checksum = c; // first char for the checksum calculation
			c_state = COMMAND_ARGUMENTS;
		}
		else if (c_state == COMMAND_ARGUMENTS && argument_size >= MAX_ARGUMENT_SIZE) {
			writeProtocolError(ERROR_MAX_ARGUMENT_SIZE, endpoint);
			return;
		}
		else if (c_state == COMMAND_ARGUMENTS && argument_index >= MAX_ARGUMENTS) {
			writeProtocolError(ERROR_MAX_ARGUMENTS, endpoint);
			return;
		}
		else if (c_state == COMMAND_ARGUMENTS && c == ',') {
			checksum ^= c;
			arguments[argument_index][argument_size] = 0;
			argument_index++;
			argument_size = 0;
		}
		else if (c_state == COMMAND_ARGUMENTS && c == '*') {
			c_state = COMMAND_CHECKSUMBYTE1;
			// a *-char terminates the previous argument just like a ,-char would have done above
			arguments[argument_index][argument_size] = 0;
			if (argument_size != 0) {
				// yes, the previous text was an argument
				argument_index++;
			}
		}
		else if (c_state == COMMAND_ARGUMENTS && (c == '\r' || c == '\n' || c == 0)) {
            c_state = COMMAND_CHECKSUMBYTE1;
            // a *-char terminates the previous argument just like a ,-char would have done above
            arguments[argument_index][argument_size] = 0;
            if (argument_size != 0) {
                // yes, the previous text was an argument
                argument_index++;
            }
            evaluateCommand(endpoint);
            return;
		}
		else if (c_state == COMMAND_ARGUMENTS ) {
			checksum ^= c;
			if (c != ' ') {
				arguments[argument_index][argument_size++] = c;
			}
		}
		else if (c_state == COMMAND_CHECKSUMBYTE1 && c >= '0' && c <= '9') {
			checksum_received = 16 * (c - '0');
			c_state = COMMAND_CHECKSUMBYTE2;
		}
		else if (c_state == COMMAND_CHECKSUMBYTE1 && c >= 'A' && c <= 'F') {
			checksum_received = 16 * (c - 'A' + 10);
			c_state = COMMAND_CHECKSUMBYTE2;
		}
		else if (c_state == COMMAND_CHECKSUMBYTE1 && c >= 'a' && c <= 'f') {
			checksum_received = 16 * (c - 'a' + 10);
			c_state = COMMAND_CHECKSUMBYTE2;
		}
		else if (c_state == COMMAND_CHECKSUMBYTE2 && c >= '0' && c <= '9') {
			checksum_received += c - '0';
			c_state = COMMAND_CHECKSUM;
		}
		else if (c_state == COMMAND_CHECKSUMBYTE2 && c >= 'A' && c <= 'F') {
			checksum_received += c - 'A' + 10;
			c_state = COMMAND_CHECKSUM;
		}
		else if (c_state == COMMAND_CHECKSUMBYTE2 && c >= 'a' && c <= 'f') {
			checksum_received += c - 'a' + 10;
			c_state = COMMAND_CHECKSUM;
		}
		else if (c_state == COMMAND_CHECKSUM && (c == '\r' || c == '\n' || c == 0)) {
			if (checksum == checksum_received) {  // compare calculated and transferred checksum
				evaluateCommand(endpoint);
				return;
			} else {
				writeProtocolError(ERROR_CHECKSUM, endpoint);
				return;
			}
		}
		else if (c_state == COMMAND_CHECKSUMBYTE1 && (c == '\r' || c == '\n' || c == 0)) {
			// it is also okay to terminate a command with ....*\r\n without a checksum - for testing
			evaluateCommand(endpoint);
			return;
		}
		else if (c_state == COMMAND_CHECKSUM) {
			// wait a sec, we got an e.g. ....*3F as checksum followed by extra chars that are not a linefeed?? Something is wrong here
			writeProtocolError(ERROR_CHECKSUM_TOO_LONG, endpoint);
			return;
		} else {
			writeProtocolError(ERROR_OTHER, endpoint);
			return;
		}
	}
	// below should never be reached
	// writeProtocolError(ERROR_OTHER);
}

bool stringToDouble(char *string_start, double * value) {
	char *string_end;
	*value = strtod(string_start, &string_end);
	if (errno == 0 && string_start != string_end && *string_end == 0) {
		return true;
	} else {
		return false;
	}
}

bool stringToByte(char *string_start, uint8_t * value) {
	*value = atoi(string_start);
	return true;
}

bool stringToInt(char *string_start, int16_t * value) {
	*value = atoi(string_start);
	return true;
}

void evaluateCommand(Endpoints endpoint) {

	switch(command) {
	case PROTOCOL_P:
		if (argument_index == 1) {
			double kp;
			if (stringToDouble(arguments[0], &kp)) {
				setPValue(kp);
				writeProtocolHead(PROTOCOL_P, endpoint);
				writeProtocolOK(endpoint);
			}
		} else {
			writeProtocolHead(PROTOCOL_P, endpoint);
			writeProtocolDouble(getPValue(), endpoint);
			writeProtocolOK(endpoint);
		}
		break;
	case PROTOCOL_I:
		if (argument_index == 1) {
			double ki;
			if (stringToDouble(arguments[0], &ki)) {
				setIValue(ki);
				writeProtocolHead(PROTOCOL_I, endpoint);
				writeProtocolOK(endpoint);
			}
		} else {
			writeProtocolHead(PROTOCOL_I, endpoint);
			writeProtocolDouble(getIValue(), endpoint);
			writeProtocolOK(endpoint);
		}
		break;
	case PROTOCOL_D:
		if (argument_index == 1) {
			double kd;
			if (stringToDouble(arguments[0], &kd)) {
				setDValue(kd);
				writeProtocolHead(PROTOCOL_D, endpoint);
				writeProtocolOK(endpoint);
			}
		} else {
			writeProtocolHead(PROTOCOL_D, endpoint);
			writeProtocolDouble(getDValue(), endpoint);
			writeProtocolOK(endpoint);
		}
		break;
	case PROTOCOL_PID:
		if (argument_index == 3) {
			double kd;
			if (stringToDouble(arguments[0], &kd)) {
				setPValue(kd);
				if (stringToDouble(arguments[1], &kd)) {
					setIValue(kd);
					if (stringToDouble(arguments[2], &kd)) {
						setDValue(kd);
						writeProtocolHead(PROTOCOL_PID, endpoint);
						writeProtocolOK(endpoint);
					}
				}
			}
		} else if (argument_index == 0) {
			writeProtocolHead(PROTOCOL_PID, endpoint);
			writeProtocolDouble(getPValue(), endpoint);
			writeProtocolDouble(getIValue(), endpoint);
			writeProtocolDouble(getDValue(), endpoint);
			writeProtocolOK(endpoint);
		} else {
			writeProtocolError(ERROR_NUMBER_OF_ARGUMENTS, endpoint);
		}
		break;
	case PROTOCOL_MAX_ACCEL:
		if (argument_index == 1) {
			double p;
			if (stringToDouble(arguments[0], &p)) {
				setMaxAcceleration(p);
				writeProtocolHead(PROTOCOL_MAX_ACCEL, endpoint);
				writeProtocolOK(endpoint);
			}
		} else {
			writeProtocolHead(PROTOCOL_MAX_ACCEL, endpoint);
			writeProtocolDouble(getMaxAcceleration(), endpoint);
			writeProtocolOK(endpoint);
		}
		break;
	case PROTOCOL_MAX_STICK_ACCEL:
		if (argument_index == 1) {
			double p;
			if (stringToDouble(arguments[0], &p)) {
				setMaxAccelerationRCStick(p);
				writeProtocolHead(PROTOCOL_MAX_STICK_ACCEL, endpoint);
				writeProtocolOK(endpoint);
			}
		} else {
			writeProtocolHead(PROTOCOL_MAX_STICK_ACCEL, endpoint);
			writeProtocolDouble(getMaxAccelerationRCStick(), endpoint);
			writeProtocolOK(endpoint);
		}
		break;
	case PROTOCOL_MIN_ERROR_DIST:
		if (argument_index == 1) {
			double p;
			if (stringToDouble(arguments[0], &p)) {
				setToleratedPositionalErrorLimit(p);
				writeProtocolHead(PROTOCOL_MIN_ERROR_DIST, endpoint);
				writeProtocolOK(endpoint);
			}
		} else {
			writeProtocolHead(PROTOCOL_MIN_ERROR_DIST, endpoint);
			writeProtocolDouble(getToleratedPositionalErrorLimit(), endpoint);
			writeProtocolOK(endpoint);
		}
		break;
	case PROTOCOL_MAX_ERROR_DIST:
		if (argument_index == 1) {
			double p;
			if (stringToDouble(arguments[0], &p)) {
				setMaxAllowedPositionalError(p);
				writeProtocolHead(PROTOCOL_MAX_ERROR_DIST, endpoint);
				writeProtocolOK(endpoint);
			}
		} else {
			writeProtocolHead(PROTOCOL_MAX_ERROR_DIST, endpoint);
			writeProtocolDouble(getMaxAllowedPositionalError(), endpoint);
			writeProtocolOK(endpoint);
		}
		break;
	case PROTOCOL_START_POS:
		if (argument_index == 1) {
			double p;
			if (stringToDouble(arguments[0], &p)) {
				setPosStart(p);
				writeProtocolHead(PROTOCOL_START_POS, endpoint);
				writeProtocolOK(endpoint);
			}
		} else {
			writeProtocolHead(PROTOCOL_START_POS, endpoint);
			writeProtocolDouble(getPosStart(), endpoint);
			writeProtocolOK(endpoint);
		}
		break;
	case PROTOCOL_END_POS:
		if (argument_index == 1) {
			double p;
			if (stringToDouble(arguments[0], &p)) {
				setPosEnd(p);
				writeProtocolHead(PROTOCOL_END_POS, endpoint);
				writeProtocolOK(endpoint);
			}
		} else {
			writeProtocolHead(PROTOCOL_END_POS, endpoint);
			writeProtocolDouble(getPosEnd(), endpoint);
			writeProtocolOK(endpoint);
		}
		break;
	case PROTOCOL_MAX_SPEED:
		if (argument_index == 1) {
			double p;
			if (stringToDouble(arguments[0], &p)) {
				setMaxSpeed(p);
				writeProtocolHead(PROTOCOL_MAX_SPEED, endpoint);
				writeProtocolOK(endpoint);
			}
		} else {
			writeProtocolHead(PROTOCOL_MAX_SPEED, endpoint);
			writeProtocolDouble(getMaxSpeed(), endpoint);
			writeProtocolOK(endpoint);
		}
		break;
	case PROTOCOL_TARGET_POS:
		if (argument_index == 1) {
			double p;
			if (stringToDouble(arguments[0], &p)) {
				setTargetPos(p);
				writeProtocolHead(PROTOCOL_TARGET_POS, endpoint);
				writeProtocolOK(endpoint);
			}
		} else {
			writeProtocolHead(PROTOCOL_TARGET_POS, endpoint);
			writeProtocolDouble(getTargetPos(), endpoint);
			writeProtocolOK(endpoint);
		}
		break;
	case PROTOCOL_PITCH:
		if (argument_index == 1) {
			double p;
			if (stringToDouble(arguments[0], &p)) {
				setPitch(p);
				writeProtocolHead(PROTOCOL_PITCH, endpoint);
				writeProtocolOK(endpoint);
			}
		} else {
			writeProtocolHead(PROTOCOL_PITCH, endpoint);
			writeProtocolDouble(getPitch(), endpoint);
			writeProtocolOK(endpoint);
		}
		break;
	case PROTOCOL_YAW:
		if (argument_index == 1) {
			double p;
			if (stringToDouble(arguments[0], &p)) {
				setYaw(p);
				writeProtocolHead(PROTOCOL_YAW, endpoint);
				writeProtocolOK(endpoint);
			}
		} else {
			writeProtocolHead(PROTOCOL_YAW, endpoint);
			writeProtocolDouble(getYaw(), endpoint);
			writeProtocolOK(endpoint);
		}
		break;
	case PROTOCOL_SPEED:
		writeProtocolHead(PROTOCOL_SPEED, endpoint);
		writeProtocolDouble(getSpeed(), endpoint);
		writeProtocolOK(endpoint);
		break;
	case PROTOCOL_POS:
		writeProtocolHead(PROTOCOL_POS, endpoint);
		writeProtocolDouble(getPos(), endpoint);
		writeProtocolOK(endpoint);
		break;
/*	case PROTOCOL_EEPROM_WRITE:
		write_errors = eeprom_write_sector_safe((uint8_t*) &activesettings, sizeof(activesettings), EEPROM_SECTOR_FOR_SETTINGS);
		writeProtocolHead(PROTOCOL_EEPROM_WRITE, endpoint);
		if (write_errors == 0) {
			writeProtocolOK(endpoint);
		} else {
			writeProtocolError(ERROR_EEPROM_SAVE, endpoint);
		}
		break; */
	case PROTOCOL_INPUT_CHANNELS:
		if (argument_index == 4) {
			int16_t p[4];
			if (stringToInt(arguments[0], &p[0]) && stringToInt(arguments[1], &p[1]) && stringToInt(arguments[2], &p[2]) && stringToInt(arguments[3], &p[3])) {
				activesettings.speed_rc_channel = p[0];
				activesettings.pitch_rc_channel = p[1];
				activesettings.yaw_rc_channel = p[2];
				activesettings.aux_rc_channel = p[3];
				writeProtocolHead(PROTOCOL_INPUT_CHANNELS, endpoint);
				writeProtocolOK(endpoint);
			}
		} else if (argument_index == 0) {
			writeProtocolHead(PROTOCOL_INPUT_CHANNELS, endpoint);
			writeProtocolInt(activesettings.speed_rc_channel, endpoint);
			writeProtocolInt(activesettings.pitch_rc_channel, endpoint);
			writeProtocolInt(activesettings.yaw_rc_channel, endpoint);
			writeProtocolInt(activesettings.aux_rc_channel, endpoint);
			writeProtocolText("\r\n", endpoint);

			int i = 0;
			for (i=0; i<SBUS_MAX_CHANNEL; i++) {
				writeProtocolText("last valid RC signal for channel ", endpoint);
				writeProtocolInt(i, endpoint);
				writeProtocolText("=", endpoint);
				writeProtocolInt(getDuty(i), endpoint);
				if (i == activesettings.speed_rc_channel) {
					writeProtocolText(" (used as speed signal input)", endpoint);
				} else if (i == activesettings.pitch_rc_channel) {
					writeProtocolText(" (used as camera pitch signal input)", endpoint);
				} else if (i == activesettings.yaw_rc_channel) {
					writeProtocolText(" (used as camera yaw signal input)", endpoint);
				} else if (i == activesettings.aux_rc_channel) {
					writeProtocolText(" (used as aux signal input)", endpoint);
				}
				writeProtocolText("\r\n", endpoint);
			}
			writeProtocolOK(endpoint);
		} else {
			writeProtocolError(ERROR_NUMBER_OF_ARGUMENTS, endpoint);
		}
		break;
	case PROTOCOL_NEUTRAL:
		if (argument_index == 2) {
			int16_t p[4];
			if (stringToInt(arguments[0], &p[0]) && stringToInt(arguments[1], &p[1])) {
				writeProtocolHead(PROTOCOL_NEUTRAL, endpoint);
				if (p[0] > 1400 && p[0] < 1600 && p[1] > 1400 && p[1] < 1600) {
					if (p[0] < p[1]) {
						setServoNeutralRange(p[0], p[1]);
					} else {
						setServoNeutralRange(p[1], p[0]);
					}
					writeProtocolOK(endpoint);
				} else {
					writeProtocolError(ERROR_NEUTRAL_RANGE, endpoint);
				}
			}
		} else {
			writeProtocolHead(PROTOCOL_NEUTRAL, endpoint);
			writeProtocolInt(activesettings.neutrallow, endpoint);
			writeProtocolInt(activesettings.neutralhigh, endpoint);
			writeProtocolOK(endpoint);
		}
		break;
	case PROTOCOL_ROTATION_DIR:
		if (argument_index == 1) {
			int16_t p;
			if (stringToInt(arguments[0], &p)) {
				writeProtocolHead(PROTOCOL_ROTATION_DIR, endpoint);
				if (p == 1 || p == -1) {
					activesettings.esc_direction = p;
					writeProtocolOK(endpoint);
				} else {
					writeProtocolError(ERROR_ESC_DIRECTION, endpoint);
				}
			}
		} else {
			writeProtocolHead(PROTOCOL_ROTATION_DIR, endpoint);
			writeProtocolInt(activesettings.esc_direction, endpoint);
			writeProtocolOK(endpoint);
		}
		break;
	case PROTOCOL_HELP:
		printHelp(endpoint);
		break;
	case PROTOCOL_INPUT:
		{
			int16_t channel = 0;
			writeProtocolHead(PROTOCOL_INPUT, endpoint);
			writeProtocolText("Input:", endpoint);
			for (channel = 0; channel < SBUS_MAX_CHANNEL; channel++) {
				writeProtocolText("    [", endpoint);
				writeProtocolInt(channel, endpoint);
				writeProtocolText("] =", endpoint);
				writeProtocolInt(getDuty(channel), endpoint);
			}
			writeProtocolOK(endpoint);
		}
		break;
	case PROTOCOL_MODE:
		if (argument_index == 1) {
			int16_t p;
			if (stringToInt(arguments[0], &p)) {
				writeProtocolHead(PROTOCOL_MODE, endpoint);
				if (p == MODE_ABSOLUTE_POSITION || p == MODE_LIMITER || p == MODE_LIMITER_ENDPOINTS || p == MODE_PASSTHROUGH) {
					activesettings.mode = p;
					resetThrottle();
					writeProtocolOK(endpoint);
				} else {
					writeProtocolError(ERROR_MODE, endpoint);
				}
			}
		} else {
			writeProtocolHead(PROTOCOL_MODE, endpoint);
			writeProtocolInt(activesettings.mode, endpoint);
			writeProtocolText("...", endpoint);
			if (activesettings.mode == MODE_ABSOLUTE_POSITION) {
				writeProtocolText("absolute position", endpoint);
			} else if (activesettings.mode == MODE_LIMITER) {
				writeProtocolText("passthrough with limiter", endpoint);
			} else if (activesettings.mode == MODE_LIMITER_ENDPOINTS) {
				writeProtocolText("passthrough with end points", endpoint);
			} else if (activesettings.mode == MODE_PASSTHROUGH) {
				writeProtocolText("passthrough", endpoint);
			}
			writeProtocolOK(endpoint);
		}
		break;
	case PROTOCOL_SETTINGS:
		{
			writeProtocolHead(PROTOCOL_SETTINGS, endpoint);
			writeProtocolOK(endpoint);
			printActiveSettings(endpoint);

		}
		break;
	case PROTOCOL_AUX_VALUE:
		if (argument_index == 1) {
			int16_t value;
			if (stringToInt(arguments[0], &value)) {
				setServoValueForChannel(SERVO_AUX, value);
				activesettings.auxvalue = value;
				writeProtocolHead(PROTOCOL_AUX_VALUE, endpoint);
				writeProtocolOK(endpoint);
			}
		} else {
			writeProtocolHead(PROTOCOL_AUX_VALUE, endpoint);
			writeProtocolDouble(getRCDataForChannel(SERVO_AUX), endpoint);
			writeProtocolOK(endpoint);
		}
		break;
	default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
		writeProtocolError(ERROR_UNKNOWN_COMMAND, endpoint);
		break;
	}
}

void printHelp(Endpoints endpoint) {
	PrintlnSerial(endpoint);
	PrintlnSerial(endpoint);
	PrintlnSerial(endpoint);
	PrintlnSerial_string("Possible commands are", endpoint);
	PrintlnSerial(endpoint);

	PrintlnSerial_string("$1 [<double>]                         set or print Kp for PID controller", endpoint);
	PrintlnSerial_string("$2 [<double>]                         set or print Ki for PID controller", endpoint);
	PrintlnSerial_string("$3 [<double>]                         set or print Kd for PID controller", endpoint);
	PrintlnSerial_string("$5 [<int>]                            set or print the Servo5 (AUX) output value", endpoint);
	PrintlnSerial_string("$a [<double>]                         set or print maximum allowed acceleration", endpoint);
	PrintlnSerial_string("$A [<double>]                         set or print maximum allowed rc signal change if mode is 1 or 3", endpoint);
	PrintlnSerial_string("$b [<double>]                         set or print begin position", endpoint);
	if (endpoint == EndPoint_USB) {
		PrintlnSerial_string("$B                                    configure the Bluetooth HC-06 module in the Flexi Port", endpoint);
	}
	PrintlnSerial_string("$c [<double>, <double>, <double>]     set or print all three PID values", endpoint);
	PrintlnSerial_string("$e [<double>]                         set or print end position", endpoint);
	PrintlnSerial_string("$f [<double>]                         set or print the tolerated positional error", endpoint);
	PrintlnSerial_string("$g [<double>]                         set or print the max positional error -> exceeding it", endpoint);
	PrintlnSerial_string("                                                                               causes an emergency stop", endpoint);
	PrintlnSerial_string("$i [<int>, <int>, <int>, <int>]       set or print input channels for Speed, Pitch, Yaw, Aux", endpoint);
	PrintlnSerial_string("$l                                    print current position", endpoint);
	PrintlnSerial_string("$m [<int>]                            set or print the mode 0..absolute", endpoint);
	PrintlnSerial_string("                                                            1..passthrough with limits", endpoint);
	PrintlnSerial_string("                                                            2..passthrough", endpoint);
	PrintlnSerial_string("                                                            3..break at endpoints", endpoint);
	PrintlnSerial_string("$n [<int>, <int>]                     set or print neutral start/end in microseconds, e.g. 1450, 1550", endpoint);
	PrintlnSerial_string("$p [<double>]                         set or print camera pitch", endpoint);
	PrintlnSerial_string("$r [<int>]                            set or print rotation direction of the ESC output, either +1 or -1", endpoint);
	PrintlnSerial_string("$s [<double>]                         set or print maximum allowed velocity", endpoint);
	PrintlnSerial_string("$t [<double>]                         set or print target position", endpoint);
	PrintlnSerial_string("$v                                    print current velocity", endpoint);
	PrintlnSerial_string("$w                                    write settings to eeprom", endpoint);
	PrintlnSerial_string("$y [<double>]                         set or print camera yaw", endpoint);
	PrintlnSerial(endpoint);
}

void printActiveSettings(Endpoints endpoint) {
	PrintlnSerial(endpoint);
	PrintlnSerial(endpoint);
	PrintlnSerial(endpoint);
	PrintlnSerial_string("Active settings", endpoint);
	PrintlnSerial(endpoint);
	PrintlnSerial(endpoint);

	PrintSerial_string("Version of the firmware: ", endpoint);
	PrintlnSerial_string(activesettings.version, endpoint);
	PrintlnSerial(endpoint);
	PrintlnSerial(endpoint);



	PrintlnSerial_string("Initially the begin/endpositions are not set.", endpoint);
	PrintlnSerial_string("The point where the controller was turned on is the zero begin position, then drive forward or backward to", endpoint);
	PrintlnSerial_string("the end position and wait there for 30 seconds motionless. Minimum distance is 1000 steps.", endpoint);
	PrintlnSerial_string("After that time, the begin and start position is defined and the controller will stay inside those limits.", endpoint);
	PrintlnSerial_string("see $b and $e commands to manually set those.", endpoint);
	PrintlnSerial(endpoint);
	PrintSerial_string("Current status:", endpoint);
	if (getPosEnd() == POS_END_NOT_SET) {
		PrintlnSerial_string("Begin/endposition not set", endpoint);
	} else if (getPosStart() < getPosEnd() ) {
		PrintSerial_double(getPosStart(), endpoint);
		PrintSerial_string("....to....", endpoint);
		PrintSerial_double(getPosEnd(), endpoint);
		PrintlnSerial(endpoint);
	} else {
		PrintSerial_double(getPosEnd(), endpoint);
		PrintSerial_string("....to....", endpoint);
		PrintSerial_double(getPosStart(), endpoint);
		PrintlnSerial(endpoint);
	}
	PrintlnSerial(endpoint);
	PrintlnSerial(endpoint);




	PrintlnSerial_string("The controller allows for a maximum acceleration and deceleration as well as max speed forward and reverse.", endpoint);
	PrintlnSerial_string("Note, these value takes effect only once the end position is set! Initially they are much smaller.", endpoint);
	PrintlnSerial(endpoint);
	PrintSerial_string("Maximum acceleration when end position is set [steps/20ms²]:", endpoint);
	PrintSerial_double(activesettings.max_accel, endpoint);
	if (getPosEnd() == POS_END_NOT_SET) {
		PrintlnSerial_string(" (not active)", endpoint);
	} else {
		PrintlnSerial(endpoint);
	}
	PrintSerial_string("Maximum velocity when end position is set [steps/20ms]:", endpoint);
	PrintSerial_double(activesettings.max_speed, endpoint);
	if (getPosEnd() == POS_END_NOT_SET) {
		PrintlnSerial_string(" (not active)", endpoint);
	} else {
		PrintlnSerial(endpoint);
	}
	PrintlnSerial(endpoint);
	PrintlnSerial(endpoint);




	PrintlnSerial_string("With the RC speed input the target position is controlled, not the motor ESC directly. Then there is a PID", endpoint);
	PrintlnSerial_string("control loop whose goal is to reach that point precisely and smoothly. For that the P, I and D coefficients", endpoint);
	PrintlnSerial_string("need to be adjusted carefully.", endpoint);
	PrintlnSerial(endpoint);
	PrintSerial_string("PID coefficients: P=", endpoint);
	PrintSerial_double(activesettings.P, endpoint);
	PrintSerial_string("    I=", endpoint);
	PrintSerial_double(activesettings.I, endpoint);
	PrintSerial_string("    D=", endpoint);
	PrintSerial_double(activesettings.D, endpoint);
	PrintlnSerial(endpoint);
	PrintlnSerial(endpoint);
	PrintlnSerial(endpoint);




	PrintlnSerial_string("One issue with a PID control loop is that it requires a continuous input/output function. The neutral range", endpoint);
	PrintlnSerial_string("of the ESC behaves differently (braking) than outside, hence should be eliminated by the controller unless", endpoint);
	PrintlnSerial_string("braking is what is desired. Hence we need the neutral start and end position of the ESC.", endpoint);
	PrintlnSerial(endpoint);
	PrintSerial_string("Neutral range[us]:", endpoint);
	PrintSerial_int(activesettings.neutrallow, endpoint);
	PrintSerial_string("...to...", endpoint);
	PrintSerial_int(activesettings.neutralhigh, endpoint);
	PrintlnSerial(endpoint);
	PrintlnSerial(endpoint);
	PrintlnSerial(endpoint);




	PrintlnSerial_string("The RC input signals are mapped to internal channel numbers. Which of these channels is used for the RC speed", endpoint);
	PrintlnSerial_string("signal, the yaw, pitch, and aux input?", endpoint);
	PrintlnSerial(endpoint);
	PrintSerial_string("Channel mapping:  Speed=", endpoint);
	PrintSerial_int(activesettings.speed_rc_channel, endpoint);
	PrintSerial_string("    Yaw=", endpoint);
	PrintSerial_int(activesettings.yaw_rc_channel, endpoint);
	PrintSerial_string("    Pitch=", endpoint);
	PrintSerial_int(activesettings.pitch_rc_channel, endpoint);
	PrintSerial_string("    Aux=", endpoint);
	PrintSerial_int(activesettings.aux_rc_channel, endpoint);
	PrintlnSerial(endpoint);
	PrintlnSerial(endpoint);
	PrintlnSerial(endpoint);




	PrintlnSerial_string("The sensor direction and the motor direction might not be the same. The result would be that the controller", endpoint);
	PrintlnSerial_string("finds it has to drive 100 steps forward, hence does increase the power and the next moment it has to drive", endpoint);
	PrintlnSerial_string("120step. The more power the more steps it needs to correct. Obviously it should have driven to reverse instead.", endpoint);
	PrintlnSerial(endpoint);
	PrintSerial_string("Motor/ESC direction:", endpoint);
	PrintSerial_int(activesettings.esc_direction, endpoint);
	PrintlnSerial(endpoint);
	PrintlnSerial(endpoint);
	PrintlnSerial(endpoint);




	PrintlnSerial_string("The controller might write no (0) or much (5) debug information.", endpoint);
	PrintlnSerial(endpoint);
	PrintSerial_string("Debug Level:", endpoint);
	PrintSerial_int(activesettings.debuglevel, endpoint);
	PrintlnSerial(endpoint);
	PrintlnSerial(endpoint);
	PrintlnSerial(endpoint);


	PrintlnSerial_string("The controller mode.", endpoint);
	PrintlnSerial(endpoint);
	PrintSerial_string("Mode:", endpoint);
	PrintSerial_int(activesettings.mode, endpoint);
	PrintSerial_string("...", endpoint);
	if (activesettings.mode == MODE_ABSOLUTE_POSITION) {
		PrintlnSerial_string("absolute position", endpoint);
	} else if (activesettings.mode == MODE_LIMITER) {
		PrintlnSerial_string("use limiter", endpoint);
	} else {
		PrintlnSerial_string("passthrough", endpoint);
	}

	PrintlnSerial(endpoint);
	PrintlnSerial(endpoint);
	PrintlnSerial(endpoint);
}


uint8_t is_ok(uint8_t *btchar_string, uint8_t * btchar_string_length) {
	if (*btchar_string_length >= 2 && btchar_string[0] == 'O' && btchar_string[1] == 'K') {
		return 1;
	} else {
		return 0;
	}
}


