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
#include "eeprom.h"
#include "usbd_cdc_if.h"

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
#define ERROR_INVALID_VALUE 		 14

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
                                "cannot configure serial bluetooth module from its own serial line",
                                "invalid value for provided argument(s)"
                               };

settings_t activesettings;
controllerstatus_t controllerstatus;


char commandline[RXBUFFERSIZE+1]; // one extra char for the null termination
static uint8_t commandlinepos = 0;

extern char commandlinebuffer[RXBUFFERSIZE];

extern TIM_HandleTypeDef htim1;

static uint8_t checksum;
static uint8_t checksum_received;
static uint8_t checksum_response;

void evaluateCommand(Endpoints endpoint);
void writeProtocolError(uint8_t, Endpoints endpoint);
void writeProtocolErrorText(char *, Endpoints endpoint);
void writeProtocolOK(Endpoints endpoint);
void writeProtocolInt(int16_t v, Endpoints endpoint);
void writeProtocolLong(int32_t v, Endpoints endpoint);
void writeProtocolHex(uint8_t v, Endpoints endpoint);
int hexDigitToInt(char digit);

void printDebugCycles(Endpoints endpoint);


uint8_t is_ok(uint8_t *btchar_string, uint8_t * btchar_string_length);

void initProtocol()
{
}

void writeProtocolError(uint8_t e, Endpoints endpoint)
{
    PrintSerial_string("$ERROR: ", endpoint);
    PrintlnSerial_string(error_string[e], endpoint);
}

void writeProtocolErrorText(char * e, Endpoints endpoint)
{
    PrintSerial_string("$ERROR: ", endpoint);
    PrintlnSerial_string(e, endpoint);
}

void writeProtocolOK(Endpoints endpoint)
{
    checksum_response ^= ' ';
    checksum_response ^= 'O';
    checksum_response ^= 'K';
    char buf[5];
    snprintf(buf, 5, "%02x", checksum_response);
    PrintSerial_string(" OK*", endpoint);
    PrintlnSerial_string(buf, endpoint);
}

void writeProtocolHead(char command, Endpoints endpoint)
{
    PrintlnSerial(endpoint);
    PrintSerial_string("$", endpoint);
    PrintSerial_char(command, endpoint);
    checksum_response = command;
}

void writeProtocolText(char * text, Endpoints endpoint)
{
    int i = 0;
    while(i<40 && text[i] != 0)
    {
        checksum_response ^= text[i];
        i++;
    }
    PrintSerial_string(text, endpoint);
}

void writeProtocolChar(char c, Endpoints endpoint)
{
    checksum_response ^= c;
    PrintSerial_char(c, endpoint);
}

void writeProtocolDouble(double v, Endpoints endpoint)
{
    char bufpd[80];
    snprintf(bufpd, sizeof(bufpd), " %7.3f ", v);
    int k = 0;
    while(k < 40 && bufpd[k] != 0)
    {
        checksum_response ^= bufpd[k];
        k++;
    }
    PrintSerial_string(bufpd, endpoint);
}

void writeProtocolInt(int16_t v, Endpoints endpoint)
{
    char bufpd[80];
    snprintf(bufpd, sizeof(bufpd), " %d ", v);
    int k = 0;
    while(k < 40 && bufpd[k] != 0)
    {
        checksum_response ^= bufpd[k];
        k++;
    }
    PrintSerial_string(bufpd, endpoint);
}

void writeProtocolHex(uint8_t v, Endpoints endpoint)
{
    char bufpd[3];
    snprintf(bufpd, sizeof(bufpd), "%02X", v);
    int k = 0;
    while(k < 3 && bufpd[k] != 0)
    {
        checksum_response ^= bufpd[k];
        k++;
    }
    PrintSerial_string(bufpd, endpoint);
}

void writeProtocolLong(int32_t v, Endpoints endpoint)
{
    char bufpd[80];
    snprintf(bufpd, sizeof(bufpd), " %ld ", v);
    int k = 0;
    while(k < 40 && bufpd[k] != 0)
    {
        checksum_response ^= bufpd[k];
        k++;
    }
    PrintSerial_string(bufpd, endpoint);
}

int hexDigitToInt(char digit) {
    if ('0' <= digit && digit <= '9') //if it's decimal
        return (int)(digit - '0');
    else if ('a' <= digit && digit <= 'f') //if it's abcdef
        return (int)(digit - ('a' - 10));
    else if ('A' <= digit && digit <= 'F') //if it's abcdef
        return (int)(digit - ('A' - 10));
    else
        return -1; //value not in [0-9][a-f] range
}



static uint8_t c_state = COMMAND_IDLE;

void serialCom(Endpoints endpoint)
{
    char c;
    uint16_t pos = 0;

    while (pos < RXBUFFERSIZE)
    {
        c = commandlinebuffer[pos++];

        if (c == 0)
        {
            // When the buffer has no more chars, processing stopped.
            return;
        }
        else if (c_state == COMMAND_IDLE)
        {
            /*
             * All characters before the command-start character are ignored
             */
            if (c == '$' || c == '!')
            {
                c_state = COMMAND_START;
            }
        }
        else if (c == '\r' || c == '\n')
        {
            /*
             * The command line string contains a null-terminated string like "I 1".
             * It is stripped the command begin character and the checksum.
             */
            commandline[commandlinepos] = 0; // Null-terminate the string
            if (c_state == COMMAND_CHECKSUM && checksum != checksum_received)
            {
                writeProtocolError(ERROR_CHECKSUM, endpoint);
            }
            else
            {
                evaluateCommand(endpoint);
            }
            checksum = 0;
            c_state = COMMAND_IDLE;
            commandlinepos = 0;
        }
        /*
         * Everything after a * character is ignored as this should be the checksum only
         */
        else if (c== '*')
        {
            c_state = COMMAND_CHECKSUMBYTE1;
        }
        else if (c_state == COMMAND_CHECKSUMBYTE1)
        {
            int b = hexDigitToInt(c);
            if (b != -1)
            {
                checksum_received = 16 * b;
                c_state = COMMAND_CHECKSUMBYTE2;
            }
        }
        else if (c_state == COMMAND_CHECKSUMBYTE2)
        {
            int b = hexDigitToInt(c);
            if (b != -1)
            {
                checksum_received += b;
                c_state = COMMAND_CHECKSUM;
            }
        }
        else if (commandlinepos < RXBUFFERSIZE && c_state == COMMAND_START)
        {
            commandline[commandlinepos++] = c;
            checksum ^= c;
        }
        else if (c_state == COMMAND_CHECKSUM)
        {
            // wait a sec, we got an e.g. ....*3F as checksum followed by extra chars that are not a linefeed?? Something is wrong here
            writeProtocolError(ERROR_CHECKSUM_TOO_LONG, endpoint);
            return;
        }
        else
        {
            writeProtocolError(ERROR_OTHER, endpoint);
            return;
        }
    }
}

void evaluateCommand(Endpoints endpoint)
{
    char command = commandline[0];
    uint16_t argument_index;

    switch(command)
    {
    case PROTOCOL_P:
    {
        double kp;
        argument_index = sscanf(commandline, "%c %lf", &command, &kp);
        if (argument_index == 2)
        {
            if (kp >= 0.0f)
            {
                setPValue(kp);
                writeProtocolHead(PROTOCOL_P, endpoint);
                writeProtocolOK(endpoint);
            }
            else
            {
                writeProtocolError(ERROR_INVALID_VALUE, endpoint);
            }
        }
        else
        {
            writeProtocolHead(PROTOCOL_P, endpoint);
            writeProtocolDouble(activesettings.P, endpoint);
            writeProtocolOK(endpoint);
        }
        break;
    }
    case PROTOCOL_I:
    {
        double ki;
        argument_index = sscanf(commandline, "%c %lf", &command, &ki);
        if (argument_index == 2)
        {
            if (ki >= 0.0f)
            {
                setIValue(ki);
                writeProtocolHead(PROTOCOL_I, endpoint);
                writeProtocolOK(endpoint);
            }
            else
            {
                writeProtocolError(ERROR_INVALID_VALUE, endpoint);
            }
        }
        else
        {
            writeProtocolHead(PROTOCOL_I, endpoint);
            writeProtocolDouble(activesettings.I, endpoint);
            writeProtocolOK(endpoint);
        }
        break;
    }
    case PROTOCOL_D:
    {
        double kd;
        argument_index = sscanf(commandline, "%c %lf", &command, &kd);
        if (argument_index == 2)
        {
            if (kd >= 0.0f)
            {
                setDValue(kd);
                writeProtocolHead(PROTOCOL_D, endpoint);
                writeProtocolOK(endpoint);
            }
            else
            {
                writeProtocolError(ERROR_INVALID_VALUE, endpoint);
            }
        }
        else
        {
            writeProtocolHead(PROTOCOL_D, endpoint);
            writeProtocolDouble(activesettings.D, endpoint);
            writeProtocolOK(endpoint);
        }
        break;
    }

    case PROTOCOL_PID:
    {
        double kp;
        double ki;
        double kd;
        argument_index = sscanf(commandline, "%c %lf %lf %lf", &command, &kp, &ki, &kd);
        if (argument_index == 4)
        {
            if (kp >= 0.0f && ki >= 0.0f && kd >= 0.0f)
            {
                setPValue(kp);
                setIValue(ki);
                setDValue(kd);
                writeProtocolHead(PROTOCOL_PID, endpoint);
                writeProtocolOK(endpoint);
            }
            else
            {
                writeProtocolError(ERROR_INVALID_VALUE, endpoint);
            }
        }
        else if (argument_index == 1)
        {
            writeProtocolHead(PROTOCOL_PID, endpoint);
            writeProtocolDouble(activesettings.P, endpoint);
            writeProtocolDouble(activesettings.I, endpoint);
            writeProtocolDouble(activesettings.D, endpoint);
            writeProtocolOK(endpoint);
        }
        else
        {
            writeProtocolError(ERROR_NUMBER_OF_ARGUMENTS, endpoint);
        }
        break;
    }
    case PROTOCOL_MAX_ACCEL:
    {
        int16_t p[2];
        argument_index = sscanf(commandline, "%c %hd %hd", &command, &p[0], &p[1]);
        if (argument_index == 3)
        {
            if (p[0] > 0 && p[1] > 0)
            {
                activesettings.stick_max_accel = p[0];
                activesettings.stick_max_accel_safemode = p[1];
                writeProtocolHead(PROTOCOL_MAX_ACCEL, endpoint);
                writeProtocolOK(endpoint);
            }
            else
            {
                writeProtocolError(ERROR_INVALID_VALUE, endpoint);
            }
        }
        else if (argument_index == 1)
        {
            writeProtocolHead(PROTOCOL_MAX_ACCEL, endpoint);
            writeProtocolInt(activesettings.stick_max_accel, endpoint);
            writeProtocolInt(activesettings.stick_max_accel_safemode, endpoint);
            writeProtocolOK(endpoint);
        }
        else
        {
            writeProtocolError(ERROR_NUMBER_OF_ARGUMENTS, endpoint);
        }
        break;
    }
    case PROTOCOL_MAX_ERROR_DIST:
    {
        double d;
        argument_index = sscanf(commandline, "%c %lf", &command, &d);
        if (argument_index == 2)
        {
            if (d > 0.0f)
            {
                activesettings.max_position_error = d;
                writeProtocolHead(PROTOCOL_MAX_ERROR_DIST, endpoint);
                writeProtocolOK(endpoint);
            }
            else
            {
                writeProtocolError(ERROR_INVALID_VALUE, endpoint);
            }
        }
        else
        {
            writeProtocolHead(PROTOCOL_MAX_ERROR_DIST, endpoint);
            writeProtocolInt(activesettings.max_position_error, endpoint);
            writeProtocolOK(endpoint);
        }
        break;
    }
    case PROTOCOL_SPEED_FACTOR:
    {
        double d;
        argument_index = sscanf(commandline, "%c %lf", &command, &d);
        if (argument_index == 2)
        {
            activesettings.stick_speed_factor = d;
            writeProtocolHead(PROTOCOL_SPEED_FACTOR, endpoint);
            writeProtocolOK(endpoint);
        }
        else
        {
            writeProtocolHead(PROTOCOL_SPEED_FACTOR, endpoint);
            writeProtocolDouble(activesettings.stick_speed_factor, endpoint);
            writeProtocolInt(getStick(), endpoint);
            writeProtocolOK(endpoint);
        }
        break;
    }
    case PROTOCOL_POS:
        writeProtocolHead(PROTOCOL_POS, endpoint);
        writeProtocolLong(activesettings.pos_start, endpoint);
        writeProtocolLong(activesettings.pos_end, endpoint);
        writeProtocolLong(getPos(), endpoint);
        writeProtocolDouble(getSpeedPosDifference(), endpoint);
        writeProtocolDouble(getSpeedPosSensor(), endpoint);
        if (activesettings.mode == MODE_ABSOLUTE_POSITION)
        {
            writeProtocolLong(getTargetPos(), endpoint);
        }
        writeProtocolOK(endpoint);
        break;
    case PROTOCOL_MAX_SPEED:
    {
        int16_t p[2];
        argument_index = sscanf(commandline, "%c %hd %hd", &command, &p[0], &p[1]);

        if (argument_index == 3)
        {
            if (p[0] > 0 && p[1] > 0)
            {
                activesettings.stick_max_speed = p[0];
                activesettings.stick_max_speed_safemode = p[1];
                writeProtocolHead(PROTOCOL_MAX_SPEED, endpoint);
                writeProtocolOK(endpoint);
            }
            else
            {
                writeProtocolError(ERROR_INVALID_VALUE, endpoint);
            }
        }
        else if (argument_index == 1)
        {
            writeProtocolHead(PROTOCOL_MAX_SPEED, endpoint);
            writeProtocolInt(activesettings.stick_max_speed, endpoint);
            writeProtocolInt(activesettings.stick_max_speed_safemode, endpoint);
            writeProtocolOK(endpoint);
        }
        else
        {
            writeProtocolError(ERROR_NUMBER_OF_ARGUMENTS, endpoint);
        }
        break;
    }
    case PROTOCOL_EEPROM_WRITE:
    {
        uint32_t write_errors = eeprom_write_sector_safe((uint8_t*) &activesettings, sizeof(activesettings), EEPROM_SECTOR_FOR_SETTINGS);
        writeProtocolHead(PROTOCOL_EEPROM_WRITE, endpoint);
        if (write_errors == 0)
        {
            writeProtocolText("\r\nSettings saved successfully", endpoint);
            writeProtocolOK(endpoint);
        }
        else
        {
            writeProtocolError(ERROR_EEPROM_SAVE, endpoint);
        }
        break;
    }
    case PROTOCOL_INPUT_CHANNELS:
    {
        /*
         * Note, the protocol does remap channel1 to chan0
         */
        int16_t p[6];
        argument_index = sscanf(commandline, "%c %hd %hd %hd %hd %hd %hd", &command, &p[0], &p[1], &p[2], &p[3], &p[4], &p[5]);

        if (argument_index >= 4)
        {
            if (p[0] > 0 && p[0] <= SBUS_MAX_CHANNEL &&
                    p[1] > 0 && p[1] <= SBUS_MAX_CHANNEL &&
                    p[2] > 0 && p[2] <= SBUS_MAX_CHANNEL &&
                    p[0] != p[1] && p[0] != p[2] && p[1] != p[2])
            {
                activesettings.rc_channel_speed = p[0]-1;
                activesettings.rc_channel_programming = p[1]-1;
                activesettings.rc_channel_endpoint = p[2]-1;
                writeProtocolHead(PROTOCOL_INPUT_CHANNELS, endpoint);
                writeProtocolInt(activesettings.rc_channel_speed+1, endpoint);
                writeProtocolInt(activesettings.rc_channel_programming+1, endpoint);
                writeProtocolInt(activesettings.rc_channel_endpoint+1, endpoint);
                if (argument_index >= 5)
                {
                    if (p[3] > 0 && p[3] <= SBUS_MAX_CHANNEL)
                    {
                        activesettings.rc_channel_max_accel = p[3]-1;
                        writeProtocolInt(activesettings.rc_channel_max_accel+1, endpoint);
                    }
                    else
                    {
                        activesettings.rc_channel_max_accel = 255; // not used
                    }
                }
                if (argument_index >= 6)
                {
                    if (p[4] > 0 && p[4] <= SBUS_MAX_CHANNEL)
                    {
                        activesettings.rc_channel_max_speed = p[4]-1;
                        writeProtocolInt(activesettings.rc_channel_max_speed+1, endpoint);
                    }
                    else
                    {
                        activesettings.rc_channel_max_speed = 255; // not used
                    }
                }
                if (argument_index >= 7)
                {
                    if (p[5] > 0 && p[5] <= SBUS_MAX_CHANNEL)
                    {
                        activesettings.rc_channel_mode = p[5]-1;
                        writeProtocolInt(activesettings.rc_channel_mode+1, endpoint);
                    }
                    else
                    {
                        activesettings.rc_channel_mode = 255; // not used
                    }
                }
                writeProtocolOK(endpoint);
            }
            else
            {
                writeProtocolError(ERROR_INVALID_VALUE, endpoint);
            }
        }
        else if (argument_index == 1)
        {
            writeProtocolHead(PROTOCOL_INPUT_CHANNELS, endpoint);
            writeProtocolInt(activesettings.rc_channel_speed+1, endpoint);
            writeProtocolInt(activesettings.rc_channel_programming+1, endpoint);
            writeProtocolInt(activesettings.rc_channel_endpoint+1, endpoint);
            writeProtocolInt(activesettings.rc_channel_max_accel+1, endpoint);
            writeProtocolInt(activesettings.rc_channel_max_speed+1, endpoint);
            writeProtocolInt(activesettings.rc_channel_mode+1, endpoint);
            writeProtocolText("\r\n", endpoint);

            int i = 0;
            for (i=0; i<SBUS_MAX_CHANNEL; i++)
            {
                writeProtocolText("last valid RC signal for channel ", endpoint);
                writeProtocolInt(i+1, endpoint);
                writeProtocolText("=", endpoint);
                writeProtocolInt(getDuty(i), endpoint);
                if (i == activesettings.rc_channel_speed)
                {
                    writeProtocolText(" (used as speed signal input)", endpoint);
                }
                else if (i == activesettings.rc_channel_programming)
                {
                    writeProtocolText(" (used as programming switch)", endpoint);
                }
                else if (i == activesettings.rc_channel_endpoint)
                {
                    writeProtocolText(" (used as endpoint switch)", endpoint);
                }
                else if (i == activesettings.rc_channel_max_accel)
                {
                    writeProtocolText(" (used as max acceleration value selector)", endpoint);
                }
                else if (i == activesettings.rc_channel_max_speed)
                {
                    writeProtocolText(" (used as max speed selector)", endpoint);
                 }
                else if (i == activesettings.rc_channel_mode)
                {
                    writeProtocolText(" (used as mode selector)", endpoint);
                }
                writeProtocolText("\r\n", endpoint);
            }
            writeProtocolText("current ESC out signal Servo 1 = ", endpoint);
            writeProtocolInt(TIM3->CCR3, endpoint);
            writeProtocolText("\r\n", endpoint);
            writeProtocolOK(endpoint);
        }
        else
        {
            writeProtocolError(ERROR_NUMBER_OF_ARGUMENTS, endpoint);
        }
        break;
    }
    case PROTOCOL_NEUTRAL:
    {
        int16_t p[2];
        argument_index = sscanf(commandline, "%c %hd %hd", &command, &p[0], &p[1]);
        if (argument_index == 3)
        {
            if (p[0] > 500 && p[0] < 2000 &&
                    p[1] > 0 && p[1] < 100)
            {
                writeProtocolHead(PROTOCOL_NEUTRAL, endpoint);
                activesettings.stick_neutral_pos = p[0];
                activesettings.stick_neutral_range = p[1];
                writeProtocolOK(endpoint);
            }
            else
            {
                writeProtocolError(ERROR_INVALID_VALUE, endpoint);
            }
        }
        else if (argument_index == 1)
        {
            writeProtocolHead(PROTOCOL_NEUTRAL, endpoint);
            writeProtocolInt(activesettings.stick_neutral_pos, endpoint);
            writeProtocolInt(activesettings.stick_neutral_range, endpoint);
            writeProtocolOK(endpoint);
        }
        else
        {
            writeProtocolError(ERROR_NUMBER_OF_ARGUMENTS, endpoint);
        }
        break;
    }
    case PROTOCOL_ESC_NEUTRAL:
    {
        int16_t p[2];
        argument_index = sscanf(commandline, "%c %hd %hd", &command, &p[0], &p[1]);
        if (argument_index == 3)
        {
            if (p[0] > 500 && p[0] < 2000 &&
                    p[1] > 0 && p[1] < 100)
            {
                writeProtocolHead(PROTOCOL_ESC_NEUTRAL, endpoint);
                activesettings.esc_neutral_pos = p[0];
                activesettings.esc_neutral_range = p[1];
                writeProtocolOK(endpoint);
            }
            else
            {
                writeProtocolError(ERROR_INVALID_VALUE, endpoint);
            }
        }
        else if (argument_index == 1)
        {
            writeProtocolHead(PROTOCOL_ESC_NEUTRAL, endpoint);
            writeProtocolInt(activesettings.esc_neutral_pos, endpoint);
            writeProtocolInt(activesettings.esc_neutral_range, endpoint);
            writeProtocolOK(endpoint);
        }
        else
        {
            writeProtocolError(ERROR_NUMBER_OF_ARGUMENTS, endpoint);
        }
        break;
    }
    case PROTOCOL_ROTATION_DIR:
    {
        int16_t p;
        argument_index = sscanf(commandline, "%c %hd", &command, &p);
        if (argument_index == 2)
        {
            if (p == 1 || p == -1 || p == 0)
            {
                writeProtocolHead(PROTOCOL_ROTATION_DIR, endpoint);
                activesettings.esc_direction = p;
                writeProtocolOK(endpoint);
            }
            else
            {
                writeProtocolError(ERROR_ESC_DIRECTION, endpoint);
            }
        }
        else
        {
            writeProtocolHead(PROTOCOL_ROTATION_DIR, endpoint);
            writeProtocolInt(activesettings.esc_direction, endpoint);
            writeProtocolOK(endpoint);
        }
        break;
    }
    case PROTOCOL_INPUT_SOURCE:
    {
        int16_t p;
        argument_index = sscanf(commandline, "%c %hd", &command, &p);
        if (argument_index == 2)
        {
            if (p == 0)
            {
                writeProtocolHead(PROTOCOL_INPUT_SOURCE, endpoint);
                activesettings.receivertype = RECEIVER_TYPE_SUMPPM;
                writeProtocolOK(endpoint);
            }
            else if (p == 1)
            {
                writeProtocolHead(PROTOCOL_INPUT_SOURCE, endpoint);
                activesettings.receivertype = RECEIVER_TYPE_SBUS;
                writeProtocolOK(endpoint);
            }
            else
            {
                writeProtocolError(ERROR_INVALID_VALUE, endpoint);
            }
        }
        else
        {
            writeProtocolHead(PROTOCOL_INPUT_SOURCE, endpoint);
            writeProtocolInt(activesettings.receivertype, endpoint);
            if (activesettings.receivertype == RECEIVER_TYPE_SUMPPM)
            {
                writeProtocolText("(Receiver is of type Sum-PPM) ", endpoint);
            }
            else
            {
                writeProtocolText("(Receiver is of type SBus) ", endpoint);
            }
            writeProtocolText("\r\nStoring all settings to eeprom\r\nReboot device to make the new receiver type active.\r\n", endpoint);
            writeProtocolOK(endpoint);
            evaluateCommand(PROTOCOL_EEPROM_WRITE);
        }
        break;
    }
    case PROTOCOL_HELP:
        printHelp(endpoint);
        break;
    case PROTOCOL_MODE:
    {
        int16_t p;
        argument_index = sscanf(commandline, "%c %hd", &command, &p);
        if (argument_index == 2)
        {
            if (p == MODE_ABSOLUTE_POSITION || p == MODE_LIMITER || p == MODE_LIMITER_ENDPOINTS || p == MODE_PASSTHROUGH)
            {
                writeProtocolHead(PROTOCOL_MODE, endpoint);
                activesettings.mode = p;
                resetThrottle();
                resetPosTarget();
                writeProtocolOK(endpoint);
            }
            else
            {
                writeProtocolError(ERROR_MODE, endpoint);
            }
        }
        else
        {
            writeProtocolHead(PROTOCOL_MODE, endpoint);
            writeProtocolInt(activesettings.mode, endpoint);
            writeProtocolText("...", endpoint);
            writeProtocolText(getCurrentModeLabel(activesettings.mode), endpoint);
            writeProtocolOK(endpoint);
        }
        break;
    }
    case PROTOCOL_SETTINGS:
    {
        writeProtocolHead(PROTOCOL_SETTINGS, endpoint);
        writeProtocolOK(endpoint);
        printActiveSettings(endpoint);
        break;
    }
    case PROTOCOL_D_CYCLES:
    {
        printDebugCycles(endpoint);
        break;
    }
    case PROTOCOL_BINARY:
    {
        uint8_t * tempsettingspointer;
        if (strlen(commandline) > 10)
        {
            uint16_t pos = 1;
            settings_t tempsettings;
            tempsettingspointer = (uint8_t *) &tempsettings;
            // Move forward to the first non-blank character after the command
            while (pos < strlen(commandline) && pos < sizeof(commandline) && commandline[pos] == ' ')
            {
                pos++;
            }

            /*
             * Two chars are one byte
             */
            uint16_t activesettingsoffset = 0;
            while (pos < strlen(commandline)-1 && pos < sizeof(commandline)-1 && activesettingsoffset < sizeof(activesettings))
            {
                int b1 = hexDigitToInt(commandline[pos++]);
                int b2 = hexDigitToInt(commandline[pos++]);
                if (b1 != -1 && b2 != -1)
                {
                    uint8_t b3 = b1*16 + b2;
                    tempsettingspointer[activesettingsoffset++] = b3;
                }
                else
                {
                    writeProtocolError(ERROR_INVALID_VALUE, endpoint);
                    return;
                }
            }
            if (activesettingsoffset < sizeof(tempsettings))
            {
                writeProtocolError(ERROR_INVALID_VALUE, endpoint);
                return;
            }
            else
            {
                memcpy(&activesettings, &tempsettings, sizeof(tempsettings));
            }
        }
        writeProtocolHead(PROTOCOL_BINARY, endpoint);
        writeProtocolChar(' ', endpoint);
        tempsettingspointer = (uint8_t *) &activesettings;
        for (int pos=0; pos<sizeof(activesettings); pos++)
        {
            writeProtocolHex(tempsettingspointer[pos], endpoint);
        }
        writeProtocolOK(endpoint);
        break;
    }
    default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
        writeProtocolError(ERROR_UNKNOWN_COMMAND, endpoint);
        break;
    }
}

void printHelp(Endpoints endpoint)
{
    PrintlnSerial(endpoint);
    PrintlnSerial(endpoint);

    PrintlnSerial_string(controllerstatus.boottext_eeprom, endpoint);

    PrintSerial_string("Current Mode: ", endpoint);
    PrintlnSerial_string(getCurrentModeLabel(activesettings.mode), endpoint);

    PrintSerial_string("Current Receiver type: ", endpoint);
    if (activesettings.receivertype == RECEIVER_TYPE_SUMPPM)
    {
        PrintlnSerial_string("Sum-PPM", endpoint);
    }
    else
    {
        PrintlnSerial_string("SBus", endpoint);
    }

    PrintSerial_string("Current CableCam operation mode: ", endpoint);
    PrintSerial_string(getSafeModeLabel(), endpoint);

    PrintlnSerial(endpoint);
    PrintlnSerial(endpoint);
    PrintlnSerial(endpoint);
    PrintlnSerial(endpoint);
    PrintlnSerial_string("Possible commands are", endpoint);
    PrintlnSerial(endpoint);

    PrintlnSerial_string("$a [<int> <int>]                        set or print maximum allowed acceleration in normal and programming mode", endpoint);
    PrintlnSerial_string("$g [<double>]                           set or print the max positional error -> exceeding it causes an emergency stop", endpoint);
    PrintlnSerial_string("$i [[[[<int> <int> <int>]               set or print input channels for Speed, Programming Switch, Endpoint Switch,...", endpoint);
    PrintlnSerial_string("      <int>] <int>] <int>]              ...Max Accel, Max Speed, Mode", endpoint);
    PrintlnSerial_string("$I [<int>]                              set or print input source 0..SumPPM", endpoint);
    PrintlnSerial_string("                                                                  1..SBus", endpoint);
    PrintlnSerial_string("$m [<int>]                              set or print the mode 0..positional", endpoint);
    PrintlnSerial_string("                                                              1..passthrough", endpoint);
    PrintlnSerial_string("                                                              2..passthrough with speed limits", endpoint);
    PrintlnSerial_string("                                                              3..passthough with speed limits & end points", endpoint);
    PrintlnSerial_string("$n [<int> <int>]                        set or print receiver neutral pos and +-range", endpoint);
    PrintlnSerial_string("$N [<int> <int>]                        set or print ESC output neutral pos and +-range", endpoint);
    PrintlnSerial_string("$p                                      print positions", endpoint);
    PrintlnSerial_string("$r [<int>]                              set or print rotation direction of the ESC output, either +1 or -1", endpoint);
    PrintlnSerial_string("$S                                      print all settings", endpoint);
    PrintlnSerial_string("$v [<int> <int>]                        set or print maximum allowed speed in normal and programming mode", endpoint);
    PrintlnSerial_string("$w                                      write settings to eeprom", endpoint);

    PrintlnSerial(endpoint);
    PrintlnSerial_string("$1 [<double>]                           set or print Kp for PID controller", endpoint);
    PrintlnSerial_string("$2 [<double>]                           set or print Ki for PID controller", endpoint);
    PrintlnSerial_string("$3 [<double>]                           set or print Kd for PID controller", endpoint);
    PrintlnSerial_string("$c [<double> <double> <double>]         set or print all three PID values", endpoint);
    PrintlnSerial_string("$f [<double>]                           set or print stick-to-hall-speed factor", endpoint);
    PrintlnSerial(endpoint);
}

void printActiveSettings(Endpoints endpoint)
{
    PrintlnSerial(endpoint);
    PrintlnSerial(endpoint);
    PrintlnSerial(endpoint);
    PrintlnSerial_string("Active settings", endpoint);
    PrintlnSerial(endpoint);
    PrintlnSerial(endpoint);

    PrintSerial_string("Version of the stored settings: ", endpoint);
    PrintlnSerial_string(activesettings.version, endpoint);
    PrintlnSerial(endpoint);
    PrintlnSerial(endpoint);

    PrintSerial_string("Controller status: ", endpoint);
    switch (controllerstatus.monitor)
    {
    case EMERGENCYBRAKE:
        PrintlnSerial_string("Emergency brake on", endpoint);
        break;
    case FREE:
        PrintlnSerial_string("Free running", endpoint);
        break;
    case ENDPOINTBRAKE:
        PrintlnSerial_string("Endpoint brake on", endpoint);
        break;
    }
    PrintlnSerial(endpoint);
    PrintlnSerial(endpoint);

    PrintlnSerial_string("Currently the endpoint limits are configured as ", endpoint);
    int32_t pos = ENCODER_VALUE;

    if (activesettings.pos_start > activesettings.pos_end)
    {
        /* start is always smaller than end anyway but below prints rely on that, hence doublecheck. */
        int32_t tmp = activesettings.pos_start;
        activesettings.pos_start = activesettings.pos_end;
        activesettings.pos_end = tmp;
    }

    PrintSerial_string(" ", endpoint);
    if (pos < activesettings.pos_start)
    {
        PrintSerial_long(pos, endpoint);
        PrintSerial_string("--- ", endpoint);
    }
    PrintSerial_long((int32_t) activesettings.pos_start, endpoint);
    PrintSerial_string(" <--------- ", endpoint);
    if (activesettings.pos_start <= pos && pos <= activesettings.pos_end)
    {
        PrintSerial_long(pos, endpoint);
    }
    PrintSerial_string(" ---------> ", endpoint);
    PrintSerial_long((int32_t) activesettings.pos_end, endpoint);
    if (pos > activesettings.pos_end)
    {
        PrintSerial_string("--- ", endpoint);
        PrintSerial_long(pos, endpoint);
    }
    PrintlnSerial(endpoint);
    PrintlnSerial(endpoint);
    PrintlnSerial(endpoint);


    PrintlnSerial_string("Neutral position and range of the RC:", endpoint);
    PrintSerial_string("  Neutral point:", endpoint);
    PrintSerial_int(activesettings.stick_neutral_pos, endpoint);
    PrintSerial_string(" +-", endpoint);
    PrintlnSerial_int(activesettings.stick_neutral_range, endpoint);
    PrintlnSerial(endpoint);
    PrintlnSerial(endpoint);


    PrintlnSerial_string("Ramp filter:", endpoint);
    PrintSerial_string("  Operational mode ", endpoint);
    if (controllerstatus.safemode == OPERATIONAL)
    {
        PrintlnSerial_string("(active)", endpoint);
    }
    else
    {
        PrintlnSerial(endpoint);
    }
    if (activesettings.mode != MODE_LIMITER && activesettings.mode != MODE_LIMITER_ENDPOINTS && activesettings.mode != MODE_ABSOLUTE_POSITION)
    {
        PrintSerial_string("Current mode ", endpoint);
        PrintSerial_string(getSafeModeLabel(), endpoint);
        PrintlnSerial_string(" does not use the accel and speed limits", endpoint);
    }
    PrintSerial_string("  Limit stick value changes to ", endpoint);
    PrintSerial_int(activesettings.stick_max_accel, endpoint);
    PrintSerial_string("* 5 every second at max", endpoint);
    PrintSerial_string(" and the max value is +-", endpoint);
    PrintSerial_int(activesettings.stick_max_speed, endpoint);
    PrintlnSerial_string(" around its neutral range", endpoint);
    PrintlnSerial(endpoint);

    PrintSerial_string("  Programming mode ", endpoint);
    if (controllerstatus.safemode == PROGRAMMING)
    {
        PrintlnSerial_string("(active)", endpoint);
    }
    else
    {
        PrintlnSerial(endpoint);
    }
    PrintSerial_string("  Limit stick value changes to ", endpoint);
    PrintSerial_int(activesettings.stick_max_accel_safemode, endpoint);
    PrintSerial_string(" * 5 every second at max", endpoint);
    PrintSerial_string(" and the max value is +-", endpoint);
    PrintSerial_int(activesettings.stick_max_speed_safemode, endpoint);
    PrintlnSerial_string(" around its neutral range", endpoint);
    PrintlnSerial(endpoint);
    PrintlnSerial(endpoint);


    PrintlnSerial_string("ESC output signal is generated around the neutral position with range", endpoint);
    PrintSerial_string("  Neutral point:", endpoint);
    PrintSerial_int(activesettings.esc_neutral_pos, endpoint);
    PrintSerial_string(" +-", endpoint);
    PrintlnSerial_int(activesettings.esc_neutral_range, endpoint);
    PrintlnSerial(endpoint);
    PrintlnSerial(endpoint);


    PrintlnSerial_string("The sensor direction and the motor direction might not be the same. If the CableCam", endpoint);
    PrintlnSerial_string("overshot the end point, pulling the stick in reverse direction to get the CableCam back between", endpoint);
    PrintlnSerial_string("the end points should be allowed, moving it further out should not. But which direction is it?", endpoint);
    PrintSerial_string("  Motor/ESC direction:", endpoint);
    PrintSerial_int(activesettings.esc_direction, endpoint);
    switch (activesettings.esc_direction)
    {
        case 1: PrintSerial_string("(positive stick = hall sensor counts up)", endpoint); break;
        case -1: PrintSerial_string("(positive stick = hall sensor counts down)", endpoint); break;
        default: PrintSerial_string("(not yet decided, drive to pos>500 on a horizontal rope to set it automatically)", endpoint); break;
    }
    PrintlnSerial(endpoint);
    PrintlnSerial(endpoint);
    PrintlnSerial(endpoint);


    PrintlnSerial_string("The controller mode.", endpoint);
    PrintSerial_string("  Mode", endpoint);
    PrintSerial_int(activesettings.mode, endpoint);
    PrintSerial_string("...", endpoint);
    PrintlnSerial_string(getCurrentModeLabel(activesettings.mode), endpoint);

    PrintlnSerial(endpoint);
    PrintlnSerial(endpoint);
    PrintlnSerial(endpoint);
}

void printDebugCycles(Endpoints endpoint)
{
    /*
     * Oldest values first, if there are any
     */
     int16_t last = controllerstatus.cyclemonitor_position;
    for (int i=last; i < CYCLEMONITOR_SAMPLE_COUNT; i++)
    {
        cyclemonitor_t * sample = & controllerstatus.cyclemonitor[i];
        if (sample->tick != 0)
        {
            PrintSerial_long(sample->tick, endpoint);
            PrintSerial_int(sample->stick, endpoint);
            PrintSerial_int(sample->esc, endpoint);
            PrintSerial_double(sample->speed, endpoint);
            PrintSerial_double(sample->pos, endpoint);
            PrintlnSerial_double(sample->distance_to_stop, endpoint);
            if (i % 20 == 0)
            {
                USBPeriodElapsed(); // it is too much data, we need to flush once a while
            }
        }
    }

    for (int i=0; i<last; i++)
    {
        cyclemonitor_t * sample = & controllerstatus.cyclemonitor[i];
        PrintSerial_long(sample->tick, endpoint);
        PrintSerial_int(sample->stick, endpoint);
        PrintSerial_int(sample->esc, endpoint);
        PrintSerial_double(sample->speed, endpoint);
        PrintSerial_double(sample->pos, endpoint);
        PrintlnSerial_double(sample->distance_to_stop, endpoint);
        if (i % 20 == 0)
        {
            USBPeriodElapsed(); // it is too much data, we need to flush once a while
        }
    }
}

uint8_t is_ok(uint8_t *btchar_string, uint8_t * btchar_string_length)
{
    if (*btchar_string_length >= 2 && btchar_string[0] == 'O' && btchar_string[1] == 'K')
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


char * getSafeModeLabel()
{
    switch (controllerstatus.safemode)
    {
        case INVALID_RC: return "INVALID_RC";
        case NOT_NEUTRAL_AT_STARTUP: return "Stick not in Neutral";
        case PROGRAMMING: return "Endpoint Programming mode";
        case OPERATIONAL: return "Operational";
        default: return "???unknown safemode???";
    }
}

char * getCurrentModeLabel(uint8_t mode)
{
    switch (mode)
    {
        case MODE_ABSOLUTE_POSITION: return "absolute position";
        case MODE_LIMITER: return "passthrough with limiter";
        case MODE_LIMITER_ENDPOINTS: return "passthrough with limiter & end points";
        case MODE_PASSTHROUGH: return "passthrough";
        default : return "????current mode???";
    }
}
