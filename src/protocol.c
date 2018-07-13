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
#include "bluetooth.h"
#include "main.h"
#include "vesc.h"
#include "systembootloader.h"


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



static const char * error_string[] = {"other errors",
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
                                "allowed modes are 1 -> passthrough, 2 -> plus limiter, 3 -> plus endpoints",
                                "configuration of the bluetooth module failed",
                                "cannot configure serial bluetooth module from its own serial line",
                                "invalid value for provided argument(s)"
                               };

settings_t activesettings;
settings2_t semipermanentsettings;

controllerstatus_t controllerstatus;

extern TIM_HandleTypeDef htim1;
extern getvalues_t vescvalues;
extern sbusFrame_t sBusFrameGimbal;

static uint8_t checksum_response;

void evaluateCommand(Endpoints endpoint, char commandlinebuffer[]);
void writeProtocolError(uint8_t, Endpoints endpoint);
void writeProtocolErrorText(char *, Endpoints endpoint);
void writeProtocolHead(char command, Endpoints endpoint);
void writeProtocolChar(char c, Endpoints endpoint);
void writeProtocolDouble(double v, Endpoints endpoint);
void writeProtocolFloat(float v, Endpoints endpoint);

void writeProtocolOK(Endpoints endpoint);
void writeProtocolInt(int16_t v, Endpoints endpoint);
void writeProtocolLong(int32_t v, Endpoints endpoint);
void writeProtocolHex(uint8_t v, Endpoints endpoint);
void writeProtocolText(const char * text, Endpoints endpoint);
int hexDigitToInt(char digit);

uint8_t check_for_neutral_is_in_the_middle(sbusData_t * rcavg, sbusData_t * rcneutral, uint8_t channel, Endpoints endpoint);
uint8_t check_for_no_input_change(sbusData_t * rcmin, sbusData_t * rcmax, Endpoints endpoint);
uint8_t check_for_multiple_channels_changed(sbusData_t * rcmin, sbusData_t * rcmax, uint8_t *channel, Endpoints endpoint);
void getDutyValues(sbusData_t * rcmin, sbusData_t * rcmax, sbusData_t * rcavg, uint32_t timeout, Endpoints endpoint);
void printChannelDutyValues(sbusData_t * rcmin, sbusData_t * rcmax, Endpoints endpoint);

void printCurrentRCInput(Endpoints endpoint);
void printCurrentSBusOut(Endpoints endpoint);

int16_t valid_channelassignment(int16_t c);

typedef  void (*pFunction)(void);

void initProtocol()
{
    controllerstatus.vesc_config = false;
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
    PrintSerial_string("$", endpoint);
    PrintSerial_char(command, endpoint);
    checksum_response = command;
}

void writeProtocolText(const char * text, Endpoints endpoint)
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
    snprintf(bufpd, sizeof(bufpd), " %1.3lf", v);
    int k = 0;
    while(k < 40 && bufpd[k] != 0)
    {
        checksum_response ^= bufpd[k];
        k++;
    }
    PrintSerial_string(bufpd, endpoint);
}

void writeProtocolFloat(float v, Endpoints endpoint)
{
    char bufpd[80];
    snprintf(bufpd, sizeof(bufpd), " %1.3f", (double) v);
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
    snprintf(bufpd, sizeof(bufpd), " %d", v);
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
    snprintf(bufpd, sizeof(bufpd), " %ld", v);
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

/*
 * SerialCom gets one string like "$i 23*5c4a\n"
 * It is the task of the previous command to assemble one line of commands, e.g. by handling backspace chars etc.
 */
void serialCom(Endpoints endpoint, char commandlinebuffer[])
{
    char c;
    uint16_t pos = 0;
    uint16_t pos_command_end = 0; // The position the command string ends, *-char, \r\n and checksum bytes not counting. This is where the string will be null terminated
    uint8_t checksum;
    uint8_t checksum_received;

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

        else if (c == '\n' || c == '\r')
        {
            /*
             * The command line string contains a null-terminated string like "$I 1".
             * It is stripped by the *-char and the checksum
             */
            if (pos_command_end == 0)
            {
                // no checksum hence terminate with the \r or \n char
                commandlinebuffer[pos - 1] = 0; // Null-terminate the string
            }
            else
            {
                commandlinebuffer[pos_command_end] = 0; // Null-terminate the string
            }
            if (c_state == COMMAND_CHECKSUM && checksum != checksum_received)
            {
                writeProtocolError(ERROR_CHECKSUM, endpoint);
            }
            else
            {
                evaluateCommand(endpoint, commandlinebuffer);
            }
            checksum = 0;
            c_state = COMMAND_IDLE;
        }
        /*
         * Everything after a * character is ignored as this should be the checksum only
         */
        else if (c== '*')
        {
            c_state = COMMAND_CHECKSUMBYTE1;
            pos_command_end = pos-1;
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
        else if (c_state == COMMAND_START)
        {
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

void evaluateCommand(Endpoints endpoint, char commandlinebuffer[])
{
    char command = commandlinebuffer[1];
    int16_t argument_index;

    switch(command)
    {
    case PROTOCOL_MAX_ACCEL:
    {
        float p[2];
        argument_index = sscanf(&commandlinebuffer[2], "%f %f", &p[0], &p[1]);
        if (argument_index == 2)
        {
            if (p[0] > 0.0f && p[1] > 0.0f && p[0] < 1.0f && p[1] < 1.0f)
            {
                activesettings.stick_max_accel = p[0]*CONTROLLERLOOPTIME_FLOAT;
                activesettings.stick_max_accel_safemode = p[1]*CONTROLLERLOOPTIME_FLOAT;
                writeProtocolHead(PROTOCOL_MAX_ACCEL, endpoint);
                writeProtocolOK(endpoint);
            }
            else
            {
                writeProtocolError(ERROR_INVALID_VALUE, endpoint);
            }
        }
        else if (argument_index <= 0)
        {
            writeProtocolHead(PROTOCOL_MAX_ACCEL, endpoint);
            writeProtocolFloat(activesettings.stick_max_accel/CONTROLLERLOOPTIME_FLOAT, endpoint);
            writeProtocolFloat(activesettings.stick_max_accel_safemode/CONTROLLERLOOPTIME_FLOAT, endpoint);
            writeProtocolOK(endpoint);
            PrintSerial_string("Current value: ", endpoint);
            PrintlnSerial_float(controllerstatus.accel_limiter, endpoint);
        }
        else
        {
            writeProtocolError(ERROR_NUMBER_OF_ARGUMENTS, endpoint);
        }
        break;
    }
    case PROTOCOL_MAX_ERROR_DIST:
    {
        float d;
        argument_index = sscanf(&commandlinebuffer[2], "%f", &d);
        if (argument_index == 1)
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
            writeProtocolFloat(activesettings.max_position_error, endpoint);
            writeProtocolOK(endpoint);
        }
        break;
    }
    case PROTOCOL_POS:
    {
        writeProtocolHead(PROTOCOL_POS, endpoint);
        writeProtocolLong(semipermanentsettings.pos_start, endpoint);
        writeProtocolLong(semipermanentsettings.pos_end, endpoint);
        writeProtocolFloat(controllerstatus.pos, endpoint);
        writeProtocolFloat(controllerstatus.speed, endpoint);
        writeProtocolOK(endpoint);
        break;
    }
    case PROTOCOL_MAX_SPEED:
    {
        float p[2];
        argument_index = sscanf(&commandlinebuffer[2], "%f %f", &p[0], &p[1]);

        if (argument_index == 2)
        {
            if (p[0] > 0.0f && p[1] > 0.0f && p[0] <= 1.0f && p[1] < 1.0f)
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
        else if (argument_index <= 0)
        {
            writeProtocolHead(PROTOCOL_MAX_SPEED, endpoint);
            writeProtocolFloat(activesettings.stick_max_speed, endpoint);
            writeProtocolFloat(activesettings.stick_max_speed_safemode, endpoint);
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
        write_errors += eeprom_write_sector_safe((uint8_t*) &semipermanentsettings, sizeof(semipermanentsettings), EEPROM_SECTOR_FOR_SEMIPERMANENTSETTINGS);
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
    case PROTOCOL_EEPROM_VOID:
    {
        eeprom_erase(EEPROM_SECTOR_FOR_SETTINGS);
        eeprom_erase(EEPROM_SECTOR_FOR_SEMIPERMANENTSETTINGS);
        writeProtocolHead(PROTOCOL_EEPROM_VOID, endpoint);
        writeProtocolText("\r\nEEPROM erased", endpoint);
        writeProtocolOK(endpoint);
        break;
    }
    case PROTOCOL_INPUT_CHANNELS:
    {
        /*
         * Note, the protocol does remap channel1 to chan0
         */
        int16_t p[8];
        argument_index = sscanf(&commandlinebuffer[2], "%hd %hd %hd %hd %hd %hd %hd %hd", &p[0], &p[1], &p[2], &p[3], &p[4], &p[5], &p[6], &p[7]);

        if (argument_index >= 1)
        {
            if (p[0] > 0 && p[0] <= SBUS_MAX_CHANNEL)
            {
                activesettings.rc_channel_speed = p[0]-1;

                writeProtocolHead(PROTOCOL_INPUT_CHANNELS, endpoint);
                writeProtocolInt(activesettings.rc_channel_speed+1, endpoint);
                if (argument_index >= 2)
                {
                    activesettings.rc_channel_programming = valid_channelassignment(p[1])-1;
                    writeProtocolInt(activesettings.rc_channel_programming+1, endpoint);
                }
                if (argument_index >= 3)
                {
                    activesettings.rc_channel_endpoint = valid_channelassignment(p[2])-1;
                    writeProtocolInt(activesettings.rc_channel_endpoint+1, endpoint);
                }
                if (argument_index >= 4)
                {
                    activesettings.rc_channel_max_accel = valid_channelassignment(p[3])-1;
                    writeProtocolInt(activesettings.rc_channel_max_accel+1, endpoint);
                }
                if (argument_index >= 5)
                {
                    activesettings.rc_channel_max_speed = valid_channelassignment(p[4])-1;
                    writeProtocolInt(activesettings.rc_channel_max_speed+1, endpoint);
                }
                if (argument_index >= 6)
                {
                    activesettings.rc_channel_mode = valid_channelassignment(p[5])-1;
                    writeProtocolInt(activesettings.rc_channel_mode+1, endpoint);
                }
                if (argument_index >= 7)
                {
                    activesettings.rc_channel_aux = valid_channelassignment(p[6])-1;
                    writeProtocolInt(activesettings.rc_channel_aux+1, endpoint);
                }
                if (argument_index >= 8)
                {
                    activesettings.rc_channel_play = valid_channelassignment(p[7])-1;
                    writeProtocolInt(activesettings.rc_channel_play+1, endpoint);
                }
                writeProtocolOK(endpoint);
            }
            else
            {
                writeProtocolError(ERROR_INVALID_VALUE, endpoint);
            }
        }
        else if (argument_index <= 0)
        {
            writeProtocolHead(PROTOCOL_INPUT_CHANNELS, endpoint);
            writeProtocolInt(activesettings.rc_channel_speed+1, endpoint);
            writeProtocolInt(activesettings.rc_channel_programming+1, endpoint);
            writeProtocolInt(activesettings.rc_channel_endpoint+1, endpoint);
            writeProtocolInt(activesettings.rc_channel_max_accel+1, endpoint);
            writeProtocolInt(activesettings.rc_channel_max_speed+1, endpoint);
            writeProtocolInt(activesettings.rc_channel_mode+1, endpoint);
            writeProtocolInt(activesettings.rc_channel_aux+1, endpoint);
            writeProtocolInt(activesettings.rc_channel_play+1, endpoint);
            writeProtocolOK(endpoint);

            printCurrentRCInput(endpoint);
        }
        else
        {
            writeProtocolError(ERROR_NUMBER_OF_ARGUMENTS, endpoint);
        }
        break;
    }
    case PROTOCOL_NEUTRAL:
    {
        int16_t p[3];
        argument_index = sscanf(&commandlinebuffer[2], "%hd %hd %hd", &p[0], &p[1], &p[2]);
        if (argument_index == 3)
        {
            if (p[0] > 500 && p[0] < 2000 &&
                    p[1] > 0 && p[1] < 100 &&
                    p[2] > 300 && p[2] < 1400)
            {
                writeProtocolHead(PROTOCOL_NEUTRAL, endpoint);
                activesettings.stick_neutral_pos = p[0];
                activesettings.stick_neutral_range = p[1];
                activesettings.stick_value_range = p[2];
                writeProtocolOK(endpoint);
            }
            else
            {
                writeProtocolError(ERROR_INVALID_VALUE, endpoint);
            }
        }
        else if (argument_index <= 0)
        {
            writeProtocolHead(PROTOCOL_NEUTRAL, endpoint);
            writeProtocolInt(activesettings.stick_neutral_pos, endpoint);
            writeProtocolInt(activesettings.stick_neutral_range, endpoint);
            writeProtocolInt(activesettings.stick_value_range, endpoint);
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
        int16_t p[3];
        argument_index = sscanf(&commandlinebuffer[2], "%hd %hd %hd", &p[0], &p[1], &p[2]);
        if (argument_index == 3)
        {
            if (p[0] > 500 && p[0] < 2000 &&
                    p[1] > 0 && p[1] < 100 && p[2] > 300 && p[2] < 900)
            {
                writeProtocolHead(PROTOCOL_ESC_NEUTRAL, endpoint);
                activesettings.esc_neutral_pos = p[0];
                activesettings.esc_neutral_range = p[1];
                activesettings.esc_value_range = p[2];
                writeProtocolOK(endpoint);
            }
            else
            {
                writeProtocolError(ERROR_INVALID_VALUE, endpoint);
            }
        }
        else if (argument_index <= 0)
        {
            writeProtocolHead(PROTOCOL_ESC_NEUTRAL, endpoint);
            writeProtocolInt(activesettings.esc_neutral_pos, endpoint);
            writeProtocolInt(activesettings.esc_neutral_range, endpoint);
            writeProtocolInt(activesettings.esc_value_range, endpoint);
            writeProtocolOK(endpoint);
        }
        else
        {
            writeProtocolError(ERROR_NUMBER_OF_ARGUMENTS, endpoint);
        }
        break;
    }
    case PROTOCOL_AUX_NEUTRAL:
    {
        int16_t p[3];
        argument_index = sscanf(&commandlinebuffer[2], "%hd %hd %hd", &p[0], &p[1], &p[2]);
        if (argument_index == 3)
        {
            if (p[0] > 500 && p[0] < 2000 &&
                    p[1] > 0 && p[1] < 100 && p[2] > 300 && p[2] < 900)
            {
                writeProtocolHead(PROTOCOL_AUX_NEUTRAL, endpoint);
                activesettings.aux_neutral_pos = p[0];
                activesettings.aux_neutral_range = p[1];
                activesettings.aux_value_range = p[2];
                writeProtocolOK(endpoint);
            }
            else
            {
                writeProtocolError(ERROR_INVALID_VALUE, endpoint);
            }
        }
        else if (argument_index <= 0)
        {
            writeProtocolHead(PROTOCOL_AUX_NEUTRAL, endpoint);
            writeProtocolInt(activesettings.aux_neutral_pos, endpoint);
            writeProtocolInt(activesettings.aux_neutral_range, endpoint);
            writeProtocolInt(activesettings.aux_value_range, endpoint);
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
        argument_index = sscanf(&commandlinebuffer[2], "%hd", &p);
        if (argument_index == 1)
        {
            if (p == 1 || p == -1 || p == 0)
            {
                writeProtocolHead(PROTOCOL_ROTATION_DIR, endpoint);
                activesettings.esc_direction = (float) p;
                writeProtocolInt( (int16_t) activesettings.esc_direction, endpoint);
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
            writeProtocolInt( (int16_t) activesettings.esc_direction, endpoint);
            writeProtocolOK(endpoint);
        }
        break;
    }
    case PROTOCOL_VESC_MAX_ERPM:
    {
        int32_t p;
        argument_index = sscanf(&commandlinebuffer[2], "%ld", &p);
        if (argument_index == 1)
        {
            if (p > 500 && p <= 100000)
            {
                writeProtocolHead(PROTOCOL_VESC_MAX_ERPM, endpoint);
                activesettings.vesc_max_erpm = p;
                writeProtocolOK(endpoint);
            }
            else
            {
                writeProtocolError(ERROR_INVALID_VALUE, endpoint);
            }
        }
        else
        {
            writeProtocolHead(PROTOCOL_VESC_MAX_ERPM, endpoint);
            writeProtocolLong(activesettings.vesc_max_erpm, endpoint);
            writeProtocolOK(endpoint);
        }
        break;
    }
    case PROTOCOL_INPUT_SOURCE:
    {
        int16_t p;
        argument_index = sscanf(&commandlinebuffer[2], "%hd", &p);
        if (argument_index == 1)
        {
            if (p == 0)
            {
                writeProtocolHead(PROTOCOL_INPUT_SOURCE, endpoint);
                activesettings.receivertype = RECEIVER_TYPE_SUMPPM;
                initPPMReceiver();
                writeProtocolOK(endpoint);
            }
            else if (p == 1)
            {
                writeProtocolHead(PROTOCOL_INPUT_SOURCE, endpoint);
                activesettings.receivertype = RECEIVER_TYPE_SBUS;
                initSBusReceiver();
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
            writeProtocolOK(endpoint);
        }
        break;
    }
    case PROTOCOL_HELP:
        printHelp(endpoint);
        break;
    case PROTOCOL_MODE:
    {
        int16_t p;
        argument_index = sscanf(&commandlinebuffer[2], "%hd", &p);
        if (argument_index == 1)
        {
            if (p == MODE_LIMITER || p == MODE_LIMITER_ENDPOINTS || p == MODE_PASSTHROUGH)
            {
                writeProtocolHead(PROTOCOL_MODE, endpoint);
                activesettings.mode = p;
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
    case PROTOCOL_DEBUG:
    {
        char c;
        argument_index = sscanf(&commandlinebuffer[2], "%1s", &c);
        if (argument_index == 1)
        {
            switch (c)
            {
            case 's':
                PrintSerial_string("SBUS packets:", endpoint);
                PrintSerial_int(controllerstatus.dsbus_pause, endpoint);
                PrintSerial_string(", valid:", endpoint);
                PrintSerial_int(controllerstatus.dsbus_valid, endpoint);
                PrintSerial_string(", errors:", endpoint);
                PrintSerial_int(controllerstatus.dsbus_errors, endpoint);
                PrintSerial_string(" (frame errors:", endpoint);
                PrintSerial_int(controllerstatus.dsbus_frame_errors, endpoint);
                PrintSerial_string(", parity errors:", endpoint);
                PrintSerial_int(controllerstatus.dsbus_parity_errors, endpoint);
                PrintSerial_string(", noise errors:", endpoint);
                PrintSerial_int(controllerstatus.dsbus_noise_errors, endpoint);
                PrintSerial_string(", overrun errors:", endpoint);
                PrintSerial_int(controllerstatus.dsbus_overrun_errors, endpoint);
                PrintlnSerial_string(")", endpoint);

                PrintSerial_string("Last packet:", endpoint);
                PrintlnSerial_hexstring(controllerstatus.dsbus, SBUS_FRAME_SIZE, endpoint);
                break;
            case 'p':
                PrintSumPPMRawData(endpoint);
                break;
            default:
                PrintlnSerial_string("unknown parameter, what to debug?", endpoint);
                break;
            }
        }
        else
        {
            PrintlnSerial_string("Missing argument what to debug", endpoint);
        }
        break;
    }
    case PROTOCOL_BINARY:
    {
        uint8_t * tempsettingspointer;
        if (strlen(commandlinebuffer) > 10)
        {
            uint16_t pos = 2;
            settings_t tempsettings;
            tempsettingspointer = (uint8_t *) &tempsettings;
            // Move forward to the first non-blank character after the command
            while (pos < strlen(commandlinebuffer) && pos < RXBUFFERSIZE-1 && commandlinebuffer[pos] == ' ')
            {
                pos++;
            }

            /*
             * Two chars are one byte
             */
            uint16_t activesettingsoffset = 0;
            while (pos < strlen(commandlinebuffer)-1 && pos < RXBUFFERSIZE-1 && activesettingsoffset < sizeof(activesettings))
            {
                int b1 = hexDigitToInt(commandlinebuffer[pos++]);
                int b2 = hexDigitToInt(commandlinebuffer[pos++]);
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
        for (size_t pos=0; pos<sizeof(activesettings); pos++)
        {
            writeProtocolHex(tempsettingspointer[pos], endpoint);
        }
        writeProtocolOK(endpoint);
        break;
    }
    case PROTOCOL_EXPO_FACTOR:
    {
        float d;
        argument_index = sscanf(&commandlinebuffer[2], "%f", &d);
        if (argument_index == 1)
        {
            if (d > 0.0f && d <= 1.0f)
            {
                activesettings.expo_factor = d;
                writeProtocolHead(PROTOCOL_EXPO_FACTOR, endpoint);
                writeProtocolOK(endpoint);
            }
            else
            {
                writeProtocolError(ERROR_INVALID_VALUE, endpoint);
                return;
            }
         }
        else
        {
            writeProtocolHead(PROTOCOL_EXPO_FACTOR, endpoint);
            writeProtocolFloat(activesettings.expo_factor, endpoint);
            writeProtocolOK(endpoint);
        }
        break;
    }
	case PROTOCOL_BLUETOOTH:
    {
        if (endpoint == EndPoint_USB) {
            writeProtocolHead(PROTOCOL_BLUETOOTH, endpoint);
            writeProtocolText("configure BT module:", endpoint);
            if (configure_bt_module() == 1) {
                writeProtocolOK(endpoint);
            } else {
                writeProtocolError(ERROR_BT_CONFIG_FAILED, endpoint);
            }
        } else {
            writeProtocolError(ERROR_BT_NOT_FROM_USB, endpoint);
        }
    }
    break;
	case PROTOCOL_VESC_BRAKE:
    {
        int16_t p[2];
        argument_index = sscanf(&commandlinebuffer[2], "%hd %hd", &p[0], &p[1]);
        if (argument_index == 2)
        {
            if (p[0] >= 0 && p[1] >= 0)
            {
                activesettings.vesc_brake_handbrake_max = 0; // not used
                activesettings.vesc_brake_handbrake_min = p[0];
                activesettings.vesc_brake_min_speed = p[1];
                writeProtocolHead(PROTOCOL_VESC_BRAKE, endpoint);
                writeProtocolOK(endpoint);
            }
            else
            {
                writeProtocolError(ERROR_INVALID_VALUE, endpoint);
                return;
            }
         }
        else
        {
            writeProtocolHead(PROTOCOL_VESC_BRAKE, endpoint);
            writeProtocolInt(activesettings.vesc_brake_handbrake_min, endpoint);
            writeProtocolInt(activesettings.vesc_brake_min_speed, endpoint);
            writeProtocolOK(endpoint);
        }
        break;
    }
    case PROTOCOL_VESC_STATUS:
    {
        /*
         * The VESC status is requested in every vesc set erpm call, hence the values should be current
         *      int16_t         temp_fet_10;                // 0x011a
         *      int16_t         temp_motor_10;              // 0x00f3
         *  	int32_t         avg_motor_current_100;      // 0x000001ba (7-10)
         *  	int32_t         avg_input_current_100;      // 0x00000066 (11-14)
         *      int32_t         avg_id_100;                 // 0x00000000 (15-18)
         *      int32_t         avg_iq_100;                 // 0xfffffe46 (19-22)
         *      int16_t         duty_now_1000;              // 0xfef8
         *      int32_t         rpm_1;                      // 0xffffe02c (25-28)
         *  	int16_t         v_in_10;                    // 0x0071
         *      int32_t         amp_hours_10000;            // 0x0000002e (31-34)
         *      int32_t         amp_hours_charged_10000;    // 0x00000000 (35-38)
         *      int32_t         watt_hours_10000;           // 0x00000212 (39-42)
         *      int32_t         watt_hours_charged_10000;   // 0x00000000 (43-46)
         *      int32_t         tachometer;                 // 0xffffe0bb (47-50)
         *      int32_t         tachometer_abs;             // 0x00003c2f (51-54)
         *      vesc_fault_code fault_code;                 // 00
         */
        writeProtocolHead(PROTOCOL_VESC_STATUS, endpoint);
        writeProtocolText("\r\nTempFET[C]:", endpoint);
        writeProtocolFloat(vesc_get_float(vescvalues.frame.temp_fet_10, 10.0f), endpoint);
        writeProtocolText("\r\nTempMotor[C]:", endpoint);
        writeProtocolFloat(vesc_get_float(vescvalues.frame.temp_motor_10, 10.0f), endpoint);
        writeProtocolText("\r\nAvgMot[A]:", endpoint);
        writeProtocolDouble(vesc_get_double(vescvalues.frame.avg_motor_current_100, 100.0), endpoint);
        writeProtocolText("\r\nAvgInput[A]:", endpoint);
        writeProtocolDouble(vesc_get_double(vescvalues.frame.avg_input_current_100, 100.0), endpoint);
        writeProtocolText("\r\nDuty[%]:", endpoint);
        writeProtocolFloat(vesc_get_float(vescvalues.frame.duty_now_1000, 1000.0f), endpoint);
        writeProtocolText("\r\nrpm:", endpoint);
        writeProtocolLong(vesc_get_long(vescvalues.frame.rpm_1), endpoint);
        writeProtocolText("\r\nBatt[V]:", endpoint);
        writeProtocolFloat(vesc_get_float(vescvalues.frame.v_in_10, 10.0f), endpoint);
        writeProtocolText("\r\nConsumed[mAh]:", endpoint);
        writeProtocolDouble(vesc_get_double(vescvalues.frame.amp_hours_10000, 10.0), endpoint);
        writeProtocolText("\r\nRegenBrake[mAh]:", endpoint);
        writeProtocolDouble(vesc_get_double(vescvalues.frame.amp_hours_charged_10000, 10.0), endpoint);
        writeProtocolText("\r\nConsumed[Wh]:", endpoint);
        writeProtocolDouble(vesc_get_double(vescvalues.frame.watt_hours_10000, 10000.0), endpoint);
        writeProtocolText("\r\nRegenBrake[Wh]:", endpoint);
        writeProtocolDouble(vesc_get_double(vescvalues.frame.watt_hours_charged_10000, 10000.0), endpoint);
        writeProtocolText("\r\nTachometer:", endpoint);
        writeProtocolLong(vesc_get_long(vescvalues.frame.tachometer), endpoint);
        writeProtocolText("\r\nTachometerAbs:", endpoint);
        writeProtocolLong(vesc_get_long(vescvalues.frame.tachometer_abs), endpoint);
        writeProtocolText("\r\nFault:", endpoint);
        switch (vescvalues.frame.fault_code)
        {
            case FAULT_CODE_NONE:               writeProtocolText(" None\r\n", endpoint); break;
            case FAULT_CODE_OVER_VOLTAGE:       writeProtocolText(" Overvoltage\r\n", endpoint); break;
            case FAULT_CODE_UNDER_VOLTAGE:      writeProtocolText(" Undervoltage\r\n", endpoint); break;
            case FAULT_CODE_DRV8302:            writeProtocolText(" DRV8302\r\n", endpoint); break;
            case FAULT_CODE_ABS_OVER_CURRENT:   writeProtocolText(" Overcurrent\r\n", endpoint); break;
            case FAULT_CODE_OVER_TEMP_FET:      writeProtocolText(" OvertempFET\r\n", endpoint); break;
            case FAULT_CODE_OVER_TEMP_MOTOR:    writeProtocolText(" OvertempMotor\r\n", endpoint); break;
        }
        writeProtocolOK(endpoint);
        break;
    }
    case PROTOCOL_SETUP:
    {
        PrintlnSerial_string("Configure the stick for CableCam movement...", endpoint);
        PrintlnSerial_string("...disabling RC Inputs", endpoint);
        controllerstatus.safemode = DISABLE_RC;
        sbusData_t rcmin;
        sbusData_t rcmax;
        sbusData_t rcavg;
        sbusData_t rcneutral;
        uint8_t channel = 255;
        PrintlnSerial_string("...Reading neutral position for 10 seconds...", endpoint);
        USBPeriodElapsed();
        getDutyValues(&rcmin, &rcmax, &rcneutral, 10000, endpoint);
        if (check_for_no_input_change(&rcmin, &rcmax, endpoint))
        {
            PrintlnSerial_string("...done.", endpoint);
            PrintlnSerial(endpoint);
            PrintlnSerial_string("Speed Channel:", endpoint);
            PrintlnSerial_string("You have 15 seconds to move the stick controlling the cablecam movements", endpoint);
            PrintlnSerial_string("to full forward and full reverse at least once...", endpoint);
            USBPeriodElapsed();
            getDutyValues(&rcmin, &rcmax, &rcavg, 15000, endpoint);
            if (check_for_multiple_channels_changed(&rcmin, &rcmax, &channel, endpoint))
            {
                if (check_for_neutral_is_in_the_middle(&rcavg, &rcneutral, channel, endpoint))
                {
                    PrintSerial_string("...Success. The identified channel is #", endpoint);
                    PrintlnSerial_int(channel+1, endpoint);
                    activesettings.rc_channel_speed = channel;
                    activesettings.stick_neutral_pos = rcneutral.servovalues[channel].duty;
                    activesettings.stick_neutral_range = 30;
                    activesettings.stick_value_range = (rcmax.servovalues[channel].duty - rcmin.servovalues[channel].duty)/2 - activesettings.stick_neutral_range;
                    PrintSerial_string("Neutral position is ", endpoint);
                    PrintSerial_int(activesettings.stick_neutral_pos, endpoint);
                    PrintSerial_string("+-", endpoint);
                    PrintlnSerial_int(activesettings.stick_neutral_range, endpoint);
                    PrintSerial_string("Stick value range is +-", endpoint);
                    PrintlnSerial_int(activesettings.stick_value_range, endpoint);


                    PrintlnSerial_string("Programming switch:", endpoint);
                    PrintlnSerial_string("You have 15 seconds to flip the programming switch", endpoint);
                    getDutyValues(&rcmin, &rcmax, &rcneutral, 10000, endpoint);
                    if (check_for_multiple_channels_changed(&rcmin, &rcmax, &channel, endpoint))
                    {
                        PrintSerial_string("...Success. The identified channel is #", endpoint);
                        PrintlnSerial_int(channel+1, endpoint);
                        activesettings.rc_channel_programming = channel;
                    }
                    else
                    {
                        PrintlnSerial_string("No channel identified - moving to next", endpoint);
                        activesettings.rc_channel_programming = 255;
                    }



                    PrintlnSerial_string("Endpoint tip switch:", endpoint);
                    PrintlnSerial_string("You have 15 seconds to click the endpoint tip switch", endpoint);
                    getDutyValues(&rcmin, &rcmax, &rcneutral, 10000, endpoint);
                    if (check_for_multiple_channels_changed(&rcmin, &rcmax, &channel, endpoint))
                    {
                        PrintSerial_string("...Success. The identified channel is #", endpoint);
                        PrintlnSerial_int(channel+1, endpoint);
                        activesettings.rc_channel_endpoint = channel;
                    }
                    else
                    {
                        PrintlnSerial_string("No channel identified - moving to next", endpoint);
                        activesettings.rc_channel_endpoint = 255;
                    }


                    PrintlnSerial_string("Max Accel dial:", endpoint);
                    PrintlnSerial_string("You have 15 seconds to move the max accel dial", endpoint);
                    getDutyValues(&rcmin, &rcmax, &rcneutral, 10000, endpoint);
                    if (check_for_multiple_channels_changed(&rcmin, &rcmax, &channel, endpoint))
                    {
                        PrintSerial_string("...Success. The identified channel is #", endpoint);
                        PrintlnSerial_int(channel+1, endpoint);
                        activesettings.rc_channel_max_accel= channel;
                    }
                    else
                    {
                        PrintlnSerial_string("No channel identified - moving to next", endpoint);
                        activesettings.rc_channel_max_accel= 255;
                    }


                    PrintlnSerial_string("Max speed dial:", endpoint);
                    PrintlnSerial_string("You have 15 seconds to move the max speed dial", endpoint);
                    getDutyValues(&rcmin, &rcmax, &rcneutral, 10000, endpoint);
                    if (check_for_multiple_channels_changed(&rcmin, &rcmax, &channel, endpoint))
                    {
                        PrintSerial_string("...Success. The identified channel is #", endpoint);
                        PrintlnSerial_int(channel+1, endpoint);
                        activesettings.rc_channel_max_speed = channel;
                    }
                    else
                    {
                        PrintlnSerial_string("No channel identified - moving to next", endpoint);
                        activesettings.rc_channel_max_speed = 255;
                    }


                    PrintlnSerial_string("Mode switch:", endpoint);
                    PrintlnSerial_string("You have 15 seconds to move the tristate mode switch", endpoint);
                    getDutyValues(&rcmin, &rcmax, &rcneutral, 10000, endpoint);
                    if (check_for_multiple_channels_changed(&rcmin, &rcmax, &channel, endpoint))
                    {
                        PrintSerial_string("...Success. The identified channel is #", endpoint);
                        PrintlnSerial_int(channel+1, endpoint);
                        activesettings.rc_channel_mode = channel;
                    }
                    else
                    {
                        PrintlnSerial_string("No channel identified - moving to next", endpoint);
                        activesettings.rc_channel_mode = 255;
                    }

                }
            }
        }
        PrintlnSerial_string("...enabling RC Inputs", endpoint);
        PrintlnSerial_string("Don't forget saving the values to make them permanent ($w command)", endpoint);
        USBPeriodElapsed();
        controllerstatus.safemode = INVALID_RC;
        break;
    }
    case PROTOCOL_READ_ERRORHANDLER:
    {
        uint8_t buffer[4096];
        int line;
        eeprom_read_sector(buffer, 4096, 5);
        PrintSerial_string("Line :", endpoint);
        memcpy(&line, &buffer[0], 2);
        PrintlnSerial_int(line, endpoint);
        PrintSerial_string("at: ", endpoint);
        buffer[256] = 0; // Just to make sure the string ends somewhere
        PrintlnSerial_string( (const char *) &buffer[2], endpoint);
        break;
    }
    case PROTOCOL_INPUT_GIMBAL:
    {
        int16_t p[8];
        argument_index = sscanf(&commandlinebuffer[2], "%hd %hd %hd %hd %hd %hd %hd %hd", &p[0], &p[1], &p[2], &p[3], &p[4], &p[5], &p[6], &p[7]);

        if (argument_index >= 1)
        {
            for (int i=0; i<8; i++)
            {
                if (i<argument_index && p[i] > 0 && p[i] <= 256)
                {
                    activesettings.rc_channel_sbus_out_mapping[i] = p[i]-1;
                }
                else
                {
                    activesettings.rc_channel_sbus_out_mapping[i] = 255;
                }
            }
        }
        writeProtocolHead(PROTOCOL_INPUT_GIMBAL, endpoint);
        for (int i=0; i<8; i++)
        {
            writeProtocolInt(activesettings.rc_channel_sbus_out_mapping[i]+1, endpoint);
        }
        writeProtocolOK(endpoint);
        printCurrentSBusOut(endpoint);
        break;
    }
    case PROTOCOL_INPUT_GIMBAL_DEFAULT:
    {
        int16_t p[8];
        argument_index = sscanf(&commandlinebuffer[2], "%hd %hd %hd %hd %hd %hd %hd %hd", &p[0], &p[1], &p[2], &p[3], &p[4], &p[5], &p[6], &p[7]);

        if (argument_index >= 1)
        {
            for (int i=0; i<8; i++)
            {
                if (i<argument_index && p[i] > 0 && p[i] <= 2000)
                {
                    activesettings.rc_channel_sbus_out_default[i] = p[i];
                }
                else
                {
                    activesettings.rc_channel_sbus_out_mapping[i] = 0;
                }
            }
        }
        writeProtocolHead(PROTOCOL_INPUT_GIMBAL_DEFAULT, endpoint);
        for (int i=0; i<8; i++)
        {
            writeProtocolInt(activesettings.rc_channel_sbus_out_default[i], endpoint);
        }
        writeProtocolOK(endpoint);
        break;
    }
    case PROTOCOL_INPUT_SINGLE:
        {
            char c;
            int16_t p;
            argument_index = sscanf(&commandlinebuffer[2], "%1s %hd", &c, &p);
            if (argument_index == 2)
            {
                if (p > 0 && p <= 256)
                {
                    switch (c)
                    {
                    case 'a':
                        activesettings.rc_channel_aux = p-1;
                        break;
                    case 'm':
                        activesettings.rc_channel_mode = p-1;
                        break;
                    case 'p':
                        activesettings.rc_channel_play = p-1;
                        break;
                    case 'e':
                        activesettings.rc_channel_endpoint = p-1;
                        break;
                    case 'A':
                        activesettings.rc_channel_max_accel = p-1;
                        break;
                    case 'V':
                        activesettings.rc_channel_max_speed = p-1;
                        break;
                    case 'P':
                        activesettings.rc_channel_programming = p-1;
                        break;
                    case 'v':
                        activesettings.rc_channel_speed = p-1;
                        break;
                    default:
                        writeProtocolError(ERROR_INVALID_VALUE, endpoint);
                        return;
                    }
                    writeProtocolHead(PROTOCOL_INPUT_SINGLE, endpoint);
                    writeProtocolOK(endpoint);
                }
                else
                {
                    writeProtocolError(ERROR_INVALID_VALUE, endpoint);
                    return;
                }
            }
            else if (argument_index == 1)
            {
                switch (c)
                {
                case 'a':
                    p = activesettings.rc_channel_aux;
                    break;
                case 'm':
                    p = activesettings.rc_channel_mode;
                    break;
                case 'p':
                    p = activesettings.rc_channel_play;
                    break;
                case 'e':
                    p = activesettings.rc_channel_endpoint;
                    break;
                case 'A':
                    p = activesettings.rc_channel_max_accel;
                    break;
                case 'V':
                    p = activesettings.rc_channel_max_speed;
                    break;
                case 'P':
                    p = activesettings.rc_channel_programming;
                    break;
                case 'v':
                    p = activesettings.rc_channel_speed;
                    break;
                default:
                    p = -1;
                }
                if (p == -1)
                {
                    writeProtocolError(ERROR_INVALID_VALUE, endpoint);
                }
                else
                {
                    writeProtocolHead(PROTOCOL_INPUT_SINGLE, endpoint);
                    writeProtocolChar(c, endpoint);
                    writeProtocolInt(p+1, endpoint);
                    writeProtocolOK(endpoint);
                }
            }
            else
            {
                writeProtocolError(ERROR_NUMBER_OF_ARGUMENTS, endpoint);
                return;
            }
            break;
        }
    case PROTOCOL_PLAY:
        {
            int16_t p;
            argument_index = sscanf(&commandlinebuffer[2], "%hd", &p);
            if (argument_index == 1)
            {
                controllerstatus.play_running = p;
                controllerstatus.play_time_lastsignal = HAL_GetTick();
            }
            writeProtocolHead(PROTOCOL_PLAY, endpoint);
            writeProtocolInt(controllerstatus.play_running, endpoint);
            writeProtocolOK(endpoint);
            break;
        }
    case PROTOCOL_POS_SOURCE:
        {
            int16_t p;
            argument_index = sscanf(&commandlinebuffer[2], "%hd", &p);
            if (argument_index == 1)
            {
                activesettings.pos_source = p;
            }
            writeProtocolHead(PROTOCOL_POS_SOURCE, endpoint);
            writeProtocolInt(activesettings.pos_source, endpoint);
            writeProtocolOK(endpoint);
            break;
        }
    case PROTOCOL_VERSION:
        {
            writeProtocolHead(PROTOCOL_VERSION, endpoint);
            writeProtocolText(activesettings.version, endpoint);
            writeProtocolOK(endpoint);
            break;
        }
    case PROTOCOL_BOOT:
        {
            /*** Jump to System Memory (ST Bootloader) ************************************/
            JumpToBootloader();
            // Does never return
        }
    case PROTOCOL_VESC_CONFIG:
        {
            controllerstatus.vesc_config = true;
            writeProtocolHead(PROTOCOL_VESC_CONFIG, endpoint);
            writeProtocolText("ON", endpoint);
            writeProtocolOK(endpoint);
            break;
        }
    default:  // we do not know how to handle the (valid) message, indicate error MSP $M!
        writeProtocolError(ERROR_UNKNOWN_COMMAND, endpoint);
        break;
    }
}

int16_t valid_channelassignment(int16_t c)
{
    if (c > 0 && c <= SBUS_MAX_CHANNEL)
    {
        return c;
    }
    else
    {
        return 256;
    }
}


uint8_t check_for_neutral_is_in_the_middle(sbusData_t * rcavg, sbusData_t * rcneutral, uint8_t channel, Endpoints endpoint)
{
    int16_t diff = rcavg->servovalues[channel].duty - rcneutral->servovalues[channel].duty;
    PrintSerial_string("Midpoint of min/max is ", endpoint);
    PrintSerial_int(rcavg->servovalues[channel].duty, endpoint);
    PrintSerial_string(" and stick neutral position is ", endpoint);
    PrintlnSerial_int(rcneutral->servovalues[channel].duty, endpoint);
    if (diff > -10 && diff < 10)
    {
        PrintlnSerial_string("This is close, hence okay.", endpoint);
        return 1;
    }
    else
    {
        PrintlnSerial_string("This is a difference >10 and hence not okay. Check the RC settings: no expo, configured min/max", endpoint);
        return 0;
    }
}

uint8_t check_for_no_input_change(sbusData_t * rcmin, sbusData_t * rcmax, Endpoints endpoint)
{
    for (int i=0; i<SBUS_MAX_CHANNEL; i++)
    {
        if (rcmax->servovalues[i].duty - rcmin->servovalues[i].duty > 200)
        {
            PrintSerial_string("Channel #", endpoint);
            PrintSerial_int(i+1, endpoint);
            PrintSerial_string(" has a min/max of ", endpoint);
            PrintSerial_int(rcmin->servovalues[i].duty, endpoint);
            PrintSerial_string("/", endpoint);
            PrintSerial_int(rcmax->servovalues[i].duty, endpoint);
            PrintlnSerial_string(" but should be constant. Please try again, keeping all in idle for the first 20 seconds.", endpoint);
            return 0;
        }
    }
    return 1;
}

uint8_t check_for_multiple_channels_changed(sbusData_t * rcmin, sbusData_t * rcmax, uint8_t *channel, Endpoints endpoint)
{
    uint8_t no_channels_with_changes = 0;
    for (int i=0; i<SBUS_MAX_CHANNEL; i++)
    {
        if (rcmax->servovalues[i].duty - rcmin->servovalues[i].duty > 300)
        {
            PrintSerial_string("Channel #", endpoint);
            PrintSerial_int(i+1, endpoint);
            PrintSerial_string(" has a min/max of ", endpoint);
            PrintSerial_int(rcmin->servovalues[i].duty, endpoint);
            PrintSerial_string("/", endpoint);
            PrintSerial_int(rcmax->servovalues[i].duty, endpoint);
            PrintlnSerial_string(", so has been moved.", endpoint);
            *channel = i;
            no_channels_with_changes++;
        }
    }
    if (no_channels_with_changes == 1)
    {
        return 1;
    }
    else
    {
        PrintlnSerial_string("More than a single channel got moved, don't know which one is the correct one.", endpoint);
        PrintlnSerial_string("Is there a mixer programmed in the RC maybe?", endpoint);
        return 0;
    }
}

void getDutyValues(sbusData_t * rcmin, sbusData_t * rcmax, sbusData_t * rcavg, uint32_t timeout, Endpoints endpoint)
{
    uint32_t lasttick = HAL_GetTick();
    uint32_t timeoutticks = HAL_GetTick() + timeout;
    PrintlnSerial(endpoint);
    for (int i=0; i<SBUS_MAX_CHANNEL; i++)
    {
        rcmax->servovalues[i].duty = 0;
        rcmin->servovalues[i].duty = 0;
    }
    while (HAL_GetTick() < timeoutticks)
    {
        if (HAL_GetTick() - lasttick > 500)
        {
            lasttick = HAL_GetTick();
            printChannelDutyValues(rcmin, rcmax, endpoint);
        }
        for (int i=0; i<SBUS_MAX_CHANNEL; i++)
        {
            if (rcmax->servovalues[i].duty < sbusdata.servovalues[i].duty)
            {
                rcmax->servovalues[i].duty = sbusdata.servovalues[i].duty;
            }
            if (rcmin->servovalues[i].duty > sbusdata.servovalues[i].duty || rcmin->servovalues[i].duty == 0)
            {
                rcmin->servovalues[i].duty = sbusdata.servovalues[i].duty;
            }
        }
    }
    for (int i=0; i<SBUS_MAX_CHANNEL; i++)
    {
        rcavg->servovalues[i].duty = (rcmax->servovalues[i].duty + rcmin->servovalues[i].duty)/2;
    }
    PrintlnSerial(endpoint);
}

void printChannelDutyValues(sbusData_t * rcmin, sbusData_t * rcmax, Endpoints endpoint)
{
    PrintSerial_char('\r', endpoint);
    for (int i=0; i<SBUS_MAX_CHANNEL; i++)
    {
        if (i != 0)
        {
            PrintSerial_string(", ", endpoint);
        }

        // Highlight all changed channels for better readability
        if (rcmax->servovalues[i].duty - rcmin->servovalues[i].duty > 200)
        {
            PrintSerial_string("***", endpoint);
        }
        PrintSerial_char('#', endpoint);
        PrintSerial_int(i+1, endpoint);
        PrintSerial_char(':', endpoint);
        PrintSerial_int(sbusdata.servovalues[i].duty, endpoint);

        if (rcmax->servovalues[i].duty - rcmin->servovalues[i].duty > 200)
        {
            PrintSerial_string("***", endpoint);
        }
    }
    USBPeriodElapsed();
}

void printHelp(Endpoints endpoint)
{
    PrintlnSerial(endpoint);
    PrintlnSerial_string("Possible commands are", endpoint);
    PrintlnSerial(endpoint);

    USBPeriodElapsed();

    PrintlnSerial_string("$1                                      start a guided setup to set the most important input channels", endpoint);
    PrintlnSerial_string("$a [<float> <float>]                    set or print maximum allowed acceleration in normal and programming mode [%/sec]", endpoint);
    PrintlnSerial_string("$A [<int>]                              set or print AUX output neutral pos and +- neutral range and the +-max range", endpoint);
    PrintlnSerial_string("$B                                      Configure the HC-05 Bluetooth module on FlexiPort (RX3/TX3)", endpoint);
    // PrintlnSerial_string("$c [<int> <int>]                        VESC handbrake current and min tacho speed to engage handbrake", endpoint);
    PrintlnSerial_string("$D [<int1> ... <int8>]                  set or print gimbal default values for each channel", endpoint);
    PrintlnSerial_string("$e [<long>]                             set or print maximum eRPMs as set in the VESC speed controller. 100% stick = this eRPM", endpoint);
    PrintlnSerial_string("$E                                      print the status of the VESC ESC", endpoint);
    PrintlnSerial_string("$g [<float>]                            set or print the max positional error -> exceeding it causes an emergency stop", endpoint);
    PrintlnSerial_string("$G [<int1> ... <int8>]                  set or print gimbal channel mapping", endpoint);
    PrintlnSerial_string("$i [<int> ..... ]                       set or print input channels for Speed, Programmingswitch, Endpointswitch,...", endpoint);
    PrintlnSerial_string("                                                  ...Max Accel, Max Speed, Mode, Aux, Playswitch", endpoint);
    PrintlnSerial_string("$I [<int>]                              set or print input source 0..SumPPM", endpoint);
    PrintlnSerial_string("                                                                  1..SBus", endpoint);
    USBPeriodElapsed();
    PrintlnSerial_string("$j <char> [<int>]                       set or print the channel assignment for a given function. Chars can be...", endpoint);
    PrintlnSerial_string("                                        v...speed channel", endpoint);
    PrintlnSerial_string("                                        m...Modeswitch channel", endpoint);
    PrintlnSerial_string("                                        P...Endpoint programming switch channel", endpoint);
    PrintlnSerial_string("                                        e...Endpoint tip switch channel", endpoint);
    PrintlnSerial_string("                                        A...Max accel dial channel", endpoint);
    PrintlnSerial_string("                                        V...Max speed dial channel", endpoint);
    PrintlnSerial_string("                                        a...Aux input channel", endpoint);
    PrintlnSerial_string("                                        p...Playswitch channel", endpoint);
    PrintlnSerial_string("$m [<int>]                              set or print the mode 1..passthrough", endpoint);
    PrintlnSerial_string("                                                              2..passthrough with speed limits", endpoint);
    PrintlnSerial_string("                                                              3..passthrough with speed limits & end points", endpoint);

    USBPeriodElapsed();

    PrintlnSerial_string("$n [<int> <int> <int>]                  set or print receiver neutral pos and +-neutral range and +-max range", endpoint);
    PrintlnSerial_string("$N [<int> <int> <int>]                  set or print ESC output neutral pos and +-neutral range and the +-max range", endpoint);
    PrintlnSerial_string("$p                                      print positions", endpoint);
    PrintlnSerial_string("$P <int>                                turn the Play command (automatic movement) on (1) or off (0) for the next 2 seconds. Send frequently.", endpoint);
    PrintlnSerial_string("$r [<float>]                            set or print rotation direction of the ESC output, either +1 or -1", endpoint);
    PrintlnSerial_string("$S                                      print all settings", endpoint);
    PrintlnSerial_string("$v [<float> <float>]                    set or print the maximum output % when stick is at 100% (=1.0) in normal and programming mode", endpoint);
    PrintlnSerial_string("$V                                      print firmware version", endpoint);
    PrintlnSerial_string("$w                                      write settings to eeprom", endpoint);
    PrintlnSerial_string("$x [<float>]                            expo factor 1.0 means linear, everything between 1 and 0 is a exponential input", endpoint);
    PrintlnSerial_string("$Z [<int>]                              set or print the encoder source 0..external encoder, 1..VESC Tacho", endpoint);

}


void printActiveSettings(Endpoints endpoint)
{
    PrintlnSerial(endpoint);

    PrintlnSerial_string(controllerstatus.boottext_eeprom, endpoint);
    PrintlnSerial(endpoint);

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
    PrintlnSerial_string(getSafeModeLabel(), endpoint);

    PrintlnSerial_string("Currently the endpoint limits are configured as ", endpoint);

     // ------- Endpoint
    if (semipermanentsettings.pos_start > semipermanentsettings.pos_end)
    {
        /* start is always smaller than end anyway but below prints rely on that, hence doublecheck. */
        float tmp = semipermanentsettings.pos_start;
        semipermanentsettings.pos_start = semipermanentsettings.pos_end;
        semipermanentsettings.pos_end = tmp;
    }

    PrintSerial_string(" ", endpoint);
    if (controllerstatus.pos < semipermanentsettings.pos_start)
    {
        PrintSerial_long((int32_t) controllerstatus.pos, endpoint);
        PrintSerial_string("--- ", endpoint);
    }
    PrintSerial_long((int32_t) semipermanentsettings.pos_start, endpoint);
    PrintSerial_string(" <--------- ", endpoint);
    if (semipermanentsettings.pos_start <= controllerstatus.pos && controllerstatus.pos <= semipermanentsettings.pos_end)
    {
        PrintSerial_long((int32_t) controllerstatus.pos, endpoint);
    }
    PrintSerial_string(" ---------> ", endpoint);
    PrintSerial_long((int32_t) semipermanentsettings.pos_end, endpoint);
    if (controllerstatus.pos > semipermanentsettings.pos_end)
    {
        PrintSerial_string("--- ", endpoint);
        PrintSerial_long((int32_t) controllerstatus.pos, endpoint);
    }
    PrintlnSerial(endpoint);

    USBPeriodElapsed();


    // ------- Ramp
    PrintlnSerial_string("Ramp filter:", endpoint);

    PrintSerial_string("The stick changes from 0% to 100% in", endpoint);
    PrintSerial_float(CONTROLLERLOOPTIME_FLOAT/controllerstatus.stick_max_accel, endpoint);
    PrintSerial_string(" seconds and output is limited to +-", endpoint);
    PrintSerial_int(((int16_t) (controllerstatus.stick_max_speed*100.0f)), endpoint);
    PrintlnSerial_string("% max speed", endpoint);
    PrintlnSerial(endpoint);

    // ------- ESC direction
    PrintlnSerial_string("The sensor direction and the motor direction might not be the same. If the CableCam", endpoint);
    PrintlnSerial_string("overshot the end point, pulling the stick in reverse direction to get the CableCam back between", endpoint);
    PrintlnSerial_string("the end points should be allowed, moving it further out should not. But which direction is it?", endpoint);
    PrintSerial_string("  Motor/ESC direction:", endpoint);
    PrintSerial_float(activesettings.esc_direction, endpoint);
    if (activesettings.esc_direction == 1.0f)
    {
        PrintlnSerial_string(" (positive stick = hall sensor counts up)", endpoint);
    }
    else if (activesettings.esc_direction == -1.0f)
    {
        PrintlnSerial_string(" (positive stick = hall sensor counts down)", endpoint);
    } else
    {
        PrintlnSerial_string(" (not yet decided, set the endpoints via the RC to get help)", endpoint);
    }
    PrintlnSerial(endpoint);

    USBPeriodElapsed();

    // ------- States
    if (controllerstatus.play_running == 1)
    {
        PrintSerial_string("Play program requested ", endpoint);
        if (controllerstatus.safemode != OPERATIONAL)
        {
            PrintlnSerial_string("but inactive as not in OPERATIONAL mode", endpoint);
        }
        else if (activesettings.mode != MODE_LIMITER_ENDPOINTS)
        {
            PrintlnSerial_string("but inactive as not in ENDPOINTS mode", endpoint);
        }
        else if (getDuty(activesettings.rc_channel_speed) != 0.0f)
        {
            PrintlnSerial_string("but inactive as speed is overridden by RC", endpoint);
        }
        else if (semipermanentsettings.pos_start == -POS_END_NOT_SET || semipermanentsettings.pos_end == POS_END_NOT_SET)
        {
            PrintlnSerial_string("but inactive as endpoints have never been set", endpoint);
        }
        else
        {
            PrintlnSerial_string("and active", endpoint);
        }
    }

    printCurrentRCInput(endpoint);
}


const char * getSafeModeLabel()
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

const char * getCurrentModeLabel(uint8_t mode)
{
    switch (mode)
    {
        case MODE_LIMITER: return "passthrough with limiter";
        case MODE_LIMITER_ENDPOINTS: return "passthrough with limiter & end points";
        case MODE_PASSTHROUGH: return "passthrough";
        default : return "????current mode???";
    }
}

void printCurrentRCInput(Endpoints endpoint)
{
    PrintSerial_string("Last RC signal received", endpoint);
    PrintSerial_long(HAL_GetTick() - sbusdata.sbusLastValidFrame, endpoint);
    PrintlnSerial_string("ms ago", endpoint);

    PrintlnSerial_string("Value ranges configured are", endpoint);
    PrintSerial_int(activesettings.stick_neutral_pos - activesettings.stick_neutral_range - activesettings.stick_value_range, endpoint);
    PrintSerial_string(" .... ",endpoint);
    PrintSerial_int(activesettings.stick_neutral_pos - activesettings.stick_neutral_range, endpoint);
    PrintSerial_string(" neutral ",endpoint);
    PrintSerial_int(activesettings.stick_neutral_pos, endpoint);
    PrintSerial_string(" neutral ",endpoint);
    PrintSerial_int(activesettings.stick_neutral_pos + activesettings.stick_neutral_range, endpoint);
    PrintSerial_string(" .... ",endpoint);
    PrintlnSerial_int(activesettings.stick_neutral_pos + activesettings.stick_neutral_range + activesettings.stick_value_range, endpoint);
    PrintlnSerial(endpoint);

    int i = 0;
    for (i=0; i<SBUS_MAX_CHANNEL; i++)
    {
        PrintSerial_string("channel", endpoint);
        PrintSerial_int(i+1, endpoint);
        PrintSerial_string(" =", endpoint);
        PrintSerial_int(sbusdata.servovalues[i].duty, endpoint);
        if (i == activesettings.rc_channel_speed)
        {
            PrintSerial_string(" (used as speed signal input)", endpoint);
        }
        else if (i == activesettings.rc_channel_programming)
        {
            PrintSerial_string(" (used as programming switch)", endpoint);
        }
        else if (i == activesettings.rc_channel_endpoint)
        {
            PrintSerial_string(" (used as endpoint switch)", endpoint);
        }
        else if (i == activesettings.rc_channel_max_accel)
        {
            PrintSerial_string(" (used as max acceleration value selector)", endpoint);
        }
        else if (i == activesettings.rc_channel_max_speed)
        {
            PrintSerial_string(" (used as max speed selector)", endpoint);
         }
        else if (i == activesettings.rc_channel_mode)
        {
            PrintSerial_string(" (used as mode selector)", endpoint);
        }
        else if (i == activesettings.rc_channel_aux)
        {
            PrintSerial_string(" (used as aux input)", endpoint);
        }
        else if (i == activesettings.rc_channel_play)
        {
            PrintSerial_string(" (used as play on/off switch)", endpoint);
        }
        PrintlnSerial(endpoint);
    }
    PrintlnSerial(endpoint);


    PrintSerial_string("current ESC out signal Servo 1 = ", endpoint);
    PrintlnSerial_int(TIM3->CCR3, endpoint);
    PrintSerial_string("current AUX out signal Servo 2 = ", endpoint);
    PrintlnSerial_int(TIM3->CCR4, endpoint);

    PrintlnSerial_string("Gimbal Out:", endpoint);
    for (i=0; i<8; i++)
    {
        PrintSerial_string("channel", endpoint);
        PrintSerial_int(i+1, endpoint);
        PrintSerial_string(" =", endpoint);
        switch (i)
        {
            case 0: PrintlnSerial_int(sBusFrameGimbal.frame.chan0, endpoint); break;
            case 1: PrintlnSerial_int(sBusFrameGimbal.frame.chan1, endpoint); break;
            case 2: PrintlnSerial_int(sBusFrameGimbal.frame.chan2, endpoint); break;
            case 3: PrintlnSerial_int(sBusFrameGimbal.frame.chan3, endpoint); break;
            case 4: PrintlnSerial_int(sBusFrameGimbal.frame.chan4, endpoint); break;
            case 5: PrintlnSerial_int(sBusFrameGimbal.frame.chan5, endpoint); break;
            case 6: PrintlnSerial_int(sBusFrameGimbal.frame.chan6, endpoint); break;
            case 7: PrintlnSerial_int(sBusFrameGimbal.frame.chan7, endpoint); break;
        }
    }

}

void printCurrentSBusOut(Endpoints endpoint)
{
    PrintSerial_string("Last RC signal received", endpoint);
    PrintSerial_long(HAL_GetTick() - sbusdata.sbusLastValidFrame, endpoint);
    PrintlnSerial_string("ms ago", endpoint);

    PrintlnSerial_string("Gimbal Out:", endpoint);
    for (int i=0; i<8; i++)
    {
        int16_t found_channel_input = -1;
        for (int j=0; j<SBUS_MAX_CHANNEL; j++)
        {
            if (activesettings.rc_channel_sbus_out_mapping[j] == i)
            {
                found_channel_input = j;
                break;
            }
        }
        if (found_channel_input == -1)
        {
            PrintSerial_string("                    ", endpoint);
        }
        else
        {
            PrintSerial_string("input channel", endpoint);
            PrintSerial_int(found_channel_input+1, endpoint);
            PrintSerial_string(" ==> ", endpoint);
        }
        PrintSerial_string("\t SBus out channel", endpoint);
        PrintSerial_int(i+1, endpoint);
        PrintSerial_string(", current output =", endpoint);
        switch (i)
        {
            case 0: PrintlnSerial_int(sBusFrameGimbal.frame.chan0, endpoint); break;
            case 1: PrintlnSerial_int(sBusFrameGimbal.frame.chan1, endpoint); break;
            case 2: PrintlnSerial_int(sBusFrameGimbal.frame.chan2, endpoint); break;
            case 3: PrintlnSerial_int(sBusFrameGimbal.frame.chan3, endpoint); break;
            case 4: PrintlnSerial_int(sBusFrameGimbal.frame.chan4, endpoint); break;
            case 5: PrintlnSerial_int(sBusFrameGimbal.frame.chan5, endpoint); break;
            case 6: PrintlnSerial_int(sBusFrameGimbal.frame.chan6, endpoint); break;
            case 7: PrintlnSerial_int(sBusFrameGimbal.frame.chan7, endpoint); break;
        }
    }

}

