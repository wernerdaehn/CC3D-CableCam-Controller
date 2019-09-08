#include "odrive.h"
#include "stdio.h"
#include "string.h"
#include "serial_print.h"
#include "protocol.h"
#include "clock_50hz.h"
#include "math.h"
#include "uart_callback.h"

void ODRIVE_set_rpm(int32_t odrive_erpm);
void ODRIVE_set_position(float target_pos);
void ODRIVE_Request_Value(ODRIVE_REQUEST_t requesttype);
int32_t getODriveIntValue(void);
float getODriveFloatValue(void);

uint8_t odriveTxBuffer[UART_CYCLIC_RXBUFFER_SIZE];

uint32_t last_odrive_read_request;
ODRIVE_REQUEST_t odriverequested = ODRIVE_REQUESTED_NONE;
ODRIVE_REQUEST_t odrivecurrentvalue = ODRIVE_REQUESTED_NONE;

uint16_t odrive_received_pos = 0;
uint16_t odrive_packet_start_pos = 0;
uint32_t odrive_bytes_written = 0;
uint32_t odrive_bytes_sent = 0;

uint8_t odrive_cyclic_rxbuffer[UART_CYCLIC_RXBUFFER_SIZE];
uint8_t odrive_rxbuffer[UART_CYCLIC_RXBUFFER_SIZE];
uint32_t odrive_last_data_timestamp = 0;

ODRIVE_STATUS_t odrivestatus;
uint16_t counter = 0;

extern UART_HandleTypeDef huart2;

void ODRIVE_init()
{
    controllerstatus.odrivestate = ODRIVE_UNKNOWN;
    if(HAL_UART_Receive_DMA(&huart2, odrive_cyclic_rxbuffer, UART_CYCLIC_RXBUFFER_SIZE) != HAL_OK)
    {
        Error_Handler();
    }
}

void ODRIVE_Output(float esc_output)
{

    switch (controllerstatus.odrivestate)
    {
    case ODRIVE_MOTOR_TO_BE_CALIBRATED:
    case ODRIVE_UNKNOWN:
        if (odrivecurrentvalue != ODRIVE_REQUESTED_MOTOR_CALIBRATED || odrivestatus.motor_calibrated  != 1)
        {
            if (odrivecurrentvalue == ODRIVE_REQUESTED_MOTOR_CALIBRATED)
            {
                PrintlnSerial_string("oDrive motor is NOT calibrated, see oDrive manual on how to do that and store the setting permanently", EndPoint_All);
            }
            ODRIVE_Request_Value(ODRIVE_REQUESTED_MOTOR_CALIBRATED);
        }
        else
        {
            if (odrivestatus.motor_calibrated  == 1)
            {
                controllerstatus.odrivestate = ODRIVE_MOTOR_CALIBRATED;
                PrintlnSerial_string("oDrive motor is calibrated", EndPoint_All);
            }
        }
        break;
    case ODRIVE_MOTOR_CALIBRATED:
        if (odrivecurrentvalue != ODRIVE_REQUESTED_ENCODER_CALIBRATED || odrivestatus.encoder_calibrated != 1)
        {
            ODRIVE_Request_Value(ODRIVE_REQUESTED_ENCODER_CALIBRATED);
        }
        else
        {
            if (odrivestatus.encoder_calibrated == 1)
            {
                controllerstatus.odrivestate = ODRIVE_ENCODER_CALIBRATED;
                PrintlnSerial_string("oDrive encoder is calibrated", EndPoint_All);
            }
        }
        break;
    case ODRIVE_ENCODER_CALIBRATED:
        if (odrivecurrentvalue != ODRIVE_REQUESTED_CLOSED_LOOP || odrivestatus.closed_loop != 8)
        {
            ODRIVE_Request_Value(ODRIVE_REQUESTED_CLOSED_LOOP);
        }
        else
        {
            if (odrivestatus.closed_loop == 8)
            {
                controllerstatus.odrivestate = ODRIVE_SPEED_MODE;
                PrintlnSerial_string("oDrive is in speed mode", EndPoint_All);
            }
        }
        break;
    case ODRIVE_SPEED_MODE:
        if (odrivecurrentvalue != ODRIVE_REQUESTED_SPEED_CONTROL_MODE)
        {
            ODRIVE_Request_Value(ODRIVE_REQUESTED_SPEED_CONTROL_MODE);
        }
        else
        {
            if (odrivestatus.speed_control_loop ==  2) // 2...velocity control
            {
                controllerstatus.odrivestate = ODRIVE_CLOSED_LOOP;
                PrintlnSerial_string("oDrive is in closed loop operation (velocity control)", EndPoint_All);
            }
            else if (odrivestatus.speed_control_loop ==  3) // 3...position control
            {
                controllerstatus.odrivestate = ODRIVE_CLOSED_LOOP;
                activesettings.pos_source = 1;
                PrintlnSerial_string("oDrive is in closed loop operation (position control)", EndPoint_All);
            }
        }
        break;
    case ODRIVE_CLOSED_LOOP:
        if (odrivecurrentvalue != ODRIVE_REQUESTED_SPEED_LIMIT)
        {
            ODRIVE_Request_Value(ODRIVE_REQUESTED_SPEED_LIMIT);
        }
        else
        {
            float p = getODriveFloatValue();
            PrintSerial_string("oDrive has a max speed of ", EndPoint_All);
            PrintSerial_float(p, EndPoint_All);
            PrintSerial_string(" and this controller of ", EndPoint_All);
            PrintlnSerial_float(activesettings.esc_max_speed, EndPoint_All);
            if (p < activesettings.esc_max_speed) // if the programmed speed is higher than the hard limit set in the odrive, use this
            {
                activesettings.esc_max_speed = p;
            }
            controllerstatus.esc_max_speed = activesettings.esc_max_speed; // controllerstatus contains the read value
            controllerstatus.odrivestate = ODRIVE_OPERATIONAL;
        }
        break;
    case ODRIVE_OPERATIONAL:
        if (is5s())
        {
            switch (counter)
            {
                case 0: ODRIVE_Request_Value(ODRIVE_REQUESTED_ERRORCODE); break;
                case 1: ODRIVE_Request_Value(ODRIVE_REQUESTED_ERRORCODEMOTOR); break;
                case 2: ODRIVE_Request_Value(ODRIVE_REQUESTED_ERRORCODECONTROLLER); break;
                case 3: ODRIVE_Request_Value(ODRIVE_REQUESTED_ERRORCODEENCODER); break;
            }
            counter++;
            if (counter > 3)
            {
                counter = 0;
            }
        }
        else if (is5Hz())
        {
            ODRIVE_Request_Value(ODRIVE_REQUESTED_POSITION);
        }

        if (activesettings.pos_source == 1)
        {
            ODRIVE_set_position(controllerstatus.target_pos);
        }
        else if (isnan(esc_output))
        {
            ODRIVE_set_rpm(0);
        }
        else
        {
            int32_t odrive_speed;
            if (activesettings.mode == MODE_PASSTHROUGH)
            {
                odrive_speed = (int32_t) ((esc_output * ((float) activesettings.esc_max_speed)));
            }
            else
            {
                // controllerstatus.stick_max_speed reduces the vesc erpm by this factor at stick=100%
                odrive_speed = (int32_t) ((esc_output * controllerstatus.stick_max_speed * ((float) activesettings.esc_max_speed)));
            }
            ODRIVE_set_rpm(odrive_speed);
        }

    }
}

void ODRIVE_set_rpm(int32_t odrive_speed)
{
    char buf[40];
    if (odrive_speed > activesettings.esc_max_speed)
    {
        odrive_speed = activesettings.esc_max_speed;
    }
    else if (odrive_speed < -activesettings.esc_max_speed)
    {
        odrive_speed = -activesettings.esc_max_speed;
    }
    uint32_t len = snprintf(buf, sizeof(buf), "v 0 %ld 0\n", odrive_speed);
    if (len > 0)
    {
        ODRIVESendPacket((uint8_t *) buf, len);
    }
}

void ODRIVE_set_position(float target_pos)
{
    char buf[40];
    uint32_t len = snprintf(buf, sizeof(buf), "p 0 %lf\n", (double) target_pos);
    if (len > 0)
    {
        ODRIVESendPacket((uint8_t *) buf, len);
    }
}


ODRIVE_REQUEST_t getCurrentODriveValueType()
{
    return odrivecurrentvalue;
}

char * getCurrentODriveResponse()
{
    return (char *) odrive_rxbuffer;
}

void ODRIVE_Request_Value(ODRIVE_REQUEST_t requesttype)
{
    if (HAL_GetTick() > last_odrive_read_request + 1000L)
    {
        odriverequested = ODRIVE_REQUESTED_NONE;
        odrivecurrentvalue = ODRIVE_REQUESTED_NONE;
    }

    if (odriverequested == ODRIVE_REQUESTED_NONE)
    {
        odriverequested = requesttype;
        last_odrive_read_request = HAL_GetTick();
        odrivecurrentvalue = ODRIVE_REQUESTED_NONE;
        switch (requesttype)
        {
        case ODRIVE_REQUESTED_MOTOR_CALIBRATED:
            ODRIVESendLine("r axis0.motor.is_calibrated\n");
            break;
        case ODRIVE_REQUESTED_ENCODER_CALIBRATED:
            ODRIVESendLine("r axis0.encoder.is_ready\n");
            break;
        case ODRIVE_REQUESTED_CLOSED_LOOP:
            ODRIVESendLine("r axis0.current_state\n");
            break;
        case ODRIVE_REQUESTED_SPEED_CONTROL_MODE:
            ODRIVESendLine("r axis0.controller.config.control_mode\n");
            break;
        case ODRIVE_REQUESTED_SPEED_LIMIT:
            ODRIVESendLine("r axis0.controller.config.vel_limit\n");
            break;
        case ODRIVE_REQUESTED_POSITION:
            ODRIVESendLine("r axis0.encoder.pos_estimate\n");
            break;
        case ODRIVE_REQUESTED_ERRORCODE:
            ODRIVESendLine("r axis0.error\n");
            break;
        case ODRIVE_REQUESTED_ERRORCODEMOTOR:
            ODRIVESendLine("r axis0.motor.error\n");
            break;
        case ODRIVE_REQUESTED_ERRORCODECONTROLLER:
            ODRIVESendLine("r axis0.controller.error\n");
            break;
        case ODRIVE_REQUESTED_ERRORCODEENCODER:
            ODRIVESendLine("r axis0.encoder.error\n");
            break;
        default:
            break;
        }
    }
}


uint16_t ODRIVEReceive()
{
    uint16_t currentreceivepos = huart2.RxXferSize - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    int16_t lastcharacter_at = currentreceivepos-1;
    uint32_t now = HAL_GetTick();

    if (lastcharacter_at < 0)
    {
        lastcharacter_at = 0;
    }


    if (currentreceivepos != odrive_received_pos)
    {
        if (now > odrive_last_data_timestamp + 1000L)
        {
            odrive_packet_start_pos = lastcharacter_at;
        }
        odrive_last_data_timestamp = now;
        odrive_received_pos = currentreceivepos;
        if (odrive_cyclic_rxbuffer[lastcharacter_at] == '\n')
        {
            controllerstatus.duart_pause[2]++;
            uint16_t packetlength = mod16(currentreceivepos - odrive_packet_start_pos, huart2.RxXferSize);
            controllerstatus.duart_last_packet_length[2] = packetlength;


            if (odrive_packet_start_pos < currentreceivepos)
            {
                memcpy(&odrive_rxbuffer[0], &odrive_cyclic_rxbuffer[odrive_packet_start_pos], packetlength);
            }
            else
            {
                memcpy(&odrive_rxbuffer[0], &odrive_cyclic_rxbuffer[odrive_packet_start_pos], huart2.RxXferSize-odrive_packet_start_pos);
                memcpy(&odrive_rxbuffer[huart2.RxXferSize-odrive_packet_start_pos], &odrive_cyclic_rxbuffer[0], currentreceivepos);
            }
            odrive_rxbuffer[packetlength] = 0x00;
            odrivecurrentvalue = odriverequested;
            odrive_packet_start_pos = currentreceivepos;
            switch (odriverequested)
            {
            case ODRIVE_REQUESTED_NONE:
                break;
            case ODRIVE_REQUESTED_MOTOR_CALIBRATED:
                odrivestatus.motor_calibrated = getODriveIntValue();
                break;
            case ODRIVE_REQUESTED_MOTOR_CALIBRATION:
                odrivestatus.motor_calibration = getODriveIntValue();
                break;
            case ODRIVE_REQUESTED_ENCODER_CALIBRATED:
                odrivestatus.encoder_calibrated = getODriveIntValue();
                break;
            case ODRIVE_REQUESTED_CLOSED_LOOP:
                odrivestatus.closed_loop = getODriveIntValue();
                break;
            case ODRIVE_REQUESTED_SPEED_CONTROL_MODE:
                odrivestatus.speed_control_loop = getODriveIntValue();
                break;
            case ODRIVE_REQUESTED_SPEED_LIMIT:
                odrivestatus.speed_limit = getODriveIntValue();
                break;
            case ODRIVE_REQUESTED_POSITION:
                odrivestatus.position = getODriveIntValue();
                controllerstatus.odrive_pos = getODriveFloatValue();
                break;
            case ODRIVE_REQUESTED_ERRORCODE:
                odrivestatus.global_error = getODriveIntValue();
                break;
            case ODRIVE_REQUESTED_ERRORCODEMOTOR:
                odrivestatus.motor_error = getODriveIntValue();
                break;
            case ODRIVE_REQUESTED_ERRORCODECONTROLLER:
                odrivestatus.controller_error = getODriveIntValue();
                break;
            case ODRIVE_REQUESTED_ERRORCODEENCODER:
                odrivestatus.encoder_error = getODriveIntValue();
                break;
            }
            odriverequested = ODRIVE_REQUESTED_NONE;
        }
    }
    return 0;
}

int32_t getODriveIntValue()
{
    int32_t p;
    uint16_t argument_index = sscanf((char *) odrive_rxbuffer, "%ld", &p);
    if (argument_index == 1)
    {
        return p;
    }
    else
    {
        return 0;
    }
}

float getODriveFloatValue()
{
    float p;
    uint16_t argument_index = sscanf((char *) odrive_rxbuffer, "%f", &p);
    if (argument_index == 1)
    {
        return p;
    }
    else
    {
        return 0;
    }
}



void ODRIVESendPacket(uint8_t *ptr, uint32_t len)
{
    if (odriverequested == ODRIVE_REQUESTED_NONE)
    {
        memcpy(odriveTxBuffer, ptr, len);
        HAL_UART_Transmit_DMA(&huart2, odriveTxBuffer, len);
    }
}

void ODRIVESendLine(const char *ptr)
{
    odrive_last_data_timestamp = HAL_GetTick();
    // PrintlnSerial_string(ptr, EndPoint_USB);
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *) ptr, strlen(ptr));
}

void ODRIVE_Print_Errors(Endpoints endpoint)
{
    if (odrivestatus.global_error & AXIS_ERROR_INVALID_STATE)
    {
        PrintlnSerial_string("axis.error = ERROR_INVALID_STATE", endpoint);
    }
    if (odrivestatus.global_error & AXIS_ERROR_DC_BUS_UNDER_VOLTAGE)
    {
        PrintlnSerial_string("axis.error = ERROR_DC_BUS_UNDER_VOLTAGE", endpoint);
    }
    if (odrivestatus.global_error & AXIS_ERROR_DC_BUS_OVER_VOLTAGE)
    {
        PrintlnSerial_string("axis.error = ERROR_DC_BUS_OVER_VOLTAGE", endpoint);
    }
    if (odrivestatus.global_error & AXIS_ERROR_CURRENT_MEASUREMENT_TIMEOUT)
    {
        PrintlnSerial_string("axis.error = ERROR_CURRENT_MEASUREMENT_TIMEOUT", endpoint);
    }
    if (odrivestatus.global_error & AXIS_ERROR_BRAKE_RESISTOR_DISARMED)
    {
        PrintlnSerial_string("axis.error = ERROR_BRAKE_RESISTOR_DISARMED", endpoint);
    }
    if (odrivestatus.global_error & AXIS_ERROR_MOTOR_DISARMED)
    {
        PrintlnSerial_string("axis.error = ERROR_MOTOR_DISARMED", endpoint);
    }
    if (odrivestatus.global_error & AXIS_ERROR_MOTOR_FAILED)
    {
        PrintlnSerial_string("axis.error = ERROR_MOTOR_FAILED", endpoint);
    }
    if (odrivestatus.global_error & AXIS_ERROR_SENSORLESS_ESTIMATOR_FAILED)
    {
        PrintlnSerial_string("axis.error = ERROR_SENSORLESS_ESTIMATOR_FAILED", endpoint);
    }
    if (odrivestatus.global_error & AXIS_ERROR_ENCODER_FAILED)
    {
        PrintlnSerial_string("axis.error = ERROR_ENCODER_FAILED", endpoint);
    }
    if (odrivestatus.global_error & AXIS_ERROR_CONTROLLER_FAILED)
    {
        PrintlnSerial_string("axis.error = ERROR_CONTROLLER_FAILED", endpoint);
    }
    if (odrivestatus.global_error & AXIS_ERROR_POS_CTRL_DURING_SENSORLESS)
    {
        PrintlnSerial_string("axis.error = ERROR_POS_CTRL_DURING_SENSORLESS", endpoint);
    }
    if (odrivestatus.global_error & AXIS_ERROR_WATCHDOG_TIMER_EXPIRED)
    {
        PrintlnSerial_string("axis.error = ERROR_WATCHDOG_TIMER_EXPIRED", endpoint);
    }


    if (odrivestatus.motor_error & MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE)
    {
        PrintlnSerial_string("axis.motor.error = ERROR_PHASE_RESISTANCE_OUT_OF_RANGE", endpoint);
    }
    if (odrivestatus.motor_error & MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE)
    {
        PrintlnSerial_string("axis.motor.error = ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE", endpoint);
    }
    if (odrivestatus.motor_error & MOTOR_ERROR_ADC_FAILED)
    {
        PrintlnSerial_string("axis.motor.error = ERROR_ADC_FAILED", endpoint);
    }
    if (odrivestatus.motor_error & MOTOR_ERROR_DRV_FAULT)
    {
        PrintlnSerial_string("axis.motor.error = ERROR_DRV_FAULT", endpoint);
    }
    if (odrivestatus.motor_error & MOTOR_ERROR_CONTROL_DEADLINE_MISSED)
    {
        PrintlnSerial_string("axis.motor.error = ERROR_CONTROL_DEADLINE_MISSED", endpoint);
    }
    if (odrivestatus.motor_error & MOTOR_ERROR_NOT_IMPLEMENTED_MOTOR_TYPE)
    {
        PrintlnSerial_string("axis.motor.error = ERROR_NOT_IMPLEMENTED_MOTOR_TYPE", endpoint);
    }
    if (odrivestatus.motor_error & MOTOR_ERROR_BRAKE_CURRENT_OUT_OF_RANGE)
    {
        PrintlnSerial_string("axis.motor.error = ERROR_BRAKE_CURRENT_OUT_OF_RANGE", endpoint);
    }
    if (odrivestatus.motor_error & MOTOR_ERROR_MODULATION_MAGNITUDE)
    {
        PrintlnSerial_string("axis.motor.error = ERROR_MODULATION_MAGNITUDE", endpoint);
    }
    if (odrivestatus.motor_error & MOTOR_ERROR_BRAKE_DEADTIME_VIOLATION)
    {
        PrintlnSerial_string("axis.motor.error = ERROR_BRAKE_DEADTIME_VIOLATION", endpoint);
    }
    if (odrivestatus.motor_error & MOTOR_ERROR_UNEXPECTED_TIMER_CALLBACK)
    {
        PrintlnSerial_string("axis.motor.error = ERROR_UNEXPECTED_TIMER_CALLBACK", endpoint);
    }
    if (odrivestatus.motor_error & MOTOR_ERROR_CURRENT_SENSE_SATURATION)
    {
        PrintlnSerial_string("axis.motor.error = ERROR_CURRENT_SENSE_SATURATION", endpoint);
    }
    if (odrivestatus.motor_error & MOTOR_ERROR_INVERTER_OVER_TEMP)
    {
        PrintlnSerial_string("axis.motor.error = ERROR_PHASE_INVERTER_OVER_TEMP", endpoint);
    }
    if (odrivestatus.motor_error & MOTOR_ERROR_CURRENT_UNSTABLE)
    {
        PrintlnSerial_string("axis.motor.error = ERROR_CURRENT_UNSTABLE", endpoint);
    }

    if (odrivestatus.controller_error & CONTROLLER_ERROR_OVERSPEED)
    {
        PrintlnSerial_string("axis.controller.error = ERROR_OVERSPEED", endpoint);
    }

    if (odrivestatus.encoder_error & ENCODER_ERROR_UNSTABLE_GAIN)
    {
        PrintlnSerial_string("axis.encoder.error = ERROR_UNSTABLE_GAIN", endpoint);
    }
    if (odrivestatus.encoder_error & ENCODER_ERROR_CPR_OUT_OF_RANGE)
    {
        PrintlnSerial_string("axis.encoder.error = ERROR_CPR_OUT_OF_RANGE", endpoint);
    }
    if (odrivestatus.encoder_error & ENCODER_ERROR_NO_RESPONSE)
    {
        PrintlnSerial_string("axis.encoder.error = ERROR_NO_RESPONSE", endpoint);
    }
    if (odrivestatus.encoder_error & ENCODER_ERROR_UNSUPPORTED_ENCODER_MODE)
    {
        PrintlnSerial_string("axis.encoder.error = ERROR_UNSUPPORTED_ENCODER_MODE", endpoint);
    }
    if (odrivestatus.encoder_error & ENCODER_ERROR_ILLEGAL_HALL_STATE)
    {
        PrintlnSerial_string("axis.encoder.error = ERROR_ILLEGAL_HALL_STATE", endpoint);
    }
    if (odrivestatus.encoder_error & ENCODER_ERROR_INDEX_NOT_FOUND_YET)
    {
        PrintlnSerial_string("axis.encoder.error = ERROR_INDEX_NOT_FOUND_YET", endpoint);
    }
}
