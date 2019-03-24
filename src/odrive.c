#include "odrive.h"
#include "stdio.h"
#include "string.h"
#include "serial_print.h"
#include "protocol.h"
#include "clock_50hz.h"
#include "math.h"
#include "uart_callback.h"

void ODRIVE_set_rpm(int32_t odrive_erpm);
bool ODRIVE_Request_Value(ODRIVE_REQUEST_t requesttype);
void ODRIVESendLine(const char *ptr);

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
        if (ODRIVE_Request_Value(ODRIVE_REQUESTED_MOTOR_CALIBRATED))
        {
            if (odrive_rxbuffer[0] == '1')
            {
                controllerstatus.odrivestate = ODRIVE_MOTOR_CALIBRATED;
            }
            else if (odrive_rxbuffer[0] == '0')
            {
                controllerstatus.odrivestate = ODRIVE_MOTOR_TO_BE_CALIBRATED;
            }
        }
        break;
    case ODRIVE_MOTOR_CALIBRATED:
        if (ODRIVE_Request_Value(ODRIVE_REQUESTED_ENCODER_CALIBRATED))
        {
            if (odrive_rxbuffer[0] == '1')
            {
                controllerstatus.odrivestate = ODRIVE_ENCODER_CALIBRATED;
            }
        }
        break;
    case ODRIVE_ENCODER_CALIBRATED:
        if (ODRIVE_Request_Value(ODRIVE_REQUESTED_CLOSED_LOOP))
        {
            if (odrive_rxbuffer[0] == '8')
            {
                controllerstatus.odrivestate = ODRIVE_SPEED_MODE;
            }
        }
        break;
    case ODRIVE_SPEED_MODE:
        if (ODRIVE_Request_Value(ODRIVE_REQUESTED_SPEED_CONTROL_MODE))
        {
            if (odrive_rxbuffer[0] == '2') // 2...velocity control
            {
                controllerstatus.odrivestate = ODRIVE_CLOSED_LOOP;
            }
        }
        break;
    case ODRIVE_CLOSED_LOOP:
        if (ODRIVE_Request_Value(ODRIVE_REQUESTED_SPEED_LIMIT))
        {
            float p;
            uint16_t argument_index = sscanf((char *) odrive_rxbuffer, "%f", &p);
            if (argument_index == 1)
            {
                if (p < activesettings.esc_max_speed) // if the programmed speed is higher than the hard limit set in the odrive, use this
                {
                    activesettings.esc_max_speed = p;
                }
                controllerstatus.esc_max_speed = p; // controllerstatus contains the read value
            }
            controllerstatus.odrivestate = ODRIVE_OPERATIONAL;
        }
        break;
    case ODRIVE_OPERATIONAL:
       if (isnan(esc_output))
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

ODRIVE_REQUEST_t getCurrentODriveValueType()
{
    return odrivecurrentvalue;
}

char * getCurrentODriveResponse()
{
    return (char *) odrive_rxbuffer;
}

bool ODRIVE_Request_Value(ODRIVE_REQUEST_t requesttype)
{
    if (HAL_GetTick() > last_odrive_read_request + 1000L)
    {
        odriverequested = ODRIVE_REQUESTED_NONE;
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
        default:
            break;
        }
        return false;
    }
    else if (odrivecurrentvalue == requesttype)
    {
        odriverequested = ODRIVE_REQUESTED_NONE;
        return true;
    }
    else if (odrivecurrentvalue != ODRIVE_REQUESTED_NONE)
    {
        return false;
    }
    else
    {
        return false;
    }
}


uint16_t ODRIVEReceive()
{
    uint16_t currentreceivepos = huart2.RxXferSize - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    uint32_t now = HAL_GetTick();

    if (currentreceivepos != odrive_received_pos)
    {
        odrive_last_data_timestamp = now;
        odrive_received_pos = currentreceivepos;
    }
    else if (now < odrive_last_data_timestamp + 5)
    {
        // minimum packet send time is 5ms plus 15ms pause. It is requested every 200ms
    }
    else // no data within 20ms must be the packet end or no new data
    {
        if (odrive_packet_start_pos != currentreceivepos)
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
        }
    }
    return 0;
}


void ODRIVESendPacket(uint8_t *ptr, uint32_t len)
{
    // TX start without checking if the previous TX did complete for safety reasons. Since this is called at fixed intervals, data was transmitted already for sure
    memcpy(odriveTxBuffer, ptr, len);
    HAL_UART_Transmit_DMA(&huart2, odriveTxBuffer, len);
}

void ODRIVESendLine(const char *ptr)
{
    HAL_UART_Transmit_DMA(&huart2, (uint8_t *) ptr, strlen(ptr));
}

