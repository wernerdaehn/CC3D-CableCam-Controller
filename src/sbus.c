
#include "stm32f4xx_hal.h"
#include "clock_50Hz.h"
#include "config.h"
#include "sbus.h"
#include "protocol.h"
#include "math.h"
#include "string.h"


/*
 * Observations
 *
 * FrSky X8R
 * time between frames: 6ms.
 * time to send frame: 3ms.
*
 * Futaba R6208SB/R6303SB
 * time between frames: 11ms.
 * time to send frame: 3ms.
 */

#define SBUS_TIME_NEEDED_PER_FRAME 4


#define SBUS_STATE_FAILSAFE (1 << 0)
#define SBUS_STATE_SIGNALLOSS (1 << 1)

#define SBUS_FRAME_BEGIN_BYTE 0x0F
#define SBUS_FRAME_END_BYTE 0x00

#define SBUS_DIGITAL_CHANNEL_MIN 173
#define SBUS_DIGITAL_CHANNEL_MAX 1812

#define SBUS_FLAG_CHANNEL_16        (1 << 0)
#define SBUS_FLAG_CHANNEL_17        (1 << 1)
#define SBUS_FLAG_SIGNAL_LOSS       (1 << 2)
#define SBUS_FLAG_FAILSAFE_ACTIVE   (1 << 3)

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

void setServoValues(void);
uint16_t convertDutyIntoSBus(float duty, uint8_t channel);

sbusFrame_t sbusFrame;
sbusFrame_t sBusFrameGimbal;
uint32_t lastsbussend = 0;
uint16_t uart1_received_pos = 0;
uint16_t uart1_packet_start_pos = 0;

uint8_t uart1_rxbuffer[UART_CYCLIC_RXBUFFER_SIZE];

uint16_t dutyarray[64];
uint16_t duty_pos = 0;

sbusData_t sbusdata;

void UART1_init()
{
    if(HAL_UART_Receive_DMA(&huart1, uart1_rxbuffer, UART_CYCLIC_RXBUFFER_SIZE) != HAL_OK)
    {
        Error_Handler();
    }
}

void initSBusData(uint8_t receivertype)
{
    sbusdata.sbusLastValidFrame = 0L;
    sbusdata.receivertype = receivertype;

    sBusFrameGimbal.frame.chan8 = 0;
    sBusFrameGimbal.frame.chan9 = 0;
    sBusFrameGimbal.frame.chan10 = 0;
    sBusFrameGimbal.frame.chan11 = 0;
    sBusFrameGimbal.frame.chan12 = 0;
    sBusFrameGimbal.frame.chan13 = 0;
    sBusFrameGimbal.frame.chan14 = 0;
    sBusFrameGimbal.frame.chan15 = 0;
    sBusFrameGimbal.frame.flags = 0;
    sBusFrameGimbal.frame.syncByte = SBUS_FRAME_BEGIN_BYTE;
    sBusFrameGimbal.frame.endByte = SBUS_FRAME_END_BYTE;

    controllerstatus.duart_pause[1] = 0;
    controllerstatus.duart_valid[1] = 0;
    controllerstatus.duart_errors[1] = 0;
    controllerstatus.duart_frame_errors[1] = 0;
    controllerstatus.duart_parity_errors[1] = 0;
    controllerstatus.duart_noise_errors[1] = 0;
    controllerstatus.duart_overrun_errors[1] = 0;
}

void setGimbalValues(float channel_values[])
{
    sBusFrameGimbal.frame.chan0 = convertDutyIntoSBus(channel_values[0],0);
    sBusFrameGimbal.frame.chan1 = convertDutyIntoSBus(channel_values[1],1);
    sBusFrameGimbal.frame.chan2 = convertDutyIntoSBus(channel_values[2],2);
    sBusFrameGimbal.frame.chan3 = convertDutyIntoSBus(channel_values[3],3);
    sBusFrameGimbal.frame.chan4 = convertDutyIntoSBus(channel_values[4],4);
    sBusFrameGimbal.frame.chan5 = convertDutyIntoSBus(channel_values[5],5);
    sBusFrameGimbal.frame.chan6 = convertDutyIntoSBus(channel_values[6],6);
    sBusFrameGimbal.frame.chan7 = convertDutyIntoSBus(channel_values[7],7);
}

uint16_t convertDutyIntoSBus(float duty, uint8_t channel)
{
    if (isnan(duty))
    {
        return activesettings.rc_channel_sbus_out_default[channel];
    }
    else
    {
        return (uint16_t) ((duty * 800.0f) + 992.0f);
    }
}

void setServoValues()
{
    // if (sbusFrame.frame.syncByte == SBUS_FRAME_BEGIN_BYTE && sbusFrame.frame.endByte == SBUS_FRAME_END_BYTE)
    /*
     * Removed the end byte check as futaba receivers follow their own logic there
     */
    if (sbusFrame.frame.syncByte == SBUS_FRAME_BEGIN_BYTE)
    {
        sbusdata.sbusLastValidFrame = HAL_GetTick();
        sbusdata.servovalues[0].duty = (sbusFrame.frame.chan0);
        sbusdata.servovalues[1].duty = (sbusFrame.frame.chan1);
        sbusdata.servovalues[2].duty = (sbusFrame.frame.chan2);
        sbusdata.servovalues[3].duty = (sbusFrame.frame.chan3);
        sbusdata.servovalues[4].duty = (sbusFrame.frame.chan4);
        sbusdata.servovalues[5].duty = (sbusFrame.frame.chan5);
        sbusdata.servovalues[6].duty = (sbusFrame.frame.chan6);
        sbusdata.servovalues[7].duty = (sbusFrame.frame.chan7);
        sbusdata.servovalues[8].duty = (sbusFrame.frame.chan8);
        sbusdata.servovalues[9].duty = (sbusFrame.frame.chan9);
        sbusdata.servovalues[10].duty = (sbusFrame.frame.chan10);
        sbusdata.servovalues[11].duty = (sbusFrame.frame.chan11);
        sbusdata.servovalues[12].duty = (sbusFrame.frame.chan12);
        sbusdata.servovalues[13].duty = (sbusFrame.frame.chan13);
        sbusdata.servovalues[14].duty = (sbusFrame.frame.chan14);
        sbusdata.servovalues[15].duty = (sbusFrame.frame.chan15);
        sbusdata.channel16 = (sbusFrame.frame.flags & SBUS_FLAG_CHANNEL_16);
        sbusdata.channel17 = (sbusFrame.frame.flags & SBUS_FLAG_CHANNEL_17);
        sbusdata.signalloss = sbusFrame.frame.flags & SBUS_FLAG_SIGNAL_LOSS;
        sbusdata.failsafeactive = sbusFrame.frame.flags & SBUS_FLAG_FAILSAFE_ACTIVE;
        sbusdata.counter_sbus_valid_data++;
    }
}

/*
 * getDuty() returns the current duty value as percentage of the full stick range, removing the neutral range.
 * It returns NAN in case the value is too old or the RC does not send a valid channel value or a wrong channel is requested.
 */
float getDuty(uint8_t channel)
{
    if (HAL_GetTick() - sbusdata.sbusLastValidFrame > 3000L || sbusdata.failsafeactive)
    {
        // No valid signal for more than 3 seconds --> invalid reading
        return NAN;
    }
    else if (channel < SBUS_MAX_CHANNEL && controllerstatus.safemode != DISABLE_RC)
    {
        int16_t val = 0;
        if (sbusdata.servovalues[channel].duty != 0)
        {
            // realign the uint16 sbus data to be centered around zero, e.g. sbusdata = 1300, neutral_pos = 1000 --> val = +300
            val = sbusdata.servovalues[channel].duty - activesettings.stick_neutral_pos;
            if (val >= 0)
            {
                if (val <= activesettings.stick_neutral_range)
                {
                    // Values from 0...<neutral> are considered as idle --> val = 0
                    return 0;
                }
                else
                {
                    // The return value does not have a neutral range, so eliminate that
                    val -= activesettings.stick_neutral_range;
                    if (val > activesettings.stick_value_range)
                    {
                        // The percentage cannot be higher than +1.00
                        val = activesettings.stick_value_range;
                    }
                }
            }
            else
            {
                // The val is something between -<stick_vale_range>...-1
                if (val >= -activesettings.stick_neutral_range)
                {
                    // Within neutral range
                    return 0;
                }
                else
                {
                    val += activesettings.stick_neutral_range;
                    if (val < -activesettings.stick_value_range)
                    {
                        // The percentage cannot be lower than -1.00
                        val = -activesettings.stick_value_range;
                    }
                }
            }
            return ((float) val)/activesettings.stick_value_range;
        }
    }
    return NAN;
}

/*
 * getDutyIgnoreNeutral() returns the current duty value as percentage of the full stick range.
 * It returns NAN in case the value is too old or the RC does not send a valid channel value or a wrong channel is requested.
 */
float getDutyIgnoreNeutral(uint8_t channel)
{
    if (HAL_GetTick() - sbusdata.sbusLastValidFrame > 3000L || sbusdata.failsafeactive)
    {
        // No valid signal for more than 3 seconds --> invalid reading
        return NAN;
    }
    else if (channel < SBUS_MAX_CHANNEL && controllerstatus.safemode != DISABLE_RC)
    {
        int16_t val = 0;
        if (sbusdata.servovalues[channel].duty != 0)
        {
            // realign the uint16 sbus data to be centered around zero, e.g. sbusdata = 1300, neutral_pos = 1000 --> val = +300
            val = sbusdata.servovalues[channel].duty - activesettings.stick_neutral_pos;
            if (val > activesettings.stick_value_range)
            {
                // The percentage cannot be higher than +1.00
                val = activesettings.stick_value_range;
            }
            else if (val < -activesettings.stick_value_range)
            {
                // The percentage cannot be lower than -1.00
                val = -activesettings.stick_value_range;
            }
            return ((float) val)/activesettings.stick_value_range;
        }
    }
    return NAN;
}

uint8_t* getSBUSFrameAddress(void)
{
    return &sbusFrame.bytes[0];
}

uint8_t* getSBUSFrameGimbalAddress(void)
{
    return &sBusFrameGimbal.bytes[0];
}

void setNextDuty(uint16_t v)
{
    dutyarray[duty_pos++] = v;
    if (duty_pos >= 64)
    {
        duty_pos = 0;
    }
}


void printSBUSChannels(Endpoints endpoint)
{
    PrintSerial_string("Time: ", endpoint);
    PrintlnSerial_long(sbusdata.sbusLastValidFrame, endpoint);
    PrintSerial_string("  SBUS Errors: ", endpoint);
    PrintlnSerial_long(sbusdata.counter_sbus_errors, endpoint);

    PrintSerial_string("LastError: ", endpoint);
    PrintlnSerial_long(huart1.ErrorCode, endpoint);

    PrintlnSerial_hexstring(huart1.pRxBuffPtr, SBUS_FRAME_SIZE, endpoint);

    uint32_t age = HAL_GetTick() - sbusdata.sbusLastValidFrame;
    if (sbusdata.counter_sbus_frames == 0)
    {
        PrintlnSerial_string("Never received any SBUS data", endpoint);
    }
    else
    {
        if (age > 3000L)
        {
            PrintSerial_string("Data too old: ", endpoint);
            PrintlnSerial_long(age, endpoint);
        }
        else
        {
            for (int i = 0; i < SBUS_MAX_CHANNEL; i++)
            {
                PrintSerial_string("  ", endpoint);
                PrintSerial_int(sbusdata.servovalues[i].duty, endpoint);
            }
            PrintlnSerial(endpoint);
        }
    }

}

void PrintSumPPMRawData(Endpoints endpoint)
{
    PrintSerial_string("Duty & Pause Timing for SumPPM:", endpoint);
    for (int i = 0; i < 64; i++)
    {
        PrintSerial_string("  ", endpoint);
        PrintSerial_int(dutyarray[i], endpoint);
    }
    PrintlnSerial(endpoint);
}


void SBusSendCycle()
{
    uint32_t currenttick = HAL_GetTick();
    if (currenttick - lastsbussend >= 15) // && (huart6.gState && 0x01) == 0x00)
    {
        HAL_UART_Transmit_DMA(&huart6, &sBusFrameGimbal.bytes[0], SBUS_FRAME_SIZE);
        lastsbussend = currenttick;
    }
}

uint16_t UART1_Receive()
{
    uint16_t currentreceivepos = huart1.RxXferSize - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
    uint32_t now = HAL_GetTick();

    if (currentreceivepos != uart1_received_pos)
    {
        controllerstatus.duart_last_data[1] = now;
        uart1_received_pos = currentreceivepos;
    }
    else if (now < controllerstatus.duart_last_data[1] + 5)
    {
        // minimum packet send time is 5ms plus 15ms pause. It is requested every 200ms
    }
    else // no data within 20ms must be the packet end or no new data
    {
        if (uart1_packet_start_pos != currentreceivepos)
        {
            controllerstatus.duart_pause[1]++;
            uint16_t packetlength = mod16(currentreceivepos - uart1_packet_start_pos, huart1.RxXferSize);
            controllerstatus.duart_last_packet_length[1] = packetlength;
            if (uart1_packet_start_pos + SBUS_FRAME_SIZE < huart1.RxXferSize)
            {
                memcpy(sbusFrame.bytes, &uart1_rxbuffer[uart1_packet_start_pos], SBUS_FRAME_SIZE);
            }
            else
            {
                memcpy(&sbusFrame.bytes[0], &uart1_rxbuffer[uart1_packet_start_pos], huart1.RxXferSize-uart1_packet_start_pos);
                memcpy(&sbusFrame.bytes[huart1.RxXferSize-uart1_packet_start_pos], &uart1_rxbuffer[0], SBUS_FRAME_SIZE-(huart1.RxXferSize-uart1_packet_start_pos));
            }
            if (sbusFrame.bytes[0] == SBUS_FRAME_BEGIN_BYTE && packetlength == SBUS_FRAME_SIZE && sbusFrame.bytes[SBUS_FRAME_SIZE-1] == SBUS_FRAME_END_BYTE)
            {
                setServoValues();
                controllerstatus.duart_valid[1]++;
            }
            uart1_packet_start_pos = currentreceivepos;
        }
    }
    return 0;
}

