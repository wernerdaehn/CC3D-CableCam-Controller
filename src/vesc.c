#include "vesc.h"
#include "stdio.h"
#include "string.h"
#include "serial_print.h"
#include "protocol.h"
#include "clock_50hz.h"
#include "math.h"

uint16_t crc16(uint8_t *buf, uint16_t len);

// CRC Table
const unsigned short crc16_tab[] = { 0x0000, 0x1021, 0x2042, 0x3063, 0x4084,
		0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad,
		0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7,
		0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
		0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485, 0xa56a,
		0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672,
		0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719,
		0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7,
		0x0840, 0x1861, 0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948,
		0x9969, 0xa90a, 0xb92b, 0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50,
		0x3a33, 0x2a12, 0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b,
		0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
		0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49, 0x7e97,
		0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70, 0xff9f, 0xefbe,
		0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9, 0xb1ca,
		0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
		0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d,
		0xd31c, 0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214,
		0x6277, 0x7256, 0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c,
		0xc50d, 0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
		0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3,
		0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634, 0xd94c, 0xc96d,
		0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab, 0x5844, 0x4865, 0x7806,
		0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e,
		0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1,
		0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b,
		0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0,
		0x0cc1, 0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
		0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0 };


// Packets to send
sendrpm_t rpmpacket;
sendhandbrake_t handbrakepacket;
requestvalues_t requestvaluespacket;
sendcurrentbrake_t currentbrakepacket;

// Receiving packets
getvalues_t vescvalues;

uint8_t vesc_rxbuffer[VESC_RXBUFFER_SIZE];
uint32_t vesc_packet_start_timestamp;
int32_t tacho_old;
int16_t handbrake_current;

extern UART_HandleTypeDef huart2;

void VESC_init()
{
    rpmpacket.frame.startbyte = 0x02;
    rpmpacket.frame.length = 0x05;
    rpmpacket.frame.command = COMM_SET_RPM;
    rpmpacket.frame.stop = 0x03;

    handbrakepacket.frame.startbyte = 0x02;
    handbrakepacket.frame.length = 0x05;
    handbrakepacket.frame.command = COMM_SET_HANDBRAKE;
    handbrakepacket.frame.stop = 0x03;

    requestvaluespacket.frame.startbyte = 0x02;
    requestvaluespacket.frame.length = 0x01;
    requestvaluespacket.frame.command = COMM_GET_VALUES;
    requestvaluespacket.frame.stop = 0x03;

    currentbrakepacket.frame.startbyte = 0x02;
    currentbrakepacket.frame.length = 0x05;
    currentbrakepacket.frame.command = COMM_SET_CURRENT_BRAKE;
    currentbrakepacket.frame.stop = 0x03;

    handbrake_current = activesettings.vesc_brake_handbrake_max;
}

uint8_t* getRequestValuePacketFrameAddress(void)
{
    return vesc_rxbuffer;
}

float vesc_get_float(uint16_t uartfield, float scale)
{
    int16_t swapped = (int16_t) __REV16(uartfield);
    return ((float) swapped)/scale;
}

double vesc_get_double(uint32_t uartfield, double scale)
{
    int32_t swapped = (int32_t) __REV(uartfield);
    return ((double) swapped)/scale;
}

int32_t vesc_get_long(uint32_t uartfield)
{
    return (int32_t) __REV(uartfield);
}

int16_t vesc_get_int(uint16_t uartfield)
{
    return (int16_t) __REV16(uartfield);
}


void VESC_Output(float esc_output)
{
    VESC_request_values();
    int32_t tacho_current = (int32_t) __REV(vescvalues.frame.tachometer_abs);
    if (esc_output == 0.0f || isnan(esc_output))
    {
        int32_t diff = (float) (tacho_current - tacho_old);
        if (diff >= activesettings.vesc_brake_min_speed)
        {
            // We are moving fast and the target speed is zero, e.g. due to an emergency brake
            // Therefore brake as hard as you can before enabling the handbrake
            VESC_set_rpm(0);
        }
        else
        {
            handbrake_current += diff - 1;

            if (handbrake_current > activesettings.vesc_brake_handbrake_max)
            {
                handbrake_current = activesettings.vesc_brake_handbrake_max;
            }
            else if (handbrake_current < activesettings.vesc_brake_handbrake_min)
            {
                handbrake_current = activesettings.vesc_brake_handbrake_min;
            }
            VESC_set_handbrake_current(handbrake_current);
        }
    }
    else
    {
        // controllerstatus.stick_max_speed reduces the vesc erpm by this factor at stick=100%
        int32_t vesc_erpm = (int32_t) ((esc_output * controllerstatus.stick_max_speed * ((float) activesettings.vesc_max_erpm)));
        VESC_set_rpm(vesc_erpm);
        handbrake_current = activesettings.vesc_brake_handbrake_max;
    }
    tacho_old = tacho_current;
}

void VESC_set_rpm(int32_t erpm)
{
    rpmpacket.frame.speed = __REV(erpm);
    rpmpacket.frame.crc = __REV16(crc16(&rpmpacket.bytes[2], rpmpacket.frame.length));

    HAL_UART_Transmit(&huart2, rpmpacket.bytes, sizeof(rpmpacket.bytes), 1000);
}

void VESC_set_handbrake_current(int32_t brake_current)
{
    handbrakepacket.frame.brakecurrent_1000 = __REV(brake_current*1000);
    handbrakepacket.frame.crc = __REV16(crc16(&handbrakepacket.bytes[2], handbrakepacket.frame.length));

    HAL_UART_Transmit(&huart2, handbrakepacket.bytes, sizeof(handbrakepacket.bytes), 1000);
}

void VESC_set_currentbrake_current(int32_t brake_current)
{
    currentbrakepacket.frame.brakecurrent_1000 = __REV(brake_current*1000);
    currentbrakepacket.frame.crc = __REV16(crc16(&currentbrakepacket.bytes[2], currentbrakepacket.frame.length));

    HAL_UART_Transmit(&huart2, currentbrakepacket.bytes, sizeof(currentbrakepacket.bytes), 1000);
}

uint16_t crc16(uint8_t *buf, uint16_t len) {
	uint16_t i;
	uint16_t cksum = 0;
	for (i = 0; i < len; i++) {
		cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
	}
	return cksum;
}

void VESC_request_values()
{
    requestvaluespacket.frame.crc = __REV16(crc16(&requestvaluespacket.bytes[2], requestvaluespacket.frame.length));
    HAL_UART_Transmit(&huart2, requestvaluespacket.bytes, sizeof(requestvaluespacket.bytes), 1000);
}


uint16_t packetend = VESC_RXBUFFER_SIZE;

void VESC_IRQHandler(UART_HandleTypeDef *huart)
{
    uint32_t isrflags   = READ_REG(huart->Instance->SR);
    uint32_t cr1its     = READ_REG(huart->Instance->CR1);
    uint32_t cr3its     = READ_REG(huart->Instance->CR3);
    uint32_t errorflags = 0x00U;

    uint8_t byteReceived = (uint8_t)(huart->Instance->DR & (uint16_t)0x00FF);

    /* If no error occurs */
    errorflags = (isrflags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
    if(errorflags == RESET)
    {
        /* UART in mode Receiver -------------------------------------------------*/
        if(((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
        {
            uint32_t now = HAL_GetTick();
            uint32_t frameduration = now - vesc_packet_start_timestamp;

            if (frameduration > 500)
            {
                huart->pRxBuffPtr = vesc_rxbuffer;
                huart->RxXferCount = VESC_RXBUFFER_SIZE;
                vesc_packet_start_timestamp = now;
                packetend = VESC_RXBUFFER_SIZE;
            }

            if (huart->RxXferCount > 0)
            {
                *huart->pRxBuffPtr = byteReceived;

                if (huart->RxXferCount == VESC_RXBUFFER_SIZE && *huart->pRxBuffPtr != 0x02)
                {
                    /* It is a first byte and it is not a valid start-byte, so we are in the middle of a frame.
                     *	 Hence do not forward the pointer but wait for a valid start byte.
                     */
                }
                else
                {
                    huart->RxXferCount--;
                    huart->pRxBuffPtr += 1U;
                    if (huart->RxXferCount == VESC_RXBUFFER_SIZE-2)
                    {
                       /*
                         * The length byte
                         */
                        packetend = VESC_RXBUFFER_SIZE - byteReceived - 2 - 3;
                    }
                    else if (huart->RxXferCount == packetend)
                    {
                        // Last byte of the frame has to be the stop byte
                        if (byteReceived == 0x03)
                        {
                            uint16_t crc_calculated = crc16(&vesc_rxbuffer[2], vesc_rxbuffer[1]); // payload data starts at index 2, the vesc_rxbuffer[1] contains the payload length byte
                            uint16_t crc_data = (vesc_rxbuffer[VESC_RXBUFFER_SIZE-packetend-3] << 8) + vesc_rxbuffer[VESC_RXBUFFER_SIZE-packetend-2];
                            if (crc_calculated == crc_data)
                            {
                                switch (vesc_rxbuffer[2])
                                {
                                case COMM_GET_VALUES:
                                    {
                                        if (vesc_rxbuffer[1] >= GETVALUES_SIZE)
                                        {
                                            // PrintlnSerial_hexstring(vesc_rxbuffer, VESC_RXBUFFER_SIZE-packetend, EndPoint_All);
                                            memcpy(&vescvalues.bytes[0], &vesc_rxbuffer[3], GETVALUES_SIZE);
                                            // PrintlnSerial_hexstring(vescvalues.bytes, GETVALUES_SIZE, EndPoint_All);
                                        }
                                    }
                                    break;
                                default:
                                    break;
                                }
                            }
                        }
                        huart->pRxBuffPtr = vesc_rxbuffer;
                        huart->RxXferCount = VESC_RXBUFFER_SIZE;
                    }
                }
            } // else overflow happened
            else
            {
                huart->pRxBuffPtr = vesc_rxbuffer;
                huart->RxXferCount = VESC_RXBUFFER_SIZE;
            }
        }
    }
    else
    {
        uint8_t haserror = 0;
        /* UART parity error interrupt occurred ----------------------------------*/
        if(((isrflags & USART_SR_PE) != RESET) && ((cr1its & USART_CR1_PEIE) != RESET))
        {
            huart->ErrorCode |= HAL_UART_ERROR_PE;
            __HAL_UART_CLEAR_PEFLAG(huart);
            haserror = 1;
        }

        /* UART noise error interrupt occurred -----------------------------------*/
        if(((isrflags & USART_SR_NE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
        {
            huart->ErrorCode |= HAL_UART_ERROR_NE;
            __HAL_UART_CLEAR_NEFLAG(huart);
            haserror = 1;
        }

        /* UART frame error interrupt occurred -----------------------------------*/
        if(((isrflags & USART_SR_FE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
        {
            huart->ErrorCode |= HAL_UART_ERROR_FE;
            __HAL_UART_CLEAR_FEFLAG(huart);
            haserror = 1;
        }

        /* UART Over-Run interrupt occurred --------------------------------------*/
        if(((isrflags & USART_SR_ORE) != RESET) && ((cr3its & USART_CR3_EIE) != RESET))
        {
            huart->ErrorCode |= HAL_UART_ERROR_ORE;
            __HAL_UART_CLEAR_OREFLAG(huart);
            haserror = 1;
        }
        if (haserror != 0)
        {
            // sbusdata.counter_sbus_errors++;
            huart->pRxBuffPtr = vesc_rxbuffer;
            huart->RxXferCount = VESC_RXBUFFER_SIZE;
            packetend = VESC_RXBUFFER_SIZE;
        }

    } /* End if some error occurs */

}
