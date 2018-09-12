#include "vesc.h"
#include "stdio.h"
#include "string.h"
#include "serial_print.h"
#include "protocol.h"
#include "clock_50hz.h"
#include "math.h"

uint16_t crc16(uint8_t *buf, uint16_t len);
uint16_t crc16cyclic(uint8_t *buf, uint16_t buflen, uint16_t start, uint16_t len);
uint16_t mod16(int16_t number, int16_t base);

uint8_t uart2TxBuffer[VESC_RXBUFFER_SIZE];

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

uint16_t uart2_received_pos = 0;
uint16_t vesc_packet_start_pos = 0;
uint32_t vescvalues_requested_at = 0;
bool uart2_tx_ready = true;

// Receiving packets
getvalues_t vescvalues;

uint8_t vesc_rxbuffer[VESC_RXBUFFER_SIZE];
uint32_t vesc_last_data_timestamp = 0;

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
    requestvaluespacket.frame.crc = __REV16(crc16(&requestvaluespacket.bytes[2], requestvaluespacket.frame.length)); // As the packet is static tis can be done here


    currentbrakepacket.frame.startbyte = 0x02;
    currentbrakepacket.frame.length = 0x05;
    currentbrakepacket.frame.command = COMM_SET_CURRENT_BRAKE;
    currentbrakepacket.frame.stop = 0x03;

    if(HAL_UART_Receive_DMA(&huart2, vesc_rxbuffer, VESC_RXBUFFER_SIZE) != HAL_OK)
    {
        Error_Handler();
    }
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
    if (!controllerstatus.vesc_config) // No VESC Output when Bluetooth has control about the VESC
    {
        if (isnan(esc_output))
        {
            VESC_set_rpm(0);
        }
        else
        {
            int32_t vesc_erpm;
            if (activesettings.mode == MODE_PASSTHROUGH)
            {
                vesc_erpm = (int32_t) ((esc_output * ((float) activesettings.vesc_max_erpm)));
            }
            else
            {
                // controllerstatus.stick_max_speed reduces the vesc erpm by this factor at stick=100%
                vesc_erpm = (int32_t) ((esc_output * controllerstatus.stick_max_speed * ((float) activesettings.vesc_max_erpm)));
            }
            VESC_set_rpm(vesc_erpm);
        }
    }
}

void VESC_set_rpm(int32_t erpm)
{
    if (!controllerstatus.vesc_config) // No VESC Output when Bluetooth has control about the VESC
    {
        if (is5Hz())
        {
            UART2Append(requestvaluespacket.bytes, sizeof(requestvaluespacket.bytes));
            vescvalues_requested_at = HAL_GetTick();
        }
        else
        {
            rpmpacket.frame.speed = __REV(erpm);
            rpmpacket.frame.crc = __REV16(crc16(&rpmpacket.bytes[2], rpmpacket.frame.length));
            UART2Append((uint8_t *)&rpmpacket.frame, sizeof(rpmpacket.bytes));
        }
    }
}

/*
void VESC_set_handbrake_current(int32_t brake_current)
{
    if (!controllerstatus.vesc_config) // No VESC Output when Bluetooth has control about the VESC
    {
        handbrakepacket.frame.brakecurrent_1000 = __REV(brake_current*1000);
        handbrakepacket.frame.crc = __REV16(crc16(&handbrakepacket.bytes[2], handbrakepacket.frame.length));

        UART2Append(handbrakepacket.bytes, sizeof(handbrakepacket.bytes));
    }
}

void VESC_set_currentbrake_current(int32_t brake_current)
{
    if (!controllerstatus.vesc_config) // No VESC Output when Bluetooth has control about the VESC
    {
        currentbrakepacket.frame.brakecurrent_1000 = __REV(brake_current*1000);
        currentbrakepacket.frame.crc = __REV16(crc16(&currentbrakepacket.bytes[2], currentbrakepacket.frame.length));

        UART2Append(currentbrakepacket.bytes, sizeof(currentbrakepacket.bytes));
    }
}
*/

uint16_t crc16(uint8_t *buf, uint16_t len) {
	uint16_t i;
	uint16_t cksum = 0;
	for (i = 0; i < len; i++) {
		cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
	}
	return cksum;
}

uint16_t crc16cyclic(uint8_t *buf, uint16_t buflen, uint16_t start, uint16_t len) {
	uint16_t i;
	uint16_t cksum = 0;
	for (i = 0; i < len; i++) {
		cksum = crc16_tab[(((cksum >> 8) ^ buf[(start+i)%buflen]) & 0xFF)] ^ (cksum << 8);
	}
	return cksum;
}

uint16_t mod16(int16_t number, int16_t base)
{
    number = number % base;
    if (number < 0)
    {
        return base + number;
    }
    else
    {
        return number;
    }
}

uint16_t UART2_Receive()
{
    uint16_t currentreceivepos = huart2.RxXferSize - __HAL_DMA_GET_COUNTER(huart2.hdmarx);
    uint32_t now = HAL_GetTick();

    if (currentreceivepos != uart2_received_pos)
    {
        // uint16_t len = mod16(currentreceivepos - uart2_received_pos, huart2.RxXferSize);
        vesc_last_data_timestamp = now;
        uart2_received_pos = currentreceivepos;
    }
    else if (now < vesc_last_data_timestamp + 5)
    {
        // minimum packet send time is 5ms plus 15ms pause. It is requested every 200ms
    }
    else // no data within 20ms must be the packet end or no new data
    {
        if (vesc_packet_start_pos != currentreceivepos)
        {
            vescvalues_requested_at = 0;
            uint16_t packetlength = mod16(currentreceivepos - vesc_packet_start_pos, huart2.RxXferSize);
            if (vesc_rxbuffer[mod16(vesc_packet_start_pos, huart2.RxXferSize)] == 0x02)
            {
                uint16_t len = vesc_rxbuffer[mod16(vesc_packet_start_pos+1, huart2.RxXferSize)];
                if (len + 5 == packetlength)
                {
                    if (vesc_rxbuffer[mod16(currentreceivepos-1, huart2.RxXferSize)] == 0x03)
                    {
                        uint16_t crc_calculated = crc16cyclic(vesc_rxbuffer, VESC_RXBUFFER_SIZE, vesc_packet_start_pos+2, len); // payload data starts at index 2, the vesc_rxbuffer[1] contains the payload length byte
                        uint16_t crc_data = (vesc_rxbuffer[mod16(currentreceivepos-3, huart2.RxXferSize)] << 8) + vesc_rxbuffer[mod16(currentreceivepos-2, huart2.RxXferSize)];
                        if (crc_calculated == crc_data)
                        {
                            switch (vesc_rxbuffer[mod16(vesc_packet_start_pos+2, huart2.RxXferSize)])
                            {
                            case COMM_GET_VALUES:
                                {
                                    uint16_t start = mod16(vesc_packet_start_pos+3, huart2.RxXferSize);
                                    if (start + GETVALUES_SIZE < huart2.RxXferSize)
                                    {
                                        memcpy(&vescvalues.bytes[0], &vesc_rxbuffer[start], GETVALUES_SIZE);
                                    }
                                    else
                                    {
                                        memcpy(&vescvalues.bytes[0], &vesc_rxbuffer[start], huart2.RxXferSize-start);
                                        memcpy(&vescvalues.bytes[huart2.RxXferSize-start], &vesc_rxbuffer[0], GETVALUES_SIZE-(huart2.RxXferSize-start));
                                    }
                                }
                            }
                        }
                        else
                        {
                            /* PrintSerial_string("crc invalid", EndPoint_USB);
                            PrintSerial_int(crc_data, EndPoint_USB);
                            PrintSerial_int(crc_calculated, EndPoint_USB); */
                        }
                    }
                    else
                    {
                        /* PrintSerial_string("end byte wrong", EndPoint_USB);
                        PrintSerial_hexchar(vesc_rxbuffer[mod16(currentreceivepos-1, huart2.RxXferSize)], EndPoint_USB);
                        PrintlnSerial(EndPoint_USB); */
                    }
                }
                else
                {
                    /* PrintSerial_string("packet length != requested length", EndPoint_USB);
                    PrintSerial_int(len+5, EndPoint_USB);
                    PrintlnSerial_int(packetlength, EndPoint_USB); */
                }
            }
            else
            {
                /* PrintSerial_string("invalid start byte", EndPoint_USB);
                PrintSerial_hexchar(vesc_rxbuffer[mod16(vesc_packet_start_pos, huart2.RxXferSize)], EndPoint_USB);
                PrintlnSerial(EndPoint_USB); */
            }
        }
        vesc_packet_start_pos = currentreceivepos;
    }
    return 0;
}

/*
 * VESC supports sending single packets only, makes no sense using a cyclic send buffer
 */
void UART2Append(uint8_t *ptr, uint32_t len)
{
//    if (uart2_tx_ready)
//    {
        memcpy(uart2TxBuffer, ptr, len);
        uart2_tx_ready = false;
        HAL_UART_Transmit_DMA(&huart2, uart2TxBuffer, len);
//    }
}

void UART2Flush()
{
}
