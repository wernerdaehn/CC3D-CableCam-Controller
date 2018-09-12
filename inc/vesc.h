#ifndef VESC_H_INCLUDED
#define VESC_H_INCLUDED
#include "stm32f4xx_hal.h"


#define VESC_RXBUFFER_SIZE 512


#define GETVALUES_SIZE 53


void VESC_init(void);
void VESC_Output(float esc_output);
void VESC_set_rpm(int32_t erpm);
// void VESC_set_handbrake_current(int32_t brake_current);
// void VESC_set_currentbrake_current(int32_t brake_current);
void VESC_request_values(void);
uint8_t* getRequestValuePacketFrameAddress(void);
float vesc_get_float(uint16_t uartfield, float scale);
double vesc_get_double(uint32_t uartfield, double scale);
int32_t vesc_get_long(uint32_t uartfield);
int16_t vesc_get_int(uint16_t uartfield);
uint16_t UART2_Receive(void);

void UART2Append(uint8_t *ptr, uint32_t len);
void UART2Flush(void);


typedef enum {
	FAULT_CODE_NONE = 0,
	FAULT_CODE_OVER_VOLTAGE,
	FAULT_CODE_UNDER_VOLTAGE,
	FAULT_CODE_DRV8302,
	FAULT_CODE_ABS_OVER_CURRENT,
	FAULT_CODE_OVER_TEMP_FET,
	FAULT_CODE_OVER_TEMP_MOTOR
} vesc_fault_code;

// VESC Types
// Example:
//  0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55 56 57 58 59 60 61 62
// 02 3a 04 01 1a 00 f3 00 00 01 ba 00 00 00 66 00 00 00 00 ff ff fe 46 fe f8 ff ff e0 2c 00 71 00 00 00 2e 00 00 00 00 00 00 02 12 00 00 00 00 ff ff e0 bb 00 00 3c 2f 00 09 69 58 d0 cc 6f 03
// 02 3b 04 01 4c 01 1b ff ff ff df 00 00 00 00 00 00 00 1c ff ff ff df 00 00 00 00 00 00 00 e1 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 ff ff ff fe 00 00 00 04 00 05 94 38 f8 00 8a b0 03
// getvalues_s size = 1*1 + 4*2 + 11*4 = 1 + 8 + 44 = 53
// 0x3a = 58
// The struct is using uint's at this point in time because they bytes have not been swapped big/little endian.
struct getvalues_s {
    uint16_t         temp_fet_10;                // 0x011a
	uint16_t         temp_motor_10;              // 0x00f3
	uint32_t         avg_motor_current_100;      // 0x000001ba (7-10)
	uint32_t         avg_input_current_100;      // 0x00000066 (11-14)
    uint32_t         avg_id_100;                 // 0x00000000 (15-18)
    uint32_t         avg_iq_100;                 // 0xfffffe46 (19-22)
    uint16_t         duty_now_1000;              // 0xfef8
    uint32_t         rpm_1;                      // 0xffffe02c (25-28)
	uint16_t         v_in_10;                    // 0x0071
    uint32_t         amp_hours_10000;            // 0x0000002e (31-34)
    uint32_t         amp_hours_charged_10000;    // 0x00000000 (35-38)
    uint32_t         watt_hours_10000;           // 0x00000212 (39-42)
    uint32_t         watt_hours_charged_10000;   // 0x00000000 (43-46)
    uint32_t         tachometer;                 // 0xffffe0bb (47-50)
    uint32_t         tachometer_abs;             // 0x00003c2f (51-54)
    vesc_fault_code  fault_code;                 // 00
    // more unknown data
} __attribute__ ((__packed__));


typedef union {
    uint8_t bytes[GETVALUES_SIZE];
    struct getvalues_s frame;
} getvalues_t;


typedef enum {
	COMM_FW_VERSION = 0,
	COMM_JUMP_TO_BOOTLOADER,
	COMM_ERASE_NEW_APP,
	COMM_WRITE_NEW_APP_DATA,
	COMM_GET_VALUES,
	COMM_SET_DUTY,
	COMM_SET_CURRENT,
	COMM_SET_CURRENT_BRAKE,
	COMM_SET_RPM,
	COMM_SET_POS,
	COMM_SET_HANDBRAKE,
	COMM_SET_DETECT,
	COMM_SET_SERVO_POS,
	COMM_SET_MCCONF,
	COMM_GET_MCCONF,
	COMM_GET_MCCONF_DEFAULT,
	COMM_SET_APPCONF,
	COMM_GET_APPCONF,
	COMM_GET_APPCONF_DEFAULT,
	COMM_SAMPLE_PRINT,
	COMM_TERMINAL_CMD,
	COMM_PRINT,
	COMM_ROTOR_POSITION,
	COMM_EXPERIMENT_SAMPLE,
	COMM_DETECT_MOTOR_PARAM,
	COMM_DETECT_MOTOR_R_L,
	COMM_DETECT_MOTOR_FLUX_LINKAGE,
	COMM_DETECT_ENCODER,
	COMM_DETECT_HALL_FOC,
	COMM_REBOOT,
	COMM_ALIVE,
	COMM_GET_DECODED_PPM,
	COMM_GET_DECODED_ADC,
	COMM_GET_DECODED_CHUK,
	COMM_FORWARD_CAN,
	COMM_SET_CHUCK_DATA,
	COMM_CUSTOM_APP_DATA,
	COMM_NRF_START_PAIRING
} COMM_PACKET_ID;

struct sendrpm_s {
    uint8_t     startbyte;
    uint8_t     length;
    uint8_t     command;
    int32_t     speed;
    uint16_t    crc;
    uint8_t     stop;
} __attribute__ ((__packed__));


typedef union {
    uint8_t bytes[10];
    struct sendrpm_s frame;
} sendrpm_t;


struct sendhandbrake_s {
    uint8_t     startbyte;
    uint8_t     length;
    uint8_t     command;
    int32_t     brakecurrent_1000;
    uint16_t    crc;
    uint8_t     stop;
} __attribute__ ((__packed__));

typedef union {
    uint8_t bytes[10];
    struct sendhandbrake_s frame;
} sendhandbrake_t;


struct requestvalues_s {
    uint8_t     startbyte;
    uint8_t     length;
    uint8_t     command;
    uint16_t    crc;
    uint8_t     stop;
} __attribute__ ((__packed__));

typedef union {
    uint8_t bytes[6];
    struct requestvalues_s frame;
} requestvalues_t;


struct sendcurrentbrake_s {
    uint8_t     startbyte;
    uint8_t     length;
    uint8_t     command;
    int32_t     brakecurrent_1000;
    uint16_t    crc;
    uint8_t     stop;
} __attribute__ ((__packed__));

typedef union {
    uint8_t bytes[10];
    struct sendcurrentbrake_s frame;
} sendcurrentbrake_t;
#endif /* VESC_H_INCLUDED */
