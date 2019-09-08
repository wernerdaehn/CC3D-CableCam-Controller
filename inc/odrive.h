#ifndef ODRIVE_H_INCLUDED
#define ODRIVE_H_INCLUDED
#include "stm32f4xx_hal.h"
#include "config.h"
#include "serial_print.h"


typedef enum {
	ODRIVE_UNKNOWN = 0,
	ODRIVE_MOTOR_CALIBRATED,
	ODRIVE_MOTOR_TO_BE_CALIBRATED,
	ODRIVE_ENCODER_CALIBRATED,
	ODRIVE_SPEED_MODE,
	ODRIVE_CLOSED_LOOP,
	ODRIVE_OPERATIONAL
} ODRIVE_STATE_t;

typedef enum {
    ODRIVE_REQUESTED_NONE = 0,
    ODRIVE_REQUESTED_MOTOR_CALIBRATED,
    ODRIVE_REQUESTED_MOTOR_CALIBRATION,
    ODRIVE_REQUESTED_ENCODER_CALIBRATED,
    ODRIVE_REQUESTED_CLOSED_LOOP,
    ODRIVE_REQUESTED_SPEED_CONTROL_MODE,
    ODRIVE_REQUESTED_SPEED_LIMIT,
    ODRIVE_REQUESTED_POSITION,
    ODRIVE_REQUESTED_ERRORCODE,
    ODRIVE_REQUESTED_ERRORCODEMOTOR,
    ODRIVE_REQUESTED_ERRORCODECONTROLLER,
    ODRIVE_REQUESTED_ERRORCODEENCODER
} ODRIVE_REQUEST_t;

typedef struct
{
    int32_t motor_calibrated;
    int32_t motor_calibration;
    int32_t encoder_calibrated;
    int32_t closed_loop;
    int32_t speed_control_loop;
    int32_t speed_limit;
    int32_t position;
    int32_t global_error;
    int32_t motor_error;
    int32_t controller_error;
    int32_t encoder_error;
} ODRIVE_STATUS_t;

enum Axis_Error_t {
    AXIS_ERROR_NONE = 0x00,
    AXIS_ERROR_INVALID_STATE = 0x01, //<! an invalid state was requested
    AXIS_ERROR_DC_BUS_UNDER_VOLTAGE = 0x02,
    AXIS_ERROR_DC_BUS_OVER_VOLTAGE = 0x04,
    AXIS_ERROR_CURRENT_MEASUREMENT_TIMEOUT = 0x08,
    AXIS_ERROR_BRAKE_RESISTOR_DISARMED = 0x10, //<! the brake resistor was unexpectedly disarmed
    AXIS_ERROR_MOTOR_DISARMED = 0x20, //<! the motor was unexpectedly disarmed
    AXIS_ERROR_MOTOR_FAILED = 0x40, // Go to motor.hpp for information, check odrvX.axisX.motor.error for error value
    AXIS_ERROR_SENSORLESS_ESTIMATOR_FAILED = 0x80,
    AXIS_ERROR_ENCODER_FAILED = 0x100, // Go to encoder.hpp for information, check odrvX.axisX.encoder.error for error value
    AXIS_ERROR_CONTROLLER_FAILED = 0x200,
    AXIS_ERROR_POS_CTRL_DURING_SENSORLESS = 0x400,
    AXIS_ERROR_WATCHDOG_TIMER_EXPIRED = 0x800,
};

enum Motor_Error_t {
    MOTOR_ERROR_NONE = 0,
    MOTOR_ERROR_PHASE_RESISTANCE_OUT_OF_RANGE = 0x0001,
    MOTOR_ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE = 0x0002,
    MOTOR_ERROR_ADC_FAILED = 0x0004,
    MOTOR_ERROR_DRV_FAULT = 0x0008,
    MOTOR_ERROR_CONTROL_DEADLINE_MISSED = 0x0010,
    MOTOR_ERROR_NOT_IMPLEMENTED_MOTOR_TYPE = 0x0020,
    MOTOR_ERROR_BRAKE_CURRENT_OUT_OF_RANGE = 0x0040,
    MOTOR_ERROR_MODULATION_MAGNITUDE = 0x0080,
    MOTOR_ERROR_BRAKE_DEADTIME_VIOLATION = 0x0100,
    MOTOR_ERROR_UNEXPECTED_TIMER_CALLBACK = 0x0200,
    MOTOR_ERROR_CURRENT_SENSE_SATURATION = 0x0400,
    MOTOR_ERROR_INVERTER_OVER_TEMP = 0x0800,
    MOTOR_ERROR_CURRENT_UNSTABLE = 0x1000
};

enum Controller_Error_t {
    CONTROLLER_ERROR_NONE = 0,
    CONTROLLER_ERROR_OVERSPEED = 0x01,
};

 enum Encoder_Error_t {
    ENCODER_ERROR_NONE = 0,
    ENCODER_ERROR_UNSTABLE_GAIN = 0x01,
    ENCODER_ERROR_CPR_OUT_OF_RANGE = 0x02,
    ENCODER_ERROR_NO_RESPONSE = 0x04,
    ENCODER_ERROR_UNSUPPORTED_ENCODER_MODE = 0x08,
    ENCODER_ERROR_ILLEGAL_HALL_STATE = 0x10,
    ENCODER_ERROR_INDEX_NOT_FOUND_YET = 0x20,
};


void ODRIVE_init(void);
void ODRIVE_Output(float esc_output);

void ODRIVE_request_values(void);
uint16_t ODRIVEReceive(void);
void ODRIVESendPacket(uint8_t *ptr, uint32_t len);
char * getCurrentODriveResponse(void);
ODRIVE_REQUEST_t getCurrentODriveValueType(void);
void ODRIVESendLine(const char *ptr);
void ODRIVE_Print_Errors(Endpoints endpoint);

#endif /* ODRIVE_H_INCLUDED */
