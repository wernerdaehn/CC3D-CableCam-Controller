


#include "main.h"
#include <stdio.h>
#include "stm32_systick.h"
#include "stm32_servo_input.h"
#include "stm32_servo_output.h"
#include "controller.h"
#include "debug.h"
#include "stm32_encoder.h"
#include "protocol.h"
#include "stm32_uart.h"
#include "eeprom.h"
#include "clock_50Hz.h"
#include "config.h"
#include "string.h"
#include "sbus.h"
#include "usbd_cdc_interface.h"


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* Private functions ---------------------------------------------------------*/


GPIO_InitTypeDef GPIO_InitStructure;
USBD_HandleTypeDef USBD_Device;

int bootphase = 0;

// void InputCapture(void);

int main(void)
{
	char buf[APP_TX_BUF_SIZE];

	HAL_Init();
	// SystemInit();
	SystemClock_Config();


	Systick_Init();

	/* Init Device Library */
	USBD_Init(&USBD_Device, &VCP_Desc, 0);

	/* Add Supported Class */
	USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);

	/* Add CDC Interface Class */
	USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);

	/* Start Device Process */
	USBD_Start(&USBD_Device);

	strcpy(activesettings.version, "2015.06.01");
	activesettings.max_accel = MAX_ACCELERATION_MOVEMENT;
	activesettings.max_speed = MAX_SPEED_MOVEMENT;
	activesettings.P = 0.700;
	activesettings.I = 0.250;
	activesettings.D = 0.010;
	activesettings.neutrallow = 1454;
	activesettings.neutralhigh = 1538;
	activesettings.speed_rc_channel = RC_INPUT_ESC;
	activesettings.yaw_rc_channel = RC_INPUT_YAW;
	activesettings.pitch_rc_channel = RC_INPUT_PITCH;
	activesettings.aux_rc_channel = RC_INPUT_AUX;
	activesettings.debuglevel = DEBUG_LEVEL_MID;
	activesettings.esc_direction = +1;
	activesettings.rs_speed_signal_max_acceleration = 2.0f;
	activesettings.tolerated_positional_error_limit = 1.0f;
	activesettings.tolerated_positional_error_exceeded = 500.0f;
	activesettings.auxvalue = 1500;


	ICPWM_Init();
	PWM_Init();

	initController();
	initProtocol();

	ENCODER_Init();

	Uart_Init(38400);
	sbusInit();

	eeprom_init();





	PrintlnSerial_string("Initialization done", EndPoint_All);

	eeprom_read_sector((uint8_t *)&defaultsettings, sizeof(defaultsettings), EEPROM_SECTOR_FOR_SETTINGS);
	if (strncmp(activesettings.version, defaultsettings.version, sizeof(activesettings.version)) == 0) {
		// Version is the same
		memcpy(&activesettings, &defaultsettings, sizeof(defaultsettings));
		PrintlnSerial_string("defaults loaded from eeprom", EndPoint_All);
	} else if (strncmp("2015.01.01", defaultsettings.version, sizeof(activesettings.version)) == 0) {
		memcpy(&activesettings, &defaultsettings, sizeof(defaultsettings));
		activesettings.rs_speed_signal_max_acceleration = 2.0f;
		activesettings.tolerated_positional_error_limit = 1.0f;
		activesettings.tolerated_positional_error_exceeded = 500.0f;
		activesettings.auxvalue = 1500;
		PrintlnSerial_string("defaults loaded from eeprom using version 2015.01.01", EndPoint_All);
	} else {
		PrintlnSerial_string("eeprom does not contain valid default - keeping the system defaults", EndPoint_All);
	}
	setServoValueForChannel(SERVO_AUX, activesettings.auxvalue);

	printHelp(EndPoint_All);

	while (1) {
		if(Systick_Timer1(TIMER_CHECK,0)==TIMER_HOLD) { // every 20ms --> 50Hz
			Systick_Timer1(TIMER_START_ms, 20); // reset tick to 20ms

			// watchout, the duty cycles below can be read only once per 20ms as they are reset to zero after read, for recognizing disconnects of the RC signal cable!!!
			setRCData(ICPWM_ReadDUTY(activesettings.yaw_rc_channel), ICPWM_ReadDUTY(activesettings.pitch_rc_channel), ICPWM_ReadDUTY(activesettings.aux_rc_channel), ICPWM_ReadDUTY(activesettings.speed_rc_channel)); // set the rcData[] array values, the placeholder for the current input duty cycles

			controllercycle();
			if (is1Hz()) {
				printDebug_string("rcInput: ", DEBUG_LEVEL_VERYLOW);
				printDebug_int(ICPWM_ReadDUTY(0), DEBUG_LEVEL_VERYLOW);
				printDebug_int(ICPWM_ReadDUTY(1), DEBUG_LEVEL_VERYLOW);
				printDebug_int(ICPWM_ReadDUTY(2), DEBUG_LEVEL_VERYLOW);
				printDebug_int(ICPWM_ReadDUTY(3), DEBUG_LEVEL_VERYLOW);
				printDebug_int(ICPWM_ReadDUTY(4), DEBUG_LEVEL_VERYLOW);
				printDebug_int(ICPWM_ReadDUTY(5), DEBUG_LEVEL_VERYLOW);
				printDebug_int(ICPWM_ReadDUTY(6), DEBUG_LEVEL_VERYLOW);
				printDebug_int(ICPWM_ReadDUTY(7), DEBUG_LEVEL_VERYLOW);
				printDebug_int(ICPWM_ReadDUTY(8), DEBUG_LEVEL_VERYLOW);
				printDebug_int(ICPWM_ReadDUTY(9), DEBUG_LEVEL_VERYLOW);
				printDebug_int(ICPWM_ReadDUTY(10), DEBUG_LEVEL_VERYLOW);
				printDebug_string("   currentpos: ", DEBUG_LEVEL_VERYLOW);
				printlnDebug_int(ENCODER_ReadPos(), DEBUG_LEVEL_VERYLOW);
			}
			if (bootphase == 0 && getCounter() >= 1000) {

				/* Disable the Serial Wire Jtag Debug Port SWJ-DP */
				__HAL_AFIO_REMAP_SWJ_NONJTRST();
				__HAL_AFIO_REMAP_SWJ_DISABLE();

				__GPIOA_CLK_ENABLE();
				GPIO_InitStructure.Pin = GPIO_PIN_3;
				GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
				GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
				HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
				bootphase = 1;
			} else {
				if (is5Hz()) {
					HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);
				}
			}
		}

		if( Uart_ReceiveString(buf, APP_TX_BUF_SIZE) > 0 ) {
			serialCom(buf, EndPoint_UART3);
		}
		if( USB_ReceiveString(buf, APP_TX_BUF_SIZE) > 0 ) {
			serialCom(buf, EndPoint_USB);
		}

	}

}


/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 72000000
 *            HCLK(Hz)                       = 72000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 2
 *            APB2 Prescaler                 = 1
 *            HSE Frequency(Hz)              = 8000000
 *            HSE PREDIV1                    = 1
 *            PLLMUL                         = 9
 *            Flash Latency(WS)              = 2
 * @param  None
 * @retval None
 */
void SystemClock_Config(void)
{
	RCC_ClkInitTypeDef clkinitstruct = {0};
	RCC_OscInitTypeDef oscinitstruct = {0};
	RCC_PeriphCLKInitTypeDef rccperiphclkinit = {0};

	/* Enable HSE Oscillator and activate PLL with HSE as source */
	oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
	oscinitstruct.HSEState        = RCC_HSE_ON;
	oscinitstruct.HSEPredivValue  = RCC_HSE_PREDIV_DIV1;
	oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL9;

	oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
	oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;

	if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
	{
		Error_Handler();
	}

	/* USB clock selection */
	rccperiphclkinit.PeriphClockSelection = RCC_PERIPHCLK_USB;
	rccperiphclkinit.UsbClockSelection = RCC_USBPLLCLK_DIV1_5;
	HAL_RCCEx_PeriphCLKConfig(&rccperiphclkinit);

	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
	clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;
	clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
	if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void)
{
	hardstop();
	while (1)
	{
	}
}

void hardstop() {
	PWM_SetPWM(SERVO_1_OUT, 0);
	PWM_SetPWM(SERVO_2_OUT, 0);
	PWM_SetPWM(SERVO_3_OUT, 0);
	PWM_SetPWM(SERVO_5_OUT, 0);
}


