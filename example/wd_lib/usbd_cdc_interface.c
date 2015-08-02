/**
 ******************************************************************************
 * @file    USB_Device/CDC_Standalone/Src/usbd_cdc_interface.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    17-December-2014
 * @brief   Source file for USBD CDC interface
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32_uart.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
 * @{
 */

/** @defgroup USBD_CDC 
 * @brief usbd core module
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

#define RXBUFFERSIZE	80
#define APP_RX_DATA_SIZE  2048
#define APP_TX_DATA_SIZE  2048

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
USBD_CDC_LineCodingTypeDef LineCoding =
{
		115200, /* baud rate*/
		0x00,   /* stop bits-1*/
		0x00,   /* parity - none*/
		0x08    /* nb. of bits 8*/
};

uint8_t rxbuffer[RXBUFFERSIZE];
uint32_t bytes_received = 0;
uint32_t bytes_scanned = 0;
uint32_t last_string_start_pos = 0;
uint8_t rxbuffer_overflow = 0;

uint8_t UserRxBuffer[APP_RX_DATA_SIZE];/* Received Data over USB are stored in this buffer */
uint8_t UserTxBuffer[APP_TX_DATA_SIZE];/* Received Data over UART (CDC interface) are stored in this buffer */
uint32_t BuffLength;

uint32_t bytes_sent = 0;
uint32_t bytes_written = 0;


/* USB handler declaration */
extern USBD_HandleTypeDef  USBD_Device;

/* Private function prototypes -----------------------------------------------*/
static int8_t CDC_Itf_Init     (void);
static int8_t CDC_Itf_DeInit   (void);
static int8_t CDC_Itf_Control  (uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Itf_Receive  (uint8_t* pbuf, uint32_t *Len);

USBD_CDC_ItfTypeDef USBD_CDC_fops = 
{
		CDC_Itf_Init,
		CDC_Itf_DeInit,
		CDC_Itf_Control,
		CDC_Itf_Receive
};

/* Private functions ---------------------------------------------------------*/

/**
 * @brief  CDC_Itf_Init
 *         Initializes the CDC media low layer
 * @param  None
 * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Itf_Init(void)
{
	USBD_CDC_SetTxBuffer(&USBD_Device, UserTxBuffer, 0);
	USBD_CDC_SetRxBuffer(&USBD_Device, UserRxBuffer);
	return (USBD_OK);
}

/**
 * @brief  CDC_Itf_DeInit
 *         DeInitializes the CDC media low layer
 * @param  None
 * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Itf_DeInit(void)
{
	return (USBD_OK);
}

/**
 * @brief  CDC_Itf_Control
 *         Manage the CDC class requests
 * @param  Cmd: Command code
 * @param  Buf: Buffer containing command data (request parameters)
 * @param  Len: Number of data to be sent (in bytes)
 * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Itf_Control (uint8_t cmd, uint8_t* pbuf, uint16_t length)
{ 
	switch (cmd)
	{
	case CDC_SEND_ENCAPSULATED_COMMAND:
		/* Add your code here */
		break;

	case CDC_GET_ENCAPSULATED_RESPONSE:
		/* Add your code here */
		break;

	case CDC_SET_COMM_FEATURE:
		/* Add your code here */
		break;

	case CDC_GET_COMM_FEATURE:
		/* Add your code here */
		break;

	case CDC_CLEAR_COMM_FEATURE:
		/* Add your code here */
		break;

	case CDC_SET_LINE_CODING:
		LineCoding.bitrate    = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |\
				(pbuf[2] << 16) | (pbuf[3] << 24));
		LineCoding.format     = pbuf[4];
		LineCoding.paritytype = pbuf[5];
		LineCoding.datatype   = pbuf[6];

		/* Set the new configuration */
		break;

	case CDC_GET_LINE_CODING:
		pbuf[0] = (uint8_t)(LineCoding.bitrate);
		pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
		pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
		pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
		pbuf[4] = LineCoding.format;
		pbuf[5] = LineCoding.paritytype;
		pbuf[6] = LineCoding.datatype;

		/* Add your code here */
		break;

	case CDC_SET_CONTROL_LINE_STATE:
		/* Add your code here */
		break;

	case CDC_SEND_BREAK:
		/* Add your code here */
		break;

	default:
		break;
	}

	return (USBD_OK);
}

/**
 * @brief  CDC_Itf_DataRx
 *         Data received over USB OUT endpoint are sent over CDC interface
 *         through this function.
 * @param  Buf: Buffer of data to be transmitted
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the opeartion: USBD_OK if all operations are OK else USBD_FAIL
 */
static int8_t CDC_Itf_Receive(uint8_t* Buf, uint32_t *Len)
{
	uint32_t len1 = *Len;
	uint16_t current_pos;

	if (bytes_received + len1 > last_string_start_pos + RXBUFFERSIZE) {
		if (bytes_received > last_string_start_pos + RXBUFFERSIZE) {
			// overflow condition
			rxbuffer_overflow = 1;
			return USBD_OK;
		} else {
			len1 = last_string_start_pos + RXBUFFERSIZE - bytes_received;
		}
	}
	current_pos = bytes_received % RXBUFFERSIZE;
	if (len1 > RXBUFFERSIZE - current_pos) {
		memcpy(&rxbuffer[current_pos], Buf, RXBUFFERSIZE - current_pos);
		memcpy(&rxbuffer[0], &Buf[RXBUFFERSIZE - current_pos], len1 + current_pos - RXBUFFERSIZE);
	} else {
		memcpy(&rxbuffer[current_pos], Buf, len1);
	}
	bytes_received += len1;
	USBD_CDC_ReceivePacket(&USBD_Device);
	return (USBD_OK);
}

uint16_t USB_ReceiveString(char *ptr, uint16_t maxsize) {
	size_t len;
	uint16_t rel_string_start_pos;

	while (bytes_scanned < bytes_received) {
		CDC_TransmitBuffer((uint8_t *) &rxbuffer[bytes_scanned % RXBUFFERSIZE], 1); // echo input text
		if (rxbuffer[bytes_scanned % RXBUFFERSIZE] == '\n') {
			bytes_scanned++;
			len = bytes_scanned - last_string_start_pos;
			if (len <= maxsize) { // in case the string does not fit into the string buffer, it is ignored
				rel_string_start_pos = last_string_start_pos % RXBUFFERSIZE;
				if (rel_string_start_pos + len <= RXBUFFERSIZE) {
					memcpy(ptr, &rxbuffer[rel_string_start_pos], len);
				} else {
					memcpy(ptr, &rxbuffer[rel_string_start_pos], RXBUFFERSIZE - rel_string_start_pos);
					memcpy(&ptr[rel_string_start_pos], &rxbuffer[0], len + rel_string_start_pos - RXBUFFERSIZE);
				}
				last_string_start_pos = bytes_scanned; // next string starts here
				return len;
			} else {
				last_string_start_pos = bytes_scanned; // next string starts here
			}
		} else {
			bytes_scanned++;
		}
	}
	// No newline char was found
	if (rxbuffer_overflow == 1) {
		bytes_scanned = bytes_received;
		last_string_start_pos = bytes_received;
		rxbuffer_overflow = 0;
	}
	return 0;
}

uint8_t  CDC_TransmitBuffer(uint8_t *ptr, uint32_t len) {
	uint32_t l;
	uint32_t rel_pos = bytes_written % APP_TX_DATA_SIZE;
	uint32_t tick = HAL_GetTick();
	if (USBD_Device.dev_state == USBD_STATE_CONFIGURED && len < APP_RX_DATA_SIZE) {

		while (bytes_written + len >= bytes_sent + APP_RX_DATA_SIZE) {
			/*
			 * wait for the USB bus to send the data
			 * The USBPeriodElapsed() method is called in 20ms intervals by the systick interrupt
			 * As the USBPeriodElapsed() counts the bytes_sent upwards even if there is a problem, the condition should be true
			 * after a view ms. But adding another exit condition just to be sure.
			 */
			if (HAL_GetTick() - tick > 1000) {
				return USBD_FAIL;
			}
		}
		if (rel_pos + len > APP_TX_DATA_SIZE) {
			l = APP_TX_DATA_SIZE - rel_pos;
			memcpy(&UserTxBuffer[rel_pos], ptr, l);
			memcpy(UserTxBuffer, &ptr[l], len - l);
		} else {
			memcpy(&UserTxBuffer[rel_pos], ptr, len);
		}
		bytes_written += len;
	}
	return USBD_OK;
}

uint8_t  CDC_TransmitString(char *ptr) {
	return CDC_TransmitBuffer((uint8_t *)ptr, strlen(ptr));
}


/**
 * @brief  TIM period elapsed callback
 * @param  htim: TIM handle
 * @retval None
 */
void USBPeriodElapsed()
{
	uint32_t buffptr = bytes_sent % APP_TX_DATA_SIZE;
	uint32_t buffsize = bytes_written - bytes_sent;

	if (USBD_Device.dev_state == USBD_STATE_CONFIGURED) {
		if(bytes_written != bytes_sent)
		{
			if (buffptr + buffsize > APP_TX_DATA_SIZE) {
				buffsize = APP_TX_DATA_SIZE - buffptr;
			}
			USBD_CDC_SetTxBuffer(&USBD_Device, (uint8_t*)&UserTxBuffer[buffptr], buffsize);

			if(USBD_CDC_TransmitPacket(&USBD_Device) == USBD_OK)
			{
			}
		}
	}
	bytes_sent += buffsize;
}


/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

