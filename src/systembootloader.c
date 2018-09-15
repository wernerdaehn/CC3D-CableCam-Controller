#include "systembootloader.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "usbd_def.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "main.h"
#include "usb_device.h"

/**
 * Function to perform jump to system memory boot from user application
 *
 * Call function when you want to jump to system memory
 */
void JumpToBootloader(void) {
    *((unsigned long *)0x2001FFF0) = 0xDEADBEEF; // End of RAM

    USBD_Stop(&hUsbDeviceFS);
    USBD_DeInit(&hUsbDeviceFS);
    HAL_Delay(5000); // Wait for 5 seconds so that it looks to Windows as if the USB cable has been unplugged.

    NVIC_SystemReset();
}



