#include "systembootloader.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "usbd_def.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "main.h"

/**
 * Function to perform jump to system memory boot from user application
 *
 * Call function when you want to jump to system memory
 */
void JumpToBootloader(void) {
    *((unsigned long *)0x2001FFF0) = 0xDEADBEEF; // End of RAM

    NVIC_SystemReset();
}



