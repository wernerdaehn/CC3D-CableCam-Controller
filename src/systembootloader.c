#include "systembootloader.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include "usbd_def.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "main.h"

#define SYSMEM_ADDRESS  (uint32_t)0x1FFF0000    /* Address of System Memory (ST Bootloader) */

extern USBD_HandleTypeDef hUsbDeviceFS;

/**
 * Function to perform jump to system memory boot from user application
 *
 * Call function when you want to jump to system memory
 */
void JumpToBootloader(void) {
    GPIO_InitTypeDef GPIO_InitStruct;

    void (*SysMemBootJump)(void);

    LED_STATUS_ON;
    HAL_Delay(5000); // Wait for 5 seconds so that it looks to Windows as if the USB cable has been unplugged.

//    HAL_PCD_DevDisconnect( hUsbDeviceFS.pData );

    USBD_Stop(&hUsbDeviceFS);

    LED_STATUS_OFF;
    HAL_Delay(5000); // Wait for 5 seconds so that it looks to Windows as if the USB cable has been unplugged.

    USBD_DeInit(&hUsbDeviceFS);

    __HAL_RCC_USB_OTG_FS_FORCE_RESET();

    LED_STATUS_ON;
    HAL_Delay(5000); // Wait for 5 seconds so that it looks to Windows as if the USB cable has been unplugged.

    __HAL_RCC_USB_OTG_FS_RELEASE_RESET();

    /**USB_OTG_FS GPIO Configuration
    PA11     ------> USB_OTG_FS_DM
    PA12     ------> USB_OTG_FS_DP
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

    HAL_Delay(5000); // Wait for 5 seconds so that it looks to Windows as if the USB cable has been unplugged.


    /**
     * Step: Set system memory address.
     */
    volatile uint32_t addr = SYSMEM_ADDRESS;

    /**
     * Step: Disable RCC, set it to default (after reset) settings
     *       Internal clock, no PLL, etc.
     */
    HAL_RCC_DeInit();
    HAL_DeInit();

    /**
     * Step: Disable systick timer and reset it to default values
     */
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    /**
     * Step: Disable all interrupts
     */
    __disable_irq();

    /**
     * Step: Remap system memory to address 0x0000 0000 in address space
     */
    __HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

    /**
     * Step: Set jump memory location for system memory
     *       Use address with 4 bytes offset which specifies jump location where program starts
     */
    SysMemBootJump = (void (*)(void)) (*((uint32_t *)(addr + 4)));

    /**
     * Step: Set main stack pointer.
     *       This step must be done last otherwise local variables in this function
     *       don't have proper value since stack pointer is located on different position
     *
     *       Set direct address location which specifies stack pointer in SRAM location
     */
    __set_MSP(*(uint32_t *)addr);

    /**
     * Step: Actually call our function to jump to set location
     *       This will start system memory execution
     */
    SysMemBootJump();

    while(1);
}



