#include "eeprom.h"
#include "spi_flash.h"
#include "serial_print.h"


/* Private define ------------------------------------------------------------*/

#define  sFLASH_ID       sFLASH_M25P16_ID


/* Private variables ---------------------------------------------------------*/
uint32_t FlashID = 0;

/* Private functions ---------------------------------------------------------*/

uint8_t eeprom_init()
{
    /* Get SPI Flash ID */
    FlashID = sFLASH_ReadID();

    /* Check the SPI Flash ID */
    if (FlashID != sFLASH_ID)
    {
        return 0;
    }
    return 0;
}


/**
 * @brief  writes a data block into a fixed sector and verifies the data correctness. Used for saving settings.
 * Note that this function will erase the entire 65k big sector, hence do not place any other data there
 * @param  write: buffer address
 * @param  len: amount of bytes to be written (less than the sector size which is 65k)
 * @param  sector: the sector number
 * @retval None
 */
uint32_t eeprom_write_sector_safe(uint8_t *write, uint16_t len, uint16_t sector)
{
    if (FlashID < 10)
    {
        // Not initialized or wrong SPI ID
        return 2;
    }
    uint32_t sectoraddress = ((uint32_t) sector) * 0x00010000;
    /* Perform a write in the Flash followed by a read of the written data */
    /* Erase SPI FLASH Sector to write on */
    sFLASH_EraseSector(sectoraddress);

    /* Write Tx_Buffer data to SPI FLASH memory */
    sFLASH_WriteBuffer(write, sectoraddress, len);

    /* Read data from SPI FLASH memory */
    return sFLASH_VerifyWrite(write, sectoraddress, len);
}

uint32_t eeprom_erase(uint16_t sector)
{
    if (FlashID < 10)
    {
        // Not initialized or wrong SPI ID
        return 2;
    }
    uint32_t sectoraddress = ((uint32_t) sector) * 0x00010000;
    /* Perform a write in the Flash followed by a read of the written data */
    /* Erase SPI FLASH Sector to write on */
    return sFLASH_EraseSector(sectoraddress);
}




uint32_t eeprom_read_from_address(uint8_t *read, uint32_t len, uint32_t address)
{
    return sFLASH_ReadBuffer(read, address, len );
}

uint32_t eeprom_read_sector(uint8_t *read, uint32_t len, uint16_t start_sector)
{
    return sFLASH_ReadBuffer(read, ((uint32_t) start_sector) * 0x00010000, len );
}


static uint16_t eeprom_cycle_state = 0;
static uint8_t *eeprom_cycle_write;
static uint16_t eeprom_cycle_len;
static uint32_t eeprom_cycle_sectoraddress;
static uint32_t eeprom_cycle_timeout;

uint32_t eeprom_write_sector_async(uint8_t *write, uint16_t len, uint16_t sector)
{
    if (eeprom_cycle_state == 0)
    {
        eeprom_cycle_state++;
        eeprom_cycle_write = write;
        eeprom_cycle_len = len;
        eeprom_cycle_sectoraddress = ((uint32_t) sector) * 0x00010000;
        eeprom_cycle_timeout = HAL_GetTick() + 10000L;
        return 0;
    }
    else
    {
        return 1;
    }
}

void eeprom_cycle()
{
    if (eeprom_cycle_state != 0)
    {
        uint32_t errors = 0;
        switch (eeprom_cycle_state)
        {
        case 1:
            errors = sFLASH_EraseSector_nowait(eeprom_cycle_sectoraddress);
            eeprom_cycle_state++;
            break;
        case 2:
        case 4:
            if (sFLASH_CheckForWriteEnd() == 0)
            {
                eeprom_cycle_state++;
            }
            break;
        case 3:
            errors = sFLASH_WriteBuffer_nowait(eeprom_cycle_write, eeprom_cycle_sectoraddress, eeprom_cycle_len);
            eeprom_cycle_state++;
            break;
        case 5:
            PrintlnSerial_string("EEPROM Write successful", EndPoint_All);
            eeprom_cycle_state = 0;
            break;
        default:
            eeprom_cycle_state = 0;
        }
        if (errors != 0 || HAL_GetTick() > eeprom_cycle_timeout)
        {
            eeprom_cycle_state = 0;
            PrintlnSerial_string("EEPROM Write failed", EndPoint_All);
        }
    }
}
