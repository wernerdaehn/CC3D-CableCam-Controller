#include "eeprom.h"
#include "spi_flash.h"


/* Private define ------------------------------------------------------------*/

#define  sFLASH_ID       sFLASH_M25P16_ID


/* Private variables ---------------------------------------------------------*/
uint32_t FlashID = 0;

/* Private functions ---------------------------------------------------------*/

uint8_t eeprom_init() {
	/* Get SPI Flash ID */
	FlashID = sFLASH_ReadID();

	/* Check the SPI Flash ID */
	if (FlashID != sFLASH_ID) {
		FlashID = 1;
		return 2;
	}
	return 0;
}


/**
 * @brief  writes a data block into a fixed sector and verifies the data correctness. Used for saving settings.
 * Note that this function will erase the entire 65k big sector, hence do not place any other data there
 * @param  write: buffer address
 * @param  size: amount of bytes to be written (less than the sector size which is 65k)
 * @param  sector: the sector number
 * @retval None
 */
uint32_t eeprom_write_sector_safe(uint8_t *write, uint16_t size, uint16_t sector) {
	if (FlashID < 10) {
		// Not initialized or wrong SPI ID
		return 2;
	}
	uint32_t sectoraddress = ((uint32_t) sector) * 0x00010000;
	/* Perform a write in the Flash followed by a read of the written data */
	/* Erase SPI FLASH Sector to write on */
	sFLASH_EraseSector(sectoraddress);

	/* Write Tx_Buffer data to SPI FLASH memory */
	sFLASH_WriteBuffer(write, sectoraddress, size);

	/* Read data from SPI FLASH memory */
	return sFLASH_VerifyWrite(write, sectoraddress, size);
}

static uint32_t last_address = 0;

void eeprom_append_unverified(uint8_t *write, uint32_t size, uint16_t start_sector) {
	/*
	 * Example1:
	 * start_sector = 1 ---> start address is 0x00010000
	 * last_address = 0
	 * size = 0x0100
	 * start_address = 0x00010000 + 0x00000000
	 * sectoraddress_to_erase = (0x00010000 + 0x0000FFFF) & 0xFFFF0000 = 0x0001FFFF  & 0xFFFF0000 = 0x00010000
	 * while (0x00010000 < 0x00010100) -> erase 0x00010000
	 *
	 * Example2:
	 * start_sector = 1 ---> start address is 0x00010000
	 * last_address = 0xFF
	 * size = 0x0100
	 * start_address = 0x00010000 + 0x000000FF = 0x000100FF
	 * sectoraddress_to_erase = (0x000100FF + 0x0000FFFF) & 0xFFFF0000 = 0x0002FFFE & 0xFFFF0000 = 0x00020000
	 * while (0x00020000 < 0x00010100) -> nothing
	 *
	 */
	uint32_t start_address = ((uint32_t) start_sector) * 0xFFFF + last_address;
	uint32_t sectoraddress_to_erase = (start_address + 0x0000FFFF) & 0xFFFF0000;
	while (sectoraddress_to_erase < start_address+size) {
		sFLASH_EraseSector(sectoraddress_to_erase);
		sectoraddress_to_erase += 0x00010000;
	}
	sFLASH_WriteBuffer(write, start_address, size);
	last_address += size;
}

void eeprom_read_from_address(uint8_t *read, uint32_t size, uint32_t address) {
	sFLASH_ReadBuffer(read, address, size );
}

void eeprom_read_sector(uint8_t *read, uint32_t size, uint16_t start_sector) {
	sFLASH_ReadBuffer(read, ((uint32_t) start_sector) * 0x00010000, size );
}

