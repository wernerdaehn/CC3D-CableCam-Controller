#ifndef EEPROM_H_
#define EEPROM_H_


#include "stm32f4xx_hal.h"


#define EEPROM_SECTOR_FOR_SETTINGS 0


uint8_t eeprom_init(void);
uint32_t eeprom_write_sector_safe(uint8_t *write, uint16_t size, uint16_t sector);
void eeprom_append_unverified(uint8_t *write, uint32_t size, uint16_t start_sector);
void eeprom_read_from_address(uint8_t *read, uint32_t size, uint32_t address);
void eeprom_read_sector(uint8_t *read, uint32_t size, uint16_t start_sector);

#endif
