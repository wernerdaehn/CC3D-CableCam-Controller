#ifndef EEPROM_H_
#define EEPROM_H_


#include "stm32f4xx_hal.h"


#define EEPROM_SECTOR_FOR_SETTINGS 0
#define EEPROM_SECTOR_FOR_SEMIPERMANENTSETTINGS 3

uint8_t eeprom_init(void);
uint32_t eeprom_write_sector_safe(uint8_t *write, uint16_t len, uint16_t sector);
uint32_t eeprom_read_from_address(uint8_t *read, uint32_t len, uint32_t address);
uint32_t eeprom_read_sector(uint8_t *read, uint32_t len, uint16_t start_sector);
uint32_t eeprom_write_sector_async(uint8_t *write, uint16_t len, uint16_t sector);
uint32_t eeprom_erase(uint16_t sector);

void eeprom_cycle(void);
#endif
