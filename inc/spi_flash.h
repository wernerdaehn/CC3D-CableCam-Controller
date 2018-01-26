/**
  ******************************************************************************
  * @file    stm32_eval_spi_flash.h
  * @author  MCD Application Team
  * @version V4.5.0
  * @date    07-March-2011
  * @brief   This file contains all the functions prototypes for the stm32_eval_spi_flash
  *          firmware driver.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_FLASH_H
#define __SPI_FLASH_H

#ifdef __cplusplus
 extern "C" {
#endif

#include "stm32f4xx_hal.h"

        /**SPI3 GPIO Configuration
        PA15     ------> SPI3_NSS
        PC10     ------> SPI3_SCK
        PC11     ------> SPI3_MISO
        PC12     ------> SPI3_MOSI
        */
#define sFLASH_SPI                       SPI3
#define sFLASH_SPI_SCK_PIN               GPIO_PIN_10
#define sFLASH_SPI_SCK_GPIO_PORT         GPIOC
#define sFLASH_SPI_SCK_GPIO_CLK          __GPIOC_CLK_ENABLE
#define sFLASH_SPI_MISO_PIN              GPIO_PIN_11
#define sFLASH_SPI_MISO_GPIO_PORT        GPIOC
#define sFLASH_SPI_MISO_GPIO_CLK         __GPIOC_CLK_ENABLE
#define sFLASH_SPI_MOSI_PIN              GPIO_PIN_12
#define sFLASH_SPI_MOSI_GPIO_PORT        GPIOC
#define sFLASH_SPI_MOSI_GPIO_CLK         __GPIOC_CLK_ENABLE
#define sFLASH_CS_PIN                    GPIO_PIN_3
#define sFLASH_CS_GPIO_PORT              GPIOB
#define sFLASH_CS_GPIO_CLK               __GPIOB_CLK_ENABLE




/**
  * @brief  M25P SPI Flash supported commands
  */
#define sFLASH_CMD_WRITE          0x02  /*!< Write to Memory instruction */
#define sFLASH_CMD_WRSR           0x01  /*!< Write Status Register instruction */
#define sFLASH_CMD_WREN           0x06  /*!< Write enable instruction */
#define sFLASH_CMD_READ           0x03  /*!< Read from Memory instruction */
#define sFLASH_CMD_RDSR           0x05  /*!< Read Status Register instruction  */
#define sFLASH_CMD_RDID           0x9F  /*!< Read identification */
#define sFLASH_CMD_SE             0xD8  /*!< Sector Erase instruction */
#define sFLASH_CMD_BE             0xC7  /*!< Bulk Erase instruction */

#define sFLASH_WIP_FLAG           0x01  /*!< Write In Progress (WIP) flag */

#define sFLASH_DUMMY_BYTE         0xA5
#define sFLASH_SPI_PAGESIZE       0x100

#define sFLASH_M25P128_ID         0x202018
#define sFLASH_M25P64_ID          0x202017
#define sFLASH_M25P16_ID          0x202015



/** @defgroup STM32_EVAL_SPI_FLASH_Exported_Macros
  * @{
  */
/**
  * @brief  Select sFLASH: Chip Select pin low
  */
#define sFLASH_CS_LOW()       HAL_GPIO_WritePin(sFLASH_CS_GPIO_PORT, sFLASH_CS_PIN, GPIO_PIN_RESET)
/**
  * @brief  Deselect sFLASH: Chip Select pin high
  */
#define sFLASH_CS_HIGH()      HAL_GPIO_WritePin(sFLASH_CS_GPIO_PORT, sFLASH_CS_PIN, GPIO_PIN_SET)
/**
  * @}
  */


void sFLASH_LowLevel_Init(void);

/**
  * @brief  High layer functions
  */
void sFLASH_EraseSector(uint32_t SectorAddr);
void sFLASH_EraseBulk(void);
void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite);
void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite);
void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead);
uint32_t sFLASH_ReadID(void);
uint32_t sFLASH_VerifyWrite(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead);
/**
  * @brief  Low layer functions
  */
void sFLASH_WriteEnable(void);
void sFLASH_WaitForWriteEnd(void);


#ifdef __cplusplus
}
#endif

#endif /* __SPI_FLASH_H */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
