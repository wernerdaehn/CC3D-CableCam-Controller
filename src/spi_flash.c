/**
 ******************************************************************************
 * @file    stm32_eval_spi_flash.c
 * @author  MCD Application Team
 * @version V4.5.0
 * @date    07-March-2011
 * @brief   This file provides a set of functions needed to manage the SPI M25Pxxx
 *          FLASH memory mounted on STM32xx-EVAL board (refer to stm32_eval.h
 *          to know about the boards supporting this memory).
 *          It implements a high level communication layer for read and write
 *          from/to this memory. The needed STM32 hardware resources (SPI and
 *          GPIO) are defined in stm32xx_eval.h file, and the initialization is
 *          performed in sFLASH_LowLevel_Init() function declared in stm32xx_eval.c
 *          file.
 *          You can easily tailor this driver to any other development board,
 *          by just adapting the defines for hardware resources and
 *          sFLASH_LowLevel_Init() function.
 *
 *          +-----------------------------------------------------------+
 *          |                     Pin assignment                        |
 *          +-----------------------------+---------------+-------------+
 *          |  STM32 SPI Pins             |     sFLASH    |    Pin      |
 *          +-----------------------------+---------------+-------------+
 *          | sFLASH_CS_PIN               | ChipSelect(/S)|    1        |
 *          | sFLASH_SPI_MISO_PIN / MISO  |   DataOut(Q)  |    2        |
 *          |                             |   VCC         |    3 (3.3 V)|
 *          |                             |   GND         |    4 (0 V)  |
 *          | sFLASH_SPI_MOSI_PIN / MOSI  |   DataIn(D)   |    5        |
 *          | sFLASH_SPI_SCK_PIN / SCLK   |   Clock(C)    |    6        |
 *          |                             |    VCC        |    7 (3.3 V)|
 *          |                             |    VCC        |    8 (3.3 V)|
 *          +-----------------------------+---------------+-------------+
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

/* Includes ------------------------------------------------------------------*/
#include "spi_flash.h"
#include "main.h"

extern SPI_HandleTypeDef hspi3;

void sFLASH_StartReadSequence(uint32_t ReadAddr);

uint8_t command[4];


/**
 * @brief  Erases the specified FLASH sector.
 * @param  SectorAddr: address of the sector to erase.
 * @retval None
 */
void sFLASH_EraseSector(uint32_t SectorAddr)
{
    /*!< Send write enable instruction */
    sFLASH_WriteEnable();

    command[0] = sFLASH_CMD_SE;
    command[1] = (SectorAddr & 0xFF0000) >> 16;
    command[2] = (SectorAddr & 0xFF00) >> 8;
    command[3] = SectorAddr & 0xFF;

    /*!< Sector Erase */
    /*!< Select the FLASH: Chip Select low */
    sFLASH_CS_LOW();
    /*!< Send Sector Erase instruction */
    switch(HAL_SPI_Transmit(&hspi3, &command[0], 4, 5000))
    {
    case HAL_TIMEOUT:
        /* A Timeout Occur ______________________________________________________*/
        /* Call Timeout Handler */
        return;
        break;
        /* An Error Occur ______________________________________________________ */
    case HAL_ERROR:
        /* Call Timeout Handler */
        return;
        break;
    default:
        break;
    }
    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH();

    /*!< Wait the end of Flash writing */
    sFLASH_WaitForWriteEnd();
}

/**
 * @brief  Erases the entire FLASH.
 * @param  None
 * @retval None
 */
void sFLASH_EraseBulk(void)
{
    /*!< Send write enable instruction */
    sFLASH_WriteEnable();

    /*!< Bulk Erase */
    /*!< Select the FLASH: Chip Select low */
    sFLASH_CS_LOW();
    /*!< Send Bulk Erase instruction  */
    command[0] = sFLASH_CMD_BE;
    switch(HAL_SPI_Transmit(&hspi3, &command[0], 1, 5000))
    {
    case HAL_TIMEOUT:
        /* A Timeout Occur ______________________________________________________*/
        /* Call Timeout Handler */
        return;
        break;
        /* An Error Occur ______________________________________________________ */
    case HAL_ERROR:
        /* Call Timeout Handler */
        return;
        break;
    default:
        break;
    }

    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH();

    /*!< Wait the end of Flash writing */
    sFLASH_WaitForWriteEnd();
}

/**
 * @brief  Writes more than one byte to the FLASH with a single WRITE cycle
 *         (Page WRITE sequence).
 * @note   The number of byte can't exceed the FLASH page size.
 * @param  pBuffer: pointer to the buffer  containing the data to be written
 *         to the FLASH.
 * @param  WriteAddr: FLASH's internal address to write to.
 * @param  NumByteToWrite: number of bytes to write to the FLASH, must be equal
 *         or less than "sFLASH_PAGESIZE" value.
 * @retval None
 */
void sFLASH_WritePage(uint8_t* pBuffer, uint32_t WriteAddr,	uint16_t NumByteToWrite)
{

    /*!< Enable the write access to the FLASH */
    sFLASH_WriteEnable();

    /*!< Send "Write to Memory " instruction */
    command[0] = sFLASH_CMD_WRITE;
    command[1] = (WriteAddr & 0xFF0000) >> 16;
    command[2] = (WriteAddr & 0xFF00) >> 8;
    command[3] = WriteAddr & 0xFF;

    /*!< Select the FLASH: Chip Select low */
    sFLASH_CS_LOW();
    /*!< Send Sector Erase instruction */
    switch(HAL_SPI_Transmit(&hspi3, &command[0], 4, 5000))
    {
    case HAL_TIMEOUT:
        /* A Timeout Occur ______________________________________________________*/
        /* Call Timeout Handler */
        return;
        break;
        /* An Error Occur ______________________________________________________ */
    case HAL_ERROR:
        /* Call Timeout Handler */
        return;
        break;
    default:
        break;
    }

    /*!< while there is data to be written on the FLASH */
    switch(HAL_SPI_Transmit(&hspi3, pBuffer, NumByteToWrite, 5000))
    {
    case HAL_TIMEOUT:
        /* A Timeout Occur ______________________________________________________*/
        /* Call Timeout Handler */
        return;
        break;
        /* An Error Occur ______________________________________________________ */
    case HAL_ERROR:
        /* Call Timeout Handler */
        return;
        break;
    default:
        break;
    }


    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH();

    /*!< Wait the end of Flash writing */
    sFLASH_WaitForWriteEnd();
}

/**
 * @brief  Writes block of data to the FLASH. In this function, the number of
 *         WRITE cycles are reduced, using Page WRITE sequence.
 * @param  pBuffer: pointer to the buffer  containing the data to be written
 *         to the FLASH.
 * @param  WriteAddr: FLASH's internal address to write to.
 * @param  NumByteToWrite: number of bytes to write to the FLASH.
 * @retval None
 */
void sFLASH_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint32_t NumByteToWrite)
{
    uint32_t NumOfPage = 0, NumOfSingle = 0, Addr = 0, count = 0, temp = 0;

    Addr = WriteAddr % sFLASH_SPI_PAGESIZE;
    count = sFLASH_SPI_PAGESIZE - Addr;
    NumOfPage = NumByteToWrite / sFLASH_SPI_PAGESIZE;
    NumOfSingle = NumByteToWrite % sFLASH_SPI_PAGESIZE;

    if (Addr == 0) /*!< WriteAddr is sFLASH_PAGESIZE aligned  */
    {
        if (NumOfPage == 0) /*!< NumByteToWrite < sFLASH_PAGESIZE */
        {
            sFLASH_WritePage(pBuffer, WriteAddr, NumByteToWrite);
        }
        else   /*!< NumByteToWrite > sFLASH_PAGESIZE */
        {
            while (NumOfPage--)
            {
                sFLASH_WritePage(pBuffer, WriteAddr, sFLASH_SPI_PAGESIZE);
                WriteAddr += sFLASH_SPI_PAGESIZE;
                pBuffer += sFLASH_SPI_PAGESIZE;
            }

            sFLASH_WritePage(pBuffer, WriteAddr, NumOfSingle);
        }
    }
    else   /*!< WriteAddr is not sFLASH_PAGESIZE aligned  */
    {
        if (NumOfPage == 0) /*!< NumByteToWrite < sFLASH_PAGESIZE */
        {
            if (NumOfSingle > count) /*!< (NumByteToWrite + WriteAddr) > sFLASH_PAGESIZE */
            {
                temp = NumOfSingle - count;

                sFLASH_WritePage(pBuffer, WriteAddr, count);
                WriteAddr += count;
                pBuffer += count;

                sFLASH_WritePage(pBuffer, WriteAddr, temp);
            }
            else
            {
                sFLASH_WritePage(pBuffer, WriteAddr, NumByteToWrite);
            }
        }
        else   /*!< NumByteToWrite > sFLASH_PAGESIZE */
        {
            NumByteToWrite -= count;
            NumOfPage = NumByteToWrite / sFLASH_SPI_PAGESIZE;
            NumOfSingle = NumByteToWrite % sFLASH_SPI_PAGESIZE;

            sFLASH_WritePage(pBuffer, WriteAddr, count);
            WriteAddr += count;
            pBuffer += count;

            while (NumOfPage--)
            {
                sFLASH_WritePage(pBuffer, WriteAddr, sFLASH_SPI_PAGESIZE);
                WriteAddr += sFLASH_SPI_PAGESIZE;
                pBuffer += sFLASH_SPI_PAGESIZE;
            }

            if (NumOfSingle != 0)
            {
                sFLASH_WritePage(pBuffer, WriteAddr, NumOfSingle);
            }
        }
    }
}


/**
 * @brief  Reads a block of data from the FLASH and compares each byte with the buffer.
 * @param  pBuffer: pointer to the buffer that receives the data read from the FLASH.
 * @param  ReadAddr: FLASH's internal address to read from.
 * @param  NumByteToRead: number of bytes to read from the FLASH.
 * @retval returns the number of bytes that are different, zero if all bytes are correct
 */
uint32_t sFLASH_VerifyWrite(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead)
{
    uint32_t count_errors = 0;
    uint8_t byteinflash;

    command[0] = sFLASH_CMD_READ;
    command[1] = (ReadAddr & 0xFF0000) >> 16;
    command[2] = (ReadAddr & 0xFF00) >> 8;
    command[3] = ReadAddr & 0xFF;

    /*!< Select the FLASH: Chip Select low */
    sFLASH_CS_LOW();
    /*!< Send Sector Erase instruction */
    switch(HAL_SPI_Transmit(&hspi3, &command[0], 4, 5000))
    {
    case HAL_TIMEOUT:
        /* A Timeout Occur ______________________________________________________*/
        /* Call Timeout Handler */
        return 1;
        break;
        /* An Error Occur ______________________________________________________ */
    case HAL_ERROR:
        /* Call Timeout Handler */
        return 1;
        break;
    default:
        break;
    }

    command[0] = sFLASH_DUMMY_BYTE;
    while (NumByteToRead--) /*!< while there is data to be read */
    {
        /*!< Read a byte from the FLASH */

        switch(HAL_SPI_TransmitReceive(&hspi3, &command[0], &byteinflash, 1, 5000))
        {
        case HAL_TIMEOUT:
            /* A Timeout Occur ______________________________________________________*/
            /* Call Timeout Handler */
            return 1;
            break;
            /* An Error Occur ______________________________________________________ */
        case HAL_ERROR:
            /* Call Timeout Handler */
            return 1;
            break;
        default:
            break;
        }
        if (*pBuffer != byteinflash)
        {
            count_errors++;
        }
        /*!< Point to the next location where the byte read will be saved */
        pBuffer++;
    }

    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH();
    return count_errors;
}

/**
 * @brief  Reads a block of data from the FLASH.
 * @param  pBuffer: pointer to the buffer that receives the data read from the FLASH.
 * @param  ReadAddr: FLASH's internal address to read from.
 * @param  NumByteToRead: number of bytes to read from the FLASH.
 * @retval None
 */
void sFLASH_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr,	uint32_t NumByteToRead)
{

    /*!< Send "Read from Memory " instruction */

    command[0] = sFLASH_CMD_READ;
    command[1] = (ReadAddr & 0xFF0000) >> 16;
    command[2] = (ReadAddr & 0xFF00) >> 8;
    command[3] = ReadAddr & 0xFF;

    /*!< Select the FLASH: Chip Select low */
    sFLASH_CS_LOW();
    /*!< Send Sector Erase instruction */
    switch(HAL_SPI_Transmit(&hspi3, &command[0], 4, 5000))
    {
    case HAL_TIMEOUT:
        /* A Timeout Occur ______________________________________________________*/
        /* Call Timeout Handler */
        return;
        break;
        /* An Error Occur ______________________________________________________ */
    case HAL_ERROR:
        /* Call Timeout Handler */
        return;
        break;
    default:
        break;
    }

    command[0] = sFLASH_DUMMY_BYTE;
    while (NumByteToRead--) /*!< while there is data to be read */
    {
        /*!< Read a byte from the FLASH */
        switch(HAL_SPI_TransmitReceive(&hspi3, &command[0], pBuffer, 1, 5000))
        {
        case HAL_TIMEOUT:
            /* A Timeout Occur ______________________________________________________*/
            /* Call Timeout Handler */
            return;
            break;
            /* An Error Occur ______________________________________________________ */
        case HAL_ERROR:
            /* Call Timeout Handler */
            return;
            break;
        default:
            break;
        }
        /*!< Point to the next location where the byte read will be saved */
        pBuffer++;
    }

    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH();
}

/**
 * @brief  Reads FLASH identification.
 * @param  None
 * @retval FLASH identification
 */
uint32_t sFLASH_ReadID(void)
{
    uint32_t Temp = 0, Temp0 = 0, Temp1 = 0, Temp2 = 0;
    uint8_t idsequence[3];

    /*!< Select the FLASH: Chip Select low */
    sFLASH_CS_LOW();

    /*!< Send "RDID " instruction */
    command[0] = 0x9F;
    switch(HAL_SPI_Transmit(&hspi3, &command[0], 1, 5000))
    {
    case HAL_TIMEOUT:
        /* A Timeout Occur ______________________________________________________*/
        /* Call Timeout Handler */
        return 1;
        break;
        /* An Error Occur ______________________________________________________ */
    case HAL_ERROR:
        /* Call Timeout Handler */
        return 1;
        break;
    default:
        break;
    }

    command[0] = sFLASH_DUMMY_BYTE;
    command[1] = sFLASH_DUMMY_BYTE;
    command[2] = sFLASH_DUMMY_BYTE;

    /*!< Read a byte from the FLASH */
    switch(HAL_SPI_TransmitReceive(&hspi3, &command[0], &idsequence[0], 3, 5000))
    {
    case HAL_TIMEOUT:
        /* A Timeout Occur ______________________________________________________*/
        /* Call Timeout Handler */
        return 1;
        break;
        /* An Error Occur ______________________________________________________ */
    case HAL_ERROR:
        /* Call Timeout Handler */
        return 1;
        break;
    default:
        break;
    }

    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH();

    Temp0 = idsequence[0];
    Temp1 = idsequence[1];
    Temp2 = idsequence[2];

    Temp = (Temp0 << 16) | (Temp1 << 8) | Temp2;

    return Temp;
}



/**
 * @brief  Enables the write access to the FLASH.
 * @param  None
 * @retval None
 */
void sFLASH_WriteEnable(void)
{
    /*!< Select the FLASH: Chip Select low */
    sFLASH_CS_LOW();

    /*!< Send "Write Enable" instruction */
    command[0] = sFLASH_CMD_WREN;
    switch(HAL_SPI_Transmit(&hspi3, &command[0], 1, 5000))
    {
    case HAL_TIMEOUT:
        /* A Timeout Occur ______________________________________________________*/
        /* Call Timeout Handler */
        return;
        break;
        /* An Error Occur ______________________________________________________ */
    case HAL_ERROR:
        /* Call Timeout Handler */
        return;
        break;
    default:
        break;
    }

    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH();
}

/**
 * @brief  Polls the status of the Write In Progress (WIP) flag in the FLASH's
 *         status register and loop until write opertaion has completed.
 * @param  None
 * @retval None
 */
void sFLASH_WaitForWriteEnd(void)
{
    uint8_t flashstatus = 0;

    /*!< Select the FLASH: Chip Select low */
    sFLASH_CS_LOW();

    /*!< Send "Read Status Register" instruction */
    command[0] = sFLASH_CMD_RDSR;
    switch(HAL_SPI_Transmit(&hspi3, &command[0], 1, 5000))
    {
    case HAL_TIMEOUT:
        /* A Timeout Occur ______________________________________________________*/
        /* Call Timeout Handler */
        return;
        break;
        /* An Error Occur ______________________________________________________ */
    case HAL_ERROR:
        /* Call Timeout Handler */
        return;
        break;
    default:
        break;
    }

    command[0] = sFLASH_DUMMY_BYTE;

    /*!< Loop as long as the memory is busy with a write cycle */
    do
    {
        /*!< Send a dummy byte to generate the clock needed by the FLASH
         and put the value of the status register in FLASH_Status variable */
        switch(HAL_SPI_TransmitReceive(&hspi3, &command[0], &flashstatus, 1, 5000))
        {
        case HAL_TIMEOUT:
            /* A Timeout Occur ______________________________________________________*/
            /* Call Timeout Handler */
            return;
            break;
            /* An Error Occur ______________________________________________________ */
        case HAL_ERROR:
            /* Call Timeout Handler */
            return;
            break;
        default:
            break;
        }

    }
    while ((flashstatus & sFLASH_WIP_FLAG) == SET);   /* Write in progress */

    /*!< Deselect the FLASH: Chip Select high */
    sFLASH_CS_HIGH();
}
