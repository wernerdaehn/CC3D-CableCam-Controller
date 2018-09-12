/**
  ******************************************************************************
  * File Name          : stm32f4xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization
  *                      and de-Initialization codes.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "serial_print.h"
#include "usbd_cdc_if.h"


extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void)
{
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

    /* System interrupt init*/
    /* MemoryManagement_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
    /* BusFault_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
    /* UsageFault_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
    /* SVCall_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
    /* DebugMonitor_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
    /* PendSV_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void HAL_RTC_MspInit(RTC_HandleTypeDef* hrtc)
{

    if(hrtc->Instance==RTC)
    {
        /* Peripheral clock enable */
        __HAL_RCC_RTC_ENABLE();
    }
}

void HAL_RTC_MspDeInit(RTC_HandleTypeDef* hrtc)
{

    if(hrtc->Instance==RTC)
    {
        /* Peripheral clock disable */
        __HAL_RCC_RTC_DISABLE();
    }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi)
{

    GPIO_InitTypeDef GPIO_InitStruct;
    if(hspi->Instance==SPI1)
    {
        /* Peripheral clock enable */
        __HAL_RCC_SPI1_CLK_ENABLE();

        /**SPI1 GPIO Configuration
        PA4     ------> SPI1_NSS
        PA5     ------> SPI1_SCK
        PA6     ------> SPI1_MISO
        PA7     ------> SPI1_MOSI
        */
        GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
    else if(hspi->Instance==SPI3)
    {
        /* Peripheral clock enable */
        __HAL_RCC_SPI3_CLK_ENABLE();

        /**SPI3 GPIO Configuration
        PA15     ------> SPI3_NSS
        PC10     ------> SPI3_SCK
        PC11     ------> SPI3_MISO
        PC12     ------> SPI3_MOSI
        */
        GPIO_InitStruct.Pin = GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi)
{

    if(hspi->Instance==SPI1)
    {
        /* USER CODE BEGIN SPI1_MspDeInit 0 */

        /* USER CODE END SPI1_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_SPI1_CLK_DISABLE();

        /**SPI1 GPIO Configuration
        PA4     ------> SPI1_NSS
        PA5     ------> SPI1_SCK
        PA6     ------> SPI1_MISO
        PA7     ------> SPI1_MOSI
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
    }
    else if(hspi->Instance==SPI3)
    {
        /* Peripheral clock disable */
        __HAL_RCC_SPI3_CLK_DISABLE();

        /**SPI3 GPIO Configuration
        PA15     ------> SPI3_NSS
        PC10     ------> SPI3_SCK
        PC11     ------> SPI3_MISO
        PC12     ------> SPI3_MOSI
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_15);

        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12);
    }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm)
{

    if(htim_pwm->Instance==TIM3)
    {
        /* Peripheral clock enable */
        __HAL_RCC_TIM3_CLK_ENABLE();
    }
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* htim_encoder)
{

    GPIO_InitTypeDef GPIO_InitStruct;
    if(htim_encoder->Instance==TIM5)
    {
        /* Peripheral clock enable */
        __HAL_RCC_TIM5_CLK_ENABLE();

        /**TIM5 GPIO Configuration
        PA0-WKUP     ------> TIM5_CH1
        PA1          ------> TIM5_CH2
        */
        GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* Turn on interrupt handling on counters */
        HAL_NVIC_SetPriority(TIM5_IRQn, 0, 1);
        HAL_NVIC_EnableIRQ(TIM5_IRQn);
    }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    if(htim_base->Instance==TIM1)
    {
        /* Peripheral clock enable */
        __HAL_RCC_TIM1_CLK_ENABLE();

        /**TIM1 GPIO Configuration
        PA9      ------> TIM1_CH2 (slave - Pin not connected)
        PA10     ------> TIM1_CH3
        */
        GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* TIM1 interrupt Init */
        HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
    }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm)
{

    if(htim_pwm->Instance==TIM3)
    {
        /* Peripheral clock disable */
        __HAL_RCC_TIM3_CLK_DISABLE();
    }

}

void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* htim_encoder)
{

    if(htim_encoder->Instance==TIM5)
    {
        /* Peripheral clock disable */
        __HAL_RCC_TIM5_CLK_DISABLE();

        /**TIM5 GPIO Configuration
        PA0-WKUP     ------> TIM5_CH1
        PA1     ------> TIM5_CH2
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1);
    }
}

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

    GPIO_InitTypeDef GPIO_InitStruct;
    if(huart->Instance==USART1)
    {
        /* Peripheral clock enable */
        __HAL_RCC_USART1_CLK_ENABLE();

        /**USART1 GPIO Configuration
        PA09     ------> USART1_TX
        PA10     ------> USART1_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USART1 interrupt Init */
        HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
    else if(huart->Instance==USART2)
    {
        /* Peripheral clock enable */
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();

        /**USART2 GPIO Configuration
        PA2     ------> USART2_TX
        PA3     ------> USART2_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USART2 DMA Init */
        /* USART2_TX Init */
        hdma_usart2_tx.Instance = DMA1_Stream6;
        hdma_usart2_tx.Init.Channel = DMA_CHANNEL_4;
        hdma_usart2_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart2_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart2_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart2_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart2_tx.Init.Mode = DMA_NORMAL;
        hdma_usart2_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
        hdma_usart2_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        hdma_usart2_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
        hdma_usart2_tx.Init.MemBurst = DMA_MBURST_INC4;
        hdma_usart2_tx.Init.PeriphBurst = DMA_PBURST_INC4;
        if (HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK)
        {
          _Error_Handler(__FILE__, __LINE__);
        }

        __HAL_LINKDMA(huart, hdmatx, hdma_usart2_tx);


        /* USART2_RX Init */
        hdma_usart2_rx.Instance = DMA1_Stream5;
        hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
        hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart2_rx.Init.Mode = DMA_CIRCULAR;
        hdma_usart2_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
        hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        hdma_usart2_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
        hdma_usart2_rx.Init.MemBurst = DMA_MBURST_INC4;
        hdma_usart2_rx.Init.PeriphBurst = DMA_PBURST_INC4;

        if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
        {
          _Error_Handler(__FILE__, __LINE__);
        }

        __HAL_LINKDMA(huart, hdmarx, hdma_usart2_rx);

        HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
        HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 1);
        HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

        /* USART2 interrupt Init */
        HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
    else if(huart->Instance==USART3)
    {
        /* Peripheral clock enable */
        __HAL_RCC_USART3_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_DMA1_CLK_ENABLE();

        /**USART3 GPIO Configuration
        PB10     ------> USART3_TX
        PB11     ------> USART3_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* USART3 DMA Init */
        /* USART3_TX Init */
        hdma_usart3_tx.Instance = DMA1_Stream3;
        hdma_usart3_tx.Init.Channel = DMA_CHANNEL_4;
        hdma_usart3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart3_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart3_tx.Init.Mode = DMA_NORMAL;
        hdma_usart3_tx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_usart3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        hdma_usart3_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
        hdma_usart3_tx.Init.MemBurst = DMA_MBURST_INC4;
        hdma_usart3_tx.Init.PeriphBurst = DMA_PBURST_INC4;
        if (HAL_DMA_Init(&hdma_usart3_tx) != HAL_OK)
        {
          _Error_Handler(__FILE__, __LINE__);
        }

        __HAL_LINKDMA(huart, hdmatx, hdma_usart3_tx);

         /* USART3_RX Init */
        hdma_usart3_rx.Instance = DMA1_Stream1;
        hdma_usart3_rx.Init.Channel = DMA_CHANNEL_4;
        hdma_usart3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_usart3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart3_rx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart3_rx.Init.Mode = DMA_CIRCULAR;
        hdma_usart3_rx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_usart3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        hdma_usart3_rx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
        hdma_usart3_rx.Init.MemBurst = DMA_MBURST_INC4;
        hdma_usart3_rx.Init.PeriphBurst = DMA_PBURST_INC4;
        if (HAL_DMA_Init(&hdma_usart3_rx) != HAL_OK)
        {
          _Error_Handler(__FILE__, __LINE__);
        }

        __HAL_LINKDMA(huart, hdmarx, hdma_usart3_rx);

        /* DMA1_Stream1_IRQn interrupt configuration */
        HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
        /* DMA1_Stream3_IRQn interrupt configuration */
        HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 1);
        HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);

        /* USART3 interrupt Init */
        HAL_NVIC_SetPriority(USART3_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART3_IRQn);
    }
    else if(huart->Instance==USART6)
    {
        /* Peripheral clock enable */
        __HAL_RCC_USART6_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_DMA2_CLK_ENABLE();

        /**USART1 GPIO Configuration
        PC06     ------> USART6_TX
        PC07     ------> USART6_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* USART6 DMA Init */
        /* USART6_TX Init */
        hdma_usart6_tx.Instance = DMA2_Stream6;
        hdma_usart6_tx.Init.Channel = DMA_CHANNEL_5;
        hdma_usart6_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
        hdma_usart6_tx.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_usart6_tx.Init.MemInc = DMA_MINC_ENABLE;
        hdma_usart6_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart6_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
        hdma_usart6_tx.Init.Mode = DMA_NORMAL;
        hdma_usart6_tx.Init.Priority = DMA_PRIORITY_LOW;
        hdma_usart6_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        hdma_usart6_tx.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
        hdma_usart6_tx.Init.MemBurst = DMA_MBURST_INC4;
        hdma_usart6_tx.Init.PeriphBurst = DMA_PBURST_INC4;

        if (HAL_DMA_Init(&hdma_usart6_tx) != HAL_OK)
        {
          _Error_Handler(__FILE__, __LINE__);
        }

        __HAL_LINKDMA(huart, hdmatx, hdma_usart6_tx);

        HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

        /* USART6 interrupt Init */
        HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART6_IRQn);

    }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

    if(huart->Instance==USART1)
    {
        /* Peripheral clock disable */
        __HAL_RCC_USART1_CLK_DISABLE();

        /**USART1 GPIO Configuration
        PA09     ------> USART1_TX
        PA10     ------> USART1_RX
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

        /* USART1 interrupt DeInit */
        HAL_NVIC_DisableIRQ(USART1_IRQn);
    }
    else if(huart->Instance==USART2)
    {
        /* Peripheral clock disable */
        __HAL_RCC_USART2_CLK_DISABLE();

        /**USART2 GPIO Configuration
        PA2     ------> USART2_TX
        PA3     ------> USART2_RX
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);
        HAL_DMA_DeInit(huart->hdmatx);
        HAL_DMA_DeInit(huart->hdmarx);

        /* USART2 interrupt DeInit */
        HAL_NVIC_DisableIRQ(USART2_IRQn);
    }
    else if(huart->Instance==USART3)
    {
        /* Peripheral clock disable */
        __HAL_RCC_USART3_CLK_DISABLE();

        /**USART3 GPIO Configuration
        PB10     ------> USART3_TX
        PB11     ------> USART3_RX
        */
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);
        HAL_DMA_DeInit(huart->hdmatx);
        HAL_DMA_DeInit(huart->hdmarx);

        /* USART3 interrupt DeInit */
        HAL_NVIC_DisableIRQ(USART3_IRQn);
    }
    else if(huart->Instance==USART6)
    {
        /* Peripheral clock disable */
        __HAL_RCC_USART6_CLK_DISABLE();

        /**USART3 GPIO Configuration
        PC6     ------> USART6_TX
        PC7     ------> USART6_RX
        */
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7);
        HAL_DMA_DeInit(huart->hdmatx);

        /* USART3 interrupt DeInit */
        HAL_NVIC_DisableIRQ(USART6_IRQn);
    }
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
