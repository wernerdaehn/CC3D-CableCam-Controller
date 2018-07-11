/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "controller.h"
#include "protocol.h"
#include "config.h"

#include "clock_50Hz.h"
#include "sbus.h"
#include "serial_print.h"
#include "usbd_cdc_if.h"
#include "spi_flash.h"
#include "eeprom.h"
#include "math.h"
#include "vesc.h"
#include "systembootloader.h"

#include "uart_callback.h"

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

UART_HandleTypeDef huart6;


/* Private variables ---------------------------------------------------------*/
uint32_t lasttick;
uint8_t uart3_rxbuffer[RXBUFFERSIZE];

extern char usb_commandlinebuffer[RXBUFFERSIZE];
extern char uart3_commandlinebuffer[RXBUFFERSIZE];

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM1_Init(void);

int main(void)
{
    settings_t defaultsettings;

    /* MCU Configuration----------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_RTC_Init();
    MX_SPI1_Init();
    MX_SPI3_Init();
    MX_TIM3_Init();
    MX_USART3_UART_Init();
    MX_USB_DEVICE_Init();
    MX_USART2_UART_Init();
    MX_TIM5_Init();
    MX_USART6_UART_Init();

    /* Disable Half Transfer Interrupt */
    /* __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT); */

    initProtocol();
    VESC_init();

    /* Start encoder in interrupt mode so the counter value is changed and the time of a counter tick can be evaluated */
    HAL_TIM_Encoder_Start_IT(&htim5, TIM_CHANNEL_1 | TIM_CHANNEL_2);

    LED_WARN_OFF;

    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3) != HAL_OK)
    {
        /* PWM generation Error */
        Error_Handler();
    }
    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4) != HAL_OK)
    {
        /* PWM generation Error */
        Error_Handler();
    }

    if (eeprom_init() != 0)
    {
        /* EEPROM is a wrong chip */
        Error_Handler();
    }

    strcpy(activesettings.version, "20180531");
    activesettings.esc_direction = 0;
    activesettings.max_position_error = 100.0f;
    activesettings.mode = MODE_PASSTHROUGH;
    activesettings.receivertype = RECEIVER_TYPE_SUMPPM;
    activesettings.rc_channel_endpoint = 255;
    activesettings.rc_channel_programming = 255;
    activesettings.rc_channel_speed = 0;
    activesettings.rc_channel_max_accel = 255;
    activesettings.rc_channel_max_speed = 255;
    activesettings.rc_channel_mode = 255;
    activesettings.stick_max_accel = 0.01f;
    activesettings.stick_max_accel_safemode = 0.1f;
    activesettings.stick_max_speed = 1.0f;
    activesettings.stick_max_speed_safemode = 0.25f;
    activesettings.stick_neutral_pos = 992;
    activesettings.stick_neutral_range = 30;
    activesettings.esc_value_range = 700;
    activesettings.esc_neutral_pos = 1500;
    activesettings.esc_neutral_range = 30;
    activesettings.stick_value_range = 800;
    activesettings.vesc_max_erpm = 50000;
    activesettings.expo_factor = 0.1f;
    activesettings.vesc_brake_handbrake_max = 60;
    activesettings.vesc_brake_handbrake_min = 10;
    activesettings.vesc_brake_min_speed = 1;
    activesettings.rc_channel_aux = 255;
    activesettings.rc_channel_yaw = 255;
    activesettings.rc_channel_pitch = 255;
    activesettings.rc_channel_roll = 255;
    activesettings.rc_channel_play = 255;
    activesettings.aux_value_range = 700;
    activesettings.aux_neutral_pos = 1500;
    activesettings.aux_neutral_range = 0;
    activesettings.pos_source = 0;
    for (int i=0; i<8; i++)
    {
        activesettings.rc_channel_sbus_out_mapping[i] = 255;
        activesettings.rc_channel_sbus_out_default[i] = 992;
    }
    activesettings.structure_length = sizeof(activesettings);

    eeprom_read_sector((uint8_t *)&defaultsettings, sizeof(defaultsettings), EEPROM_SECTOR_FOR_SETTINGS);
    if (strncmp(activesettings.version, defaultsettings.version, sizeof(activesettings.version)) == 0)
    {
        // Version is the same
        memcpy(&activesettings, &defaultsettings, sizeof(defaultsettings));
        strcpy(controllerstatus.boottext_eeprom, "defaults loaded from eeprom");
    }
    else if (strncmp(defaultsettings.version, "2017", 4) == 0)
    {
        strcpy(controllerstatus.boottext_eeprom, "Values in the EEPROM outdated, please setup all again");
    }
    else if (strncmp(defaultsettings.version, "2018", 4) >= 0)
    {
        /*
         * Version stored is valid but older, hence copy the old structure over the activestructure.
         * If the structure_length argument (introduced later) is valid, this will fill the activesettings with
         * values and leave the new ones untouched, hence the defaults from above.
         */
        char current_version_string[11];
        memcpy(current_version_string, activesettings.version, 11); // capture the current version according to the default activesettings set initially
        uint16_t structuresize = defaultsettings.structure_length;
        if (structuresize == 0 || structuresize > sizeof(defaultsettings))
        {
            structuresize = sizeof(defaultsettings);
        }
        memcpy(&activesettings, &defaultsettings, structuresize); // activesettings are overwritten with the eeprom values
        memcpy(activesettings.version, current_version_string, 11); // As the version is overwritten ans well reapply the version string
        activesettings.structure_length = sizeof(activesettings); // Same with the old structure_length. It has been migrated, hence the current value counts, not the one read from eeprom.
        strcpy(controllerstatus.boottext_eeprom, "defaults loaded from eeprom using an older version");
    }
    else
    {
        strcpy(controllerstatus.boottext_eeprom, "eeprom does not contain valid default - keeping the system defaults");
    }

    /*
     * Read the semi-permanent settings from EEPROM and if they are invalid, reset to default.
     */
    eeprom_read_sector((uint8_t *)&semipermanentsettings, sizeof(semipermanentsettings), EEPROM_SECTOR_FOR_SEMIPERMANENTSETTINGS);
    if (strncmp(semipermanentsettings.version, "20180519", sizeof(semipermanentsettings.version)) != 0)
    {
        PrintlnSerial_string("Start/End-positions have been reset to the defaults", EndPoint_All);
        semipermanentsettings.pos_end = (float) POS_END_NOT_SET;
        semipermanentsettings.pos_start = (float) -POS_END_NOT_SET;
        strcpy(semipermanentsettings.version, "20180519");
    }
    else
    {
        PrintSerial_string("Startposition=", EndPoint_All);
        PrintSerial_float(semipermanentsettings.pos_start, EndPoint_All);
        PrintSerial_string("; Endposition=", EndPoint_All);
        PrintlnSerial_float(semipermanentsettings.pos_end, EndPoint_All);
    }
    semipermanentsettings.structure_length = sizeof(semipermanentsettings);


    if (activesettings.receivertype == RECEIVER_TYPE_SBUS)
    {
        initSBusReceiver();
    }
    else
    {
        initPPMReceiver();
    }

    initController();

    HAL_UART_Receive_IT(&huart2, getRequestValuePacketFrameAddress(), VESC_RXBUFFER_SIZE);

    uart_init(&huart3, uart3_rxbuffer, RXBUFFERSIZE);

    while (1)
    {
        /*
         * HAL_GetTick() returns the hardware system time value in ms.
         * Every time 20 ms have passed, the clock50Hz gets increased by one
         * and hence provides a stable information for slow frequency operations.
         */
        if (HAL_GetTick() - lasttick > CONTROLLERLOOPTIME_MS)
        {
            lasttick = HAL_GetTick();
            tickCounter();

            USBPeriodElapsed();
            if (is1Hz())
            {
                /*
                 * Every second trigger the VESC status
                 */
                VESC_request_values();
            }
            if (is1Hz() && controllerstatus.safemode == OPERATIONAL)
            {
                /*
                 * In operational mode we let the LED toggle slowly, once per second
                 *
                 */
                LED_STATUS_TOGGLE;
            }
            if (is5Hz() && controllerstatus.safemode == PROGRAMMING)
            {
                /*
                 * In programming mode we let the LED toggle quickly, 5 times per second
                 *
                 */
                LED_STATUS_TOGGLE;
            }
            if (controllerstatus.safemode == INVALID_RC ||controllerstatus.safemode == NOT_NEUTRAL_AT_STARTUP)
            {
                /*
                 * When there is no valid RC signal, the WARN LED is turned on
                 *
                 */
                LED_WARN_ON;
            }
            else
            {
                LED_WARN_OFF;
            }
            controllercycle();
        }
        if( USB_ReceiveString() > 0 )
        {
            serialCom(EndPoint_USB, usb_commandlinebuffer);
        }
        if (UART3_ReceiveString()) // with controllerstatus.vesc_config there is no data being added to the receive buffer, hence this function returns false always
        {
            serialCom(EndPoint_UART3, uart3_commandlinebuffer);
        }
        SBusSendCycle();
        eeprom_cycle();
    }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /**Configure the main internal regulator output voltage
    */
    __HAL_RCC_PWR_CLK_ENABLE();

    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB bus clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 4;
    RCC_OscInitStruct.PLL.PLLN = 72;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 3;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Initializes the CPU, AHB and APB bus clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
    PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* RTC init function */
static void MX_RTC_Init(void)
{
    /**Initialize RTC Only
    */
    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 127;
    hrtc.Init.SynchPrediv = 255;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    if (HAL_RTC_Init(&hrtc) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}

/* SPI3 init function */
static void MX_SPI3_Init(void)
{
    sFLASH_CS_HIGH();

    hspi3.Instance = SPI3;
    hspi3.Init.Mode = SPI_MODE_MASTER;
    hspi3.Init.Direction = SPI_DIRECTION_2LINES;
    hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi3.Init.NSS = SPI_NSS_SOFT;
    hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi3.Init.CRCPolynomial = 7;
    if (HAL_SPI_Init(&hspi3) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /* SPI block is enabled prior calling SPI transmit/receive functions, in order to get CLK signal properly pulled down.
         Otherwise, SPI CLK signal is not clean on this board and leads to errors during transfer */
    __HAL_SPI_ENABLE(&hspi3);
}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 16-1;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 20000; // 20ms or 50Hz
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 1500;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    sConfigOC.OCIdleState  = TIM_OCIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    HAL_TIM_MspPostInit(&htim3);
}

/* TIM5 init function */
static void MX_TIM5_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 0;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 4294967295;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 4;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 4;
    if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}


/* TIM1 init function */
static void MX_TIM1_Init(void)
{
    TIM_SlaveConfigTypeDef sSlaveConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_IC_InitTypeDef sConfigIC;

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 16-1;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 0xFFFF;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.RepetitionCounter = 0;
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
    sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
    sSlaveConfig.TriggerPolarity = TIM_TRIGGERPOLARITY_FALLING;
    sSlaveConfig.TriggerFilter = 0;
    if (HAL_TIM_SlaveConfigSynchronization(&htim1, &sSlaveConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }


    // sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    // sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
    sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
    if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}


/* USART1 init function */
/*
 * The SBus receiver
 */
static void MX_USART1_UART_Init(void)
{
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 100000;
    huart1.Init.WordLength = UART_WORDLENGTH_9B;
    huart1.Init.StopBits = UART_STOPBITS_2;
    huart1.Init.Parity = UART_PARITY_EVEN;
    huart1.Init.Mode = UART_MODE_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_8;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}

/* USART2 init function */
/*
 * VESC6 ESC communication
 */
static void MX_USART2_UART_Init(void)
{
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}

/* USART3 init function
 *
 * used vor the bluetooth communication
 */
static void MX_USART3_UART_Init(void)
{
    huart3.Instance = USART3;
    huart3.Init.BaudRate = 38400;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart3) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}

/* USART6 init function */
/*
 * The SBus out (non-inverted) for the gimbal
 */
static void MX_USART6_UART_Init(void)
{
    huart6.Instance = USART6;
    huart6.Init.BaudRate = 100000;
    huart6.Init.WordLength = UART_WORDLENGTH_9B;
    huart6.Init.StopBits = UART_STOPBITS_2;
    huart6.Init.Parity = UART_PARITY_EVEN;
    huart6.Init.Mode = UART_MODE_TX;
    huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart6.Init.OverSampling = UART_OVERSAMPLING_8;
    if (HAL_UART_Init(&huart6) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }
}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

    /*Configure GPIO pin : PC4 */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* SBus Inverter */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PB3 PB4 PB5 */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}




void initSBusReceiver()
{
    HAL_TIM_IC_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIM_IC_Stop(&htim1, TIM_CHANNEL_4);

    MX_USART1_UART_Init();

    /* Turn on interrupt for uart1/sbus */
    HAL_UART_Receive_IT(&huart1, getSBUSFrameAddress(), SBUS_FRAME_SIZE);

    /* Turn on SBUS Inverter */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

    initSBusData(RECEIVER_TYPE_SBUS);
}

void initPPMReceiver()
{
    HAL_UART_DeInit(&huart1);

    MX_TIM1_Init();

    /* ATTENTION: Inverter turned ON as well, hence the SumPPM Interrupt needs to be inverted!!!!!!!!!!
     * Turn on SBUS Inverter as well. It seems some boards do not have this pin connected but use the inverter all the time
     */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);

    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3);
    HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_4);
    initSBusData(RECEIVER_TYPE_SUMPPM);  // In case it is a servo only, the Timer Interrupt will recognize that and change the value
}




/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(const char * file, int line)
{
    /* In case of an error set ESC outputs to idle */
    TIM3->CCR3 = activesettings.esc_neutral_pos;

    uint8_t buffer[4096];
    memcpy(&buffer[0], &line, 2);
    memcpy(&buffer[2], file, strlen(file)+1);

    eeprom_write_sector_safe(buffer, 4096, 5);

    while(1)
    {
    }

}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{


}

#endif

/*
void hard_fault_handler_c(unsigned int * hardfault_args, unsigned int r4, unsigned int r5, unsigned int r6)
{
    PrintSerial_string("[Hard Fault]\n", EndPoint_All);

    PrintSerial_string("r0 =", EndPoint_All);
    PrintSerial_int(hardfault_args[0], EndPoint_All);
    PrintSerial_string(", r1 =", EndPoint_All);
    PrintSerial_int(hardfault_args[1], EndPoint_All);
    PrintSerial_string(", r2 =", EndPoint_All);
    PrintSerial_int(hardfault_args[2], EndPoint_All);
    PrintSerial_string(", r3 =", EndPoint_All);
    PrintlnSerial_int(hardfault_args[3], EndPoint_All);

    PrintSerial_string("r4 =", EndPoint_All);
    PrintSerial_int(r4, EndPoint_All);
    PrintSerial_string(", r5 =", EndPoint_All);
    PrintSerial_int(r5, EndPoint_All);
    PrintSerial_string(", r6 =", EndPoint_All);
    PrintSerial_int(r6, EndPoint_All);
    PrintSerial_string(", sp =", EndPoint_All);
    PrintlnSerial_int((unsigned int)&hardfault_args[8], EndPoint_All);

    PrintSerial_string("r12 =", EndPoint_All);
    PrintSerial_int(hardfault_args[4], EndPoint_All);
    PrintSerial_string(", lr =", EndPoint_All);
    PrintSerial_int(hardfault_args[5], EndPoint_All);
    PrintSerial_string(", pc =", EndPoint_All);
    PrintSerial_int(hardfault_args[6], EndPoint_All);
    PrintSerial_string(", psr =", EndPoint_All);
    PrintlnSerial_int(hardfault_args[7], EndPoint_All);

    PrintSerial_string("bfar =", EndPoint_All);
    PrintSerial_int(*((volatile unsigned int *)(0xE000ED38)), EndPoint_All);
    PrintSerial_string(", cfsr =", EndPoint_All);
    PrintSerial_int(*((volatile unsigned int *)(0xE000ED28)), EndPoint_All);
    PrintSerial_string(", hfsr =", EndPoint_All);
    PrintSerial_int(*((volatile unsigned int *)(0xE000ED2C)), EndPoint_All);
    PrintSerial_string(", dfsr =", EndPoint_All);
    PrintSerial_int(*((volatile unsigned int *)(0xE000ED30)), EndPoint_All);
    PrintSerial_string(", afsr=", EndPoint_All);
    PrintlnSerial_int(*((volatile unsigned int *)(0xE000ED3C)), EndPoint_All);


    USBPeriodElapsed();

    while(1);
}
*/

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
