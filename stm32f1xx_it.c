/**
 ******************************************************************************
 * @file    UART/UART_TwoBoards_ComIT/Src/stm32f1xx_it.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    17-December-2014
 * @brief   Main Interrupt Service Routines.
 *          This file provides template for all exceptions handler and
 *          peripherals interrupt service routine.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
#include "stm32_systick.h"
#include "usbd_cdc_interface.h"
#include "clock_50Hz.h"


/** @addtogroup STM32F1xx_HAL_Examples
 * @{
 */

/** @addtogroup UART_TwoBoards_ComIT
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* UART handler declared in "main.c" file */
extern UART_HandleTypeDef Uart1Handle;
extern UART_HandleTypeDef Uart3Handle;
extern TIM_HandleTypeDef    Tim1HandleServoIn;
extern uint32_t Systick_Delay;
extern uint32_t Systick_T1;
extern uint32_t Systick_T2;
extern COUNTER_t Systick_C1;
extern COUNTER_t Systick_C2;

extern PCD_HandleTypeDef hpcd;

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief   This function handles NMI exception.
 * @param  None
 * @retval None
 */
void NMI_Handler(void)
{
}

/**
 * @brief  This function handles Hard Fault exception.
 * @param  None
 * @retval None
 */
void HardFault_Handler(void)
{
	/* Go to infinite loop when Hard Fault exception occurs */
	hardstop();
	while (1)
	{
	}
}

/**
 * @brief  This function handles Memory Manage exception.
 * @param  None
 * @retval None
 */
void MemManage_Handler(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	hardstop();
	while (1)
	{
	}
}

/**
 * @brief  This function handles Bus Fault exception.
 * @param  None
 * @retval None
 */
void BusFault_Handler(void)
{
	/* Go to infinite loop when Bus Fault exception occurs */
	hardstop();
	while (1)
	{
	}
}

/**
 * @brief  This function handles Usage Fault exception.
 * @param  None
 * @retval None
 */
void UsageFault_Handler(void)
{
	/* Go to infinite loop when Usage Fault exception occurs */
	hardstop();
	while (1)
	{
	}
}

/**
 * @brief  This function handles SVCall exception.
 * @param  None
 * @retval None
 */
void SVC_Handler(void)
{
}

/**
 * @brief  This function handles Debug Monitor exception.
 * @param  None
 * @retval None
 */
void DebugMon_Handler(void)
{
}

/**
 * @brief  This function handles PendSVC exception.
 * @param  None
 * @retval None
 */
void PendSV_Handler(void)
{
}

/**
 * @brief  This function handles SysTick Handler.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void)
{
	HAL_IncTick();
	// Tick für Pause
	if(Systick_Delay != 0x00) {
		Systick_Delay--;
	}
	// Tick für Timer1
	if(Systick_T1 != 0x00) {
		Systick_T1--;
	}
	// Tick für Timer2
	if(Systick_T2 != 0x00) {
		Systick_T2--;
	}
	// Tick für Counter1
	if(Systick_C1.faktor != 0x00) {
		Systick_C1.wert++;
	}
	// Tick für Counter2
	if(Systick_C2.faktor != 0x00) {
		Systick_C2.wert++;
	}
	if (HAL_GetTick() % 20 == 0) {
		tickCounter();
		USBPeriodElapsed();
	}
}

/******************************************************************************/
/*                 STM32F1xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f1xx.s).                                               */
/******************************************************************************/
/**
 * @brief  This function handles UART interrupt request.
 * @param  None
 * @retval None
 * @Note   This function is redefined in "main.h" and related to DMA
 *         used for USART data transmission
 */
void USART3_IRQHandler(void)
{
	HAL_UART_IRQHandler(&Uart3Handle);
}

void USART1_IRQHandler(void)
{
	HAL_UART_IRQHandler(&Uart1Handle);
}

/**
  * @brief  This function handles USB Handler.
  * @param  None
  * @retval None
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd);
}


/**
 * @brief  This function handles TIMx global interrupt request.
 * @param  None
 * @retval None
 */
void TIM1_CC_IRQHandler(void)
{
	HAL_TIM_IRQHandler(&Tim1HandleServoIn);
}


/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
