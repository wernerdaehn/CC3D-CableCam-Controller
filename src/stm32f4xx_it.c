/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "uart_callback.h"
#include "vesc.h"

/* USER CODE BEGIN 0 */
#include "sbus.h"

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim5;
extern uint16_t d;

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
    Error_Handler();
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
    Error_Handler();

}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
    Error_Handler();
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
    Error_Handler();
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
    SBUS_IRQHandler(&huart1);
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
    VESC_IRQHandler(&huart2);
}

/**
* @brief This function handles USART3 global interrupt.
*/
void USART3_IRQHandler(void)
{
    UARTX_IRQHandler(&huart3);
}

/**
* @brief This function handles USART6 global interrupt.
*/
void USART6_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart6);
}


/**
* @brief This function handles USB On The Go FS global interrupt.
*/
void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
}

/**
* @brief This function handles TIM1 capture compare interrupt.
*/
void TIM1_CC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim1);
}

/**
* @brief This function handles TIM5 interrupt.
*/
void TIM5_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim5);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
