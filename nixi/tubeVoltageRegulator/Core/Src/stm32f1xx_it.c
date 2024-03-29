/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_it.h"
#include <cpu.h>


extern ADC_HandleTypeDef hadc1;
extern uint16_t lastADCResult;

extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim3;



/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{

  HAL_IncTick();
  sec1msEvent = 1;

//  if ((uwTick % 31) == 0)   { //       3203) == 0)
//	  humidTempRequired = 1;
//  }

}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles ADC1 and ADC2 global interrupts.
  */




void ADC1_2_IRQHandler(void)
{
	  if(__HAL_ADC_GET_IT_SOURCE(&hadc1, ADC_IT_EOC))
	  {
	    if(__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_EOC) )
	    {
	      __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_EOC);

	      lastADCResult = (uint16_t)  (hadc1.Instance->DR & 0x0000FFFF);
	      toggleDebugOne();
	    }
	  }

	  if(__HAL_ADC_GET_IT_SOURCE(&hadc1, ADC_IT_AWD))
	  {
	    if(__HAL_ADC_GET_FLAG(&hadc1, ADC_FLAG_AWD))
	    {
	      __HAL_ADC_CLEAR_FLAG(&hadc1, ADC_FLAG_AWD);

	      if ( lastADCResult   > hadc1.Instance->HTR ) {
	    	  if (hvPwmState == hvPwmRunning)  {
	    		  stopHvPwm();
	    	  }
	      }  else if (lastADCResult < hadc1.Instance->LTR) {
	    	  if (hvPwmState == hvPwmIdle)  {
				  startHvPwm();
			  }
	      }
	    }
	  }
}

//void TIM3_IRQHandler(void)
//{
//	  if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_CC1) != RESET)
//	  {
//	    if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_CC1) != RESET)
//	    {
//	      {
//	        __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC1);
//	        toggleDebugOne();
////	        triggerAdc1();
//	      }
//	    }
//	  }
//
//}

void cli()
{
//	 CPU_CRITICAL_ENTER();
}

void sei()
{
//	 CPU_CRITICAL_EXIT();
}

