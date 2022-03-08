/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include <stm32f1xx_it.h>
#include <cpu.h>
#include <string.h>
#include "main.h"
#include <nixi_i2c.h>
#include <screen.h>


#define useDebugPort


ADC_HandleTypeDef hadc1;
uint16_t lastADCResult;

TIM_HandleTypeDef htim2;
//TIM_HandleTypeDef htim3;


#ifdef useDebugPort
#define debugPin1_Pin GPIO_PIN_13
#define debugPin1_GPIO_Port GPIOC
#endif




void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_ADC1_Init(void);
void MX_TIM2_Init(void);

uint8_t i2cSec100MsgPending;

void sec100Tick()
{
//	triggerAdc1();

	  screenCentiSecTimer();

//	i2cSec100MsgPending = 1;
}


uint16_t clockRelValueVsMaxClk(uint16_t valAtMax)
{
	uint16_t res = 0;
	double dValAtMax = (double) valAtMax;

	double dMaxF = 72000000.0;
	double dActF = (double) HAL_RCC_GetHCLKFreq();


	double dValAtAct = dValAtMax / dMaxF;
	dValAtAct = dValAtAct *  dActF;
	res = (uint16_t) dValAtAct;

	return res;
}


void startHvPwm()
{

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	hvPwmState = hvPwmRunning;
}

void stopHvPwm()
{
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	hvPwmState = hvPwmIdle;
}

void initVariables()
{
	i2cMessageReceived = 0;
	i2cMessageSent = 0;
	lastADCResult = 0;
	i2cSec100MsgPending = 0;
	hvPwmState = hvPwmIdle;
	sec100Event = 0;
}

uint8_t debugTrigger;			// can be used for any needed kind of debugging

int main(void)
{
	initVariables();

	HAL_Init();

  SystemClock_Config();


  MX_GPIO_Init();
//  MX_TIM2_Init();
//  MX_ADC1_Init();
  initI2c();
  initScreen();
//  startSystemTimer();
//  BSP_OS_TickEnable();
   while (1)
  {
	   if (i2cSec100MsgPending != 0){

		   i2cSec100MsgPending = 0;
		   uint8_t  arr [1];
		  arr[0]=0xbb;
		  sendI2cByteArray(0x11,arr,0);
//		uint8_t stri [] = {};
//			   sendI2cByteArray(0x3c,stri,strlen((char*)stri));

	   }
	   if (sec100Event == 1)  {
		   	  sec100Event = 0;
		   	  sec100Tick();
	   }
	   if (i2cMessageReceived != 0)  {
		   if (i2cMessageReceived == 1) {

		   }  else {

		   }
		   i2cMessageReceived = 0;
	   }
	   if (i2cInitNeeded == 1) {
	   		   i2cInitNeeded = 0;
	   		   i2cReInitAfterFailure();
	   }
	   if (i2cMessageSent != 0)  {
		   if (i2cMessageSent == 1) {

		   }  else {

		   }
		   i2cMessageSent = 0;
	   }
	   if (debugTrigger == 1) {
		 debugTrigger = 0;

		 setDebugScreenJob();
	   }

  }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
   *
   *
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}



void setDebugOneOn()
{
#ifdef useDebugPort
	HAL_GPIO_WritePin(debugPin1_GPIO_Port, debugPin1_Pin, GPIO_PIN_RESET);
#endif
}

void setDebugOneOff()
{
	#ifdef useDebugPort
		HAL_GPIO_WritePin(debugPin1_GPIO_Port, debugPin1_Pin, GPIO_PIN_SET);
	#endif
}

void toggleDebugOne()
{
	CPU_SR_ALLOC();           //   critcal - methods used for debuging  reasons only
	  CPU_CRITICAL_ENTER();
	#ifdef useDebugPort
		HAL_GPIO_TogglePin(debugPin1_GPIO_Port, debugPin1_Pin);
	#endif
	CPU_CRITICAL_ENTER();
}



/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
void MX_ADC1_Init(void)
{


	GPIO_InitTypeDef GPIO_InitStruct = {0};
  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  __HAL_RCC_ADC1_CLK_ENABLE();

   __HAL_RCC_GPIOA_CLK_ENABLE();

  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.HighThreshold = 0x850;
  AnalogWDGConfig.LowThreshold = 0x800;
  AnalogWDGConfig.Channel = ADC_CHANNEL_1;
  AnalogWDGConfig.ITMode = ENABLE;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_NVIC_SetPriority(ADC1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC1_IRQn);

  __HAL_ADC_ENABLE_IT(&hadc1,ADC_IT_EOC);
  __HAL_ADC_ENABLE_IT(&hadc1,ADC_IT_AWD);


  SET_BIT(hadc1.Instance->CR2, ADC_CR2_ADON);

  SET_BIT(hadc1.Instance->CR1,ADC_CR1_AWDSGL);
  SET_BIT(hadc1.Instance->CR1,ADC_CR1_AWDEN);

  SET_BIT(hadc1.Instance->CR2, ADC_CR2_ADON);
}

void triggerAdc1()
{
//	SET_BIT(hadc1.Instance->CR2, ADC_CR2_SWSTART);
	SET_BIT(hadc1.Instance->CR2, ADC_CR2_ADON);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
void MX_TIM2_Init(void)
{

   //  PWM  on PA0

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
//  htim2.Init.Period = 3000;
  htim2.Init.Period = clockRelValueVsMaxClk(3000);
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;

//  sConfigOC.Pulse = 600;
  sConfigOC.Pulse = clockRelValueVsMaxClk(600);
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


}

/*

static void MX_TIM3_Init(void)
{
	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};
	  TIM_OC_InitTypeDef sConfigOC = {0};

	  __HAL_RCC_TIM3_CLK_ENABLE();

	  htim3.Instance = TIM3;
	  htim3.Init.Prescaler = 0;
	  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim3.Init.Period = 12345;
	  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sConfigOC.OCMode = TIM_OCMODE_TIMING;
	  sConfigOC.Pulse = 0;
	  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	 HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
	 HAL_NVIC_EnableIRQ(TIM3_IRQn);

	 __HAL_TIM_ENABLE_IT(&htim3,TIM_IT_CC1);
}

*/


/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{


  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
#ifdef useDebugPort
	GPIO_InitTypeDef GPIO_InitStruct = {0};
#endif

   __HAL_RCC_GPIOC_CLK_ENABLE();
   __HAL_RCC_GPIOD_CLK_ENABLE();

#ifdef useDebugPort

   HAL_GPIO_WritePin(debugPin1_GPIO_Port, debugPin1_Pin, GPIO_PIN_RESET);

   GPIO_InitStruct.Pin = debugPin1_Pin;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
   HAL_GPIO_Init(debugPin1_GPIO_Port, &GPIO_InitStruct);
#endif
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

