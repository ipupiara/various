/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

typedef float floatType;

void setDebugOneOn();
void setDebugOneOff();
void toggleDebugOne();

void Error_Handler(void);

//void sec1msTick();
void triggerAdc1();

void startSystemTimer(void);
void BSP_OS_TickEnable(void);
void  BSP_OS_TickDisable (void);



void startHvPwm();
void stopHvPwm();

enum {
	hvPwmIdle,
	hvPwmRunning
};

extern uint8_t  i2cMessageReceived;
extern uint8_t  i2cMessageSent;		// 1 = ok, 0 = nothing received / sent, all other mean error

extern uint8_t  sec1msEvent;
extern uint8_t  humidTempRequired;
extern uint8_t  secondTickValue;
extern uint8_t  hvPwmState;

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
