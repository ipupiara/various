/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file         stm32f1xx_hal_msp.c
  * @brief        This file provides code for the MSP Initialization
  *               and de-Initialization codes.
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
  ******************************************************************************/

#include "main.h"
#include <stm32f1xx_it.h>
#include <cpu.h>


void HAL_MspInit(void)
{
  /* USER CODE BEGIN MspInit 0 */

  /* USER CODE END MspInit 0 */

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_PWR_CLK_ENABLE();


}


/*
*********************************************************************************************************
*                                          SYS TICK DEFINES
*********************************************************************************************************
*/

#define  OS_CPU_CM_NVIC_ST_CTRL    (*((volatile uint32_t *)0xE000E010uL)) /* SysTick Ctrl & Status Reg.                  */
#define  OS_CPU_CM_NVIC_ST_RELOAD  (*((volatile uint32_t *)0xE000E014uL)) /* SysTick Reload  Value Reg.                  */
#define  OS_CPU_CM_NVIC_ST_CURRENT (*((volatile uint32_t *)0xE000E018uL)) /* SysTick Current Value Reg.                  */
#define  OS_CPU_CM_NVIC_ST_CAL     (*((volatile uint32_t *)0xE000E01CuL)) /* SysTick Cal     Value Reg.                  */
#define  OS_CPU_CM_NVIC_SHPRI1     (*((volatile uint32_t *)0xE000ED18uL)) /* System Handlers  4 to  7 Prio.              */
#define  OS_CPU_CM_NVIC_SHPRI2     (*((volatile uint32_t *)0xE000ED1CuL)) /* System Handlers  8 to 11 Prio.              */
#define  OS_CPU_CM_NVIC_SHPRI3     (*((volatile uint32_t *)0xE000ED20uL)) /* System Handlers 12 to 15 Prio.              */


#define  OS_CPU_CM_NVIC_ST_CTRL_COUNT                    0x00010000uL   /* Count flag.                                 */
#define  OS_CPU_CM_NVIC_ST_CTRL_CLK_SRC                  0x00000004uL   /* Clock Source.                               */
#define  OS_CPU_CM_NVIC_ST_CTRL_INTEN                    0x00000002uL   /* Interrupt enable.                           */
#define  OS_CPU_CM_NVIC_ST_CTRL_ENABLE                   0x00000001uL   /* Counter mode.                               */
#define  OS_CPU_CM_NVIC_PRIO_MIN                               0xFFu    /* Min handler prio.                           */

#define  OS_CPU_CFG_SYSTICK_PRIO           0u

void  OS_CPU_SysTickInit (uint32_t  cnts)
{
    uint32_t  prio;


    OS_CPU_CM_NVIC_ST_RELOAD = cnts - 1u;

                                                                /* Set SysTick handler prio.                            */
    prio  =  OS_CPU_CM_NVIC_SHPRI3;
    prio &=  0x00FFFFFFu;
    prio |= (OS_CPU_CFG_SYSTICK_PRIO << 24u);

    OS_CPU_CM_NVIC_SHPRI3 = prio;

                                                                // Enable timer.
    OS_CPU_CM_NVIC_ST_CTRL |= OS_CPU_CM_NVIC_ST_CTRL_CLK_SRC |
                              OS_CPU_CM_NVIC_ST_CTRL_ENABLE;
                                                                // Enable timer interrupt.
    OS_CPU_CM_NVIC_ST_CTRL |= OS_CPU_CM_NVIC_ST_CTRL_INTEN;
}

void  BSP_OS_TickEnable (void)
{
    CPU_REG_SYST_CSR |= (CPU_REG_SYST_CSR_TICKINT |             // Enables SysTick exception request
                         CPU_REG_SYST_CSR_ENABLE);              // Enables SysTick counter
}

void  BSP_OS_TickDisable (void)
{
    CPU_REG_SYST_CSR &= ~(CPU_REG_SYST_CSR_TICKINT |            //Disables SysTick exception request
                          CPU_REG_SYST_CSR_ENABLE);             // Disables SysTick counter
}


void startSystemTimer()
{
	 uint32_t  cpu_freq;

	 uint32_t  cnts;
	 cpu_freq = HAL_RCC_GetSysClockFreq();
	 cnts = (cpu_freq / 1000);               /* Determine nbr SysTick cnts between two OS tick intr. */

	 OS_CPU_SysTickInit(cnts);
	 BSP_OS_TickDisable();
}

