/*
 * usart.c
 *
 *  Created on: Jun 9, 2022
 *      Author: peetz
 */
#include <usart.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"

UART_HandleTypeDef huart1;
GPIO_InitTypeDef GPIO_InitStruct = {0};







void initUsart()
{
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	 /**USART1 GPIO Configuration
	    PA9     ------> USART1_TX
	    PA10     ------> USART1_RX
	    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	huart1.Instance = USART1;
	huart1.Init.BaudRate = 4800;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart1) != HAL_OK)
	{
	//Error_Handler();
	}

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

}

void receiveUartByte(uint8_t by)
{

}

uint8_t  getNextByte(uint8_t * bt)
{
	uint8_t res = 0;
	*bt = 0;
	return res;
}

void USART1_IRQHandler(void)
{
	  uint32_t isrflags   = READ_REG(huart1.Instance->SR);
//	  uint32_t cr1its     = READ_REG(huart1.Instance->CR1);
//	  uint32_t cr3its     = READ_REG(huart1.Instance->CR3);
	  uint32_t errorflags = 0x00U;
//	  uint32_t dmarequest = 0x00U;


	  errorflags = (isrflags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
	  if (errorflags == 0)
	  {
	    if ((isrflags & USART_SR_RXNE) != 0)
	    {
	    	uint8_t ch = READ_REG(huart1.Instance->DR);
	       receiveUartByte(ch);
	    }
	  }
	  if (errorflags != 0)
	  {
	    // UART parity error interrupt occurred ----------------------------------
	    if ((isrflags & USART_SR_PE) != 0)
	    {

	    }

	    // UART noise error interrupt occurred -----------------------------------
	    if ((isrflags & USART_SR_NE) != 0)
	    {

	    }

	    // UART frame error interrupt occurred -----------------------------------
	    if ((isrflags & USART_SR_FE) != 0)
	    {

	    }

	    // UART Over-Run interrupt occurred --------------------------------------
	    if ((isrflags & USART_SR_ORE) != 0)
	    {

	    }


	  }

	  // UART in mode Transmitter ------------------------------------------------
	   if ((isrflags & USART_SR_TXE) != 0)
	   {
//		   uint8_t bt;
//		   uint8_t res;
//		   res= getNextByte(&bt);
//		   if (res != 0) {
//			   WRITE_REG(huart1.Instance->DR,bt);
//		   }

//	     UART_Transmit_IT(huart);
	   }

	   // UART in mode Transmitter end --------------------------------------------
	   if ((isrflags & USART_SR_TC) != 0)
	   {
		   // transmitter is idle, important if later also transmit would be needed
//	     UART_EndTransmit_IT(huart);

	   }


//  HAL_UART_IRQHandler(&huart1);

}


