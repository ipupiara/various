/*
 * usart.c
 *
 *  Created on: Jun 9, 2022
 *      Author: peetz
 */
#include <usart.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"

UART_HandleTypeDef huart1;
GPIO_InitTypeDef GPIO_InitStruct = {0};


/////////////   USART  ///////////////////


#define startChar 0x40
#define stopChar   0x24
#define amtChars  66
#define rxBufferSz  100
char rxBuffer [rxBufferSz];

uint8_t dataReceivedUart1;
uint8_t hygrosenseMsgCnt;

enum rxStates {
	rxIdle,
	rxReceiving,
	rxReceived,
};

uint8_t  rxState;
uint8_t  rxCurrentPos;
uint16_t amtCharRcvd;
uint16_t errMsgCnt;

floatType   latestTemperature;
floatType   latestHumidity;


void enaRXIntUsart1()
{
//	UCSR1B   |= (1 << RXCIE1);
}

void disaRXIntUsart1()
{
//	UCSR1B   &= ~(1 << RXCIE1);  so far n ot needed in stm32F103
}


void getLatestClimateValues(floatType* pTemp,floatType* pHum)    // interface to hygrosense, called by user functions
{
	*pTemp = latestTemperature;
	*pHum  = latestHumidity;
}


floatType getCurrentTemperature()
{
	return latestTemperature;
}

floatType getCurrentHumidity()
{
	return latestHumidity;
}


char * reallyWorkingStrstr(const char *inStr, const char *subStr)
{
	char firstSubChar;
	size_t len;
	firstSubChar = *subStr++;
	if (!firstSubChar)
	return (char *) inStr;	// Trivial empty string case

	len = strlen(subStr);
	do {
		char currentInChar;

		do {
			currentInChar = *inStr++;
			if (!currentInChar)
			return (char *) 0;
		} while (currentInChar != firstSubChar);
	} while (strncmp(inStr, subStr, len) != 0);

	return (char *) (inStr - 1);
}




uint8_t onDataReceivedUart1IsValid()        // called by main application thread to calculate the latest data
{
	char tempS [5];
	char hydS [5];
	uint8_t validMsg = 0;

	char * v01Pos = (char *) 0;
	char * v02Pos = (char *) 0;


	memset (tempS,0,sizeof(tempS));
	memset (hydS,0,sizeof(hydS));

	//cli();
	disaRXIntUsart1();   // just stop the receiver, triac continues
//		info_printf("amtChars %i\n",amtCharRcvd);
		if ((rxState = rxReceived) && ( amtCharRcvd == amtChars))  {      // some valid message check
			validMsg = 1;
			if (v01Pos == 0) {
				v01Pos = reallyWorkingStrstr((char* )&rxBuffer,"V01");
			}
			if (v02Pos == 0) {
				v02Pos = reallyWorkingStrstr((char*)&rxBuffer,"V02");
			}
			++hygrosenseMsgCnt;
			strncpy(tempS,v01Pos+3,4);
			strncpy(hydS,v02Pos+3,4);
			rxState = rxIdle;
		}  else  {
			++ errMsgCnt;
		}
	enaRXIntUsart1();
	//sei();
	if (validMsg != 0)  {
		char* endP = tempS+3;
		floatType temp = strtoul(tempS ,&endP,0x10) ;
		temp = temp / 100;
		endP = hydS + 3;
		floatType hyd = strtoul(hydS ,&endP,0x10) ;
		hyd = hyd / 200;

		latestTemperature = temp;
		latestHumidity = hyd;
#ifdef controlheating
		controlTemperature(&temp);
#endif
	}
	return validMsg;
}


void receiveUartByte(uint8_t rxCh)
{
	if (rxCh == startChar)  {
		amtCharRcvd = 0;
		dataReceivedUart1 = 0;
		rxBuffer [amtCharRcvd] = rxCh;
		rxState = rxReceiving;
	} else
	if (rxState == rxReceiving)  {
		++ amtCharRcvd;
		if (amtCharRcvd < rxBufferSz) {
			rxBuffer [amtCharRcvd] = rxCh;
		}
		if (rxCh == stopChar) {   // no  chars lost
			rxState = rxReceived;
			dataReceivedUart1 = 1;
		}
	}
}







void initUsart()
{

	rxState = rxIdle;
	dataReceivedUart1 = 0;
	amtCharRcvd = 0;
	errMsgCnt = 0;
	latestTemperature = 0.00;
	latestHumidity  = 0.00;
	hygrosenseMsgCnt = 0;


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


	  errorflags = (isrflags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE)); // | USART_SR_NE));
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
	    	READ_REG(huart1.Instance->DR);
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


