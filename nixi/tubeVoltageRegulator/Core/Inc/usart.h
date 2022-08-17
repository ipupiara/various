/*
 * usart.h
 *
 *  Created on: Jun 9, 2022
 *      Author: peetz
 */

#ifndef INC_USART_H_
#define INC_USART_H_

#include <main.h>

extern uint8_t dataReceivedUart1;

extern uint32_t hygrosenseMsgCnt;

void initUart();
void USART1_IRQHandler(void);
uint8_t onDataReceivedUart1IsValid();
floatType getCurrentTemperature();
floatType getCurrentHumidity();

#endif /* INC_USART_H_ */
