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

void initUsart();
void USART1_IRQHandler(void);
uint8_t onDataReceivedUart1IsValid();

#endif /* INC_USART_H_ */
