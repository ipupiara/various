/*
 * screen.h
 *
 *  Created on: Jan 2, 2022
 *      Author: Brigitte
 */

#include<stdint.h>

#ifndef INC_SCREEN_H_
#define INC_SCREEN_H_

typedef enum  {
	screenWaitActive,
	screenWaitInactive
}  waitStates;

void screenMillisecTimer ();

void initScreen();
uint8_t screenWaitState;


#endif /* INC_SCREEN_H_ */
