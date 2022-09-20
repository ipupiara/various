/*
 * screen.h
 *
 *  Created on: Jan 2, 2022
 *      Author: Brigitte
 */

#include<stdint.h>

#ifndef INC_SCREEN_H_
#define INC_SCREEN_H_

void setStateName(uint8_t * stName);
void appendStateName(uint8_t* stName);
void screenCentiSecTimer ();

void paintCanScreen();
void initScreen();
void setDebugScreenJob();

void setGrowboxScreen();

void displayFatalError();

#endif /* INC_SCREEN_H_ */
