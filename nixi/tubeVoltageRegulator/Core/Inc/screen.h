/*
 * screen.h
 *
 *  Created on: Jan 2, 2022
 *      Author: Brigitte
 */

#include<stdint.h>

#ifndef INC_SCREEN_H_
#define INC_SCREEN_H_


void screenS1msTimer ();

void paintCanScreen();
void initScreen();
void setDebugScreenJob();

void displayFatalError();

#endif /* INC_SCREEN_H_ */
