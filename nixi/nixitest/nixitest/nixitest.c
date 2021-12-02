/*
 * nixitest.c
 *
 * Created: 02.12.2021 16:16:15
 *  Author: mira
 */ 


#include <avr/io.h>

enum {
	none,
	overUp,
	underLow
};


uint8_t  overUpper;
uint8_t  underLower;


void initADC()
{
	
}

void initPWM()
{
	
}

void startPWM()
{
	underLower = none;
}

void stopPWM()
{
	overUpper = none;
}

void initHW()
{
	initADC();
	initPWM();
}


int main(void)
{
	initHW();
    while(1)
    {
        if (overUpper == overUp)  {
			stopPWM();
		}
		if (underLower == underLow)  {
			startPWM();
		}
			
    }
}