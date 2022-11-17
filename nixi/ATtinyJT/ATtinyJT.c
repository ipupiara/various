#include <avr/io.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <avr/eeprom.h>
#include "ATtinyjt.h"
#include "USI_TWI_Slave.h"


int8_t runningSecondsTick;
uint8_t secCounter;



/*

void debugLightOn()
{
	PORTA |= 0x80;
}

void debugLightOff()
{
	PORTA &= ~0x80;
}


void debugLightToggle()
{
	if (PORTA & 0x80) {
		PORTA &= ~0x80;
	} else {
		PORTA |= 0x80;
	}
}
*/



ISR(TIM1_COMPA_vect)
{
		runningSecondsTick = 1;
}



void initHW()
{
	runningSecondsTick = 0;
	secCounter = 0;
	jobBuffer = 0;

	cli();
// Timer 1 as Duration Timer

	
	TCCR1A = 0x00;  // normal mode , CTC dep.on TCCR1B
	TCCR1B = (0x00 | (1<<WGM12));  //  CTC, timer still stopped
	TCCR1C = 0x00; // no Force output compare
	OCR1A = 0x7A12;  // counter top value  , this value at clk/256 will cause a delay of exact 1 sec
	TCNT1 = 0x00 ;
	 
//	TIMSK1 = 0x00; 
	TIMSK1   =  (0x00 | (1<<OCIE1A));  //  Output Compare A Match Interrupt Enable 

	TCCR1B |= (1<<CS12); // prescaler clk / 256, timer started

// adc settings
	

	sei();
//	DDRA |= 0x80; // debuglight enable
//	PORTA &= ~0x80; // debuglight off
}


void setRelaisOn()
{
	
}

void setRelaisOff()
{
	
}


void onSecondTick()
{	
	++ secCounter;
	if (secCounter >  60)  {
		setRelaisOff();
	}
	if (secCounter > 100)   {
		setRelaisOn();
		secCounter = 0;
	}
}


void jobReceived(int8_t jS)
{
	secCounter = 0;
}



int main(void)
{
//	initPotiJob();   // needs to be called before initHW();

	initHW();
	USI_TWI_Slave_Initialise(0x10);

	while(1) {
		int8_t jobB;
		cli();
		if (jobBuffer != 0) {
			jobB = jobBuffer;
			jobBuffer = 0;
		}
		sei();
		if (jobB != 0) {
			jobReceived(jobB);
			jobB = 0;
		}

//		asm volatile ( "wdr"  );       // some watchdog code, maybe used later somewhen

		if (runningSecondsTick == 1) {
			runningSecondsTick = 0;
				onSecondTick();	
		}


	}
}
