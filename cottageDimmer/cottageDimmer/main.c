/*
 * cottageDimmer.c
 *
 * Created: 2/7/2025 16:14:16
 * Author : diego
 */ 

#include <avr/io.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include <avr/eeprom.h>

void setupHw()
{
	cli();
	MCUCR |= (1 << PUD);
	DDRB &= ~(1 << DDB0);
	MCUCR |= (1 << ISC00);
	MCUCR &= ~(1 << ISC01);
	GIMSK |= (1 << INT0);
	sei();
}

void startDimmerSearch()
{
	
	
}

void stopDimmerSearch()
{
	
}

ISR(PCINT0_vect) 
{
	if ((PINB & (1 << PINB0 )) != 0 ) {
		startDimmerSearch();
	} else {
		stopDimmerSearch();
	}
	
}

int main(void)
{
    setupHw();
    while (1) 
    {
    }
}

