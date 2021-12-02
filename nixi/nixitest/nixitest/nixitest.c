/*
 * nixitest.c
 *
 * Created: 02.12.2021 16:16:15
 *  Author: mira
 */ 
#include <avr/io.h>
#include <stdio.h>
#include <string.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

#include <avr/io.h>

enum {
	none,
	overUp,
	underLow
};

int16_t lastADCVal;

uint8_t  overUpper;
uint8_t  underLower;


void initADC()
{
	overUpper = none;
	underLower = none;
	lastADCVal = 0;
	ADMUX = (1 <<  MUX0) ;   //  pb2 port (= ADC1), Vcc ref, right adjust
	ADCSRB = 0x00;  //  free running mode, unipolar, default polarity
	DIDR0 = (1 << ADC0D);   // disable digital input on adc0
	ADCSRA =  (1 << ADEN) | (1<< ADATE) | (1<< ADIE)  | (1 << ADSC) | (1<< ADPS0) | (1<< ADPS1)| (1<< ADPS2);  
							//  enable adc, enable auto trigger, enable interrupts, start conversion, 128 prescaer ( approx 62,5 kHz adc clk) with 8 Mhz clk
}

void initPWM()
{
	
}

void startPWM()
{
	cli();
	underLower = none;
	sei();
}

void stopPWM()
{
	cli();
	overUpper = none;
	sei();
}

uint8_t  getUnderLower()
{
	uint8_t res;
	cli();
	res = underLower;
	sei();
	return res;
}

uint8_t  getOverUpper()
{
	uint8_t res;
	cli();
	res = overUpper;
	sei();
	return res;
}

void initHW()
{
	cli();
	initPWM();  //  start pwm first since adc uses pwm  (though anyhow interrupts should be disabled..)
	initADC();
	sei();
}

ISR (ADC_vect)
{
	cli();
	lastADCVal = ADC;
	sei();
}


int main(void)
{
	initHW();
    while(1)
    {
        if (getOverUpper() == overUp)  {     // atomic access due to 8-bit value access in single cycle 
			stopPWM();
		}
		if (getUnderLower() == underLow)  {
			startPWM();
		}
			
    }
}