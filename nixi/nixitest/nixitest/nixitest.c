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

#define adcLowValue  0x1df
#define adcHighValue 0x21f

enum {
	adcIdle,
	adcRunning,
	adcOverUpNeedsStop,
	adcUnderLowNeedsRunning
};

int16_t lastADCVal;

uint8_t  adcState;

void initADC()
{
	adcState = adcIdle;
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
	adcState = adcRunning;
	sei();
}

void stopPWM()
{
	cli();
	adcState = adcIdle;
	sei();
}

uint8_t  getAdcState()
{
	uint8_t res;
	cli();
	res = adcState;
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
	if ((lastADCVal > adcHighValue) && (adcState != adcIdle))  {
		adcState = adcOverUpNeedsStop;
	} else if ((lastADCVal < adcLowValue) & (adcState != adcRunning))  {
		adcState = adcUnderLowNeedsRunning;
	}
	sei();
}


int main(void)
{
	initHW();
    while(1)
    {
        if (getAdcState() == adcOverUpNeedsStop)  {     // atomic access due to 8-bit value access in single cycle 
			stopPWM();
		}
		if (getAdcState() == adcUnderLowNeedsRunning)  {
			startPWM();
		}
			
    }
}