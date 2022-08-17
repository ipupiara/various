
//#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stm32f1xx_it.h>

#include "TriacIntr.h"
#include "TriacDefines.h"
#include <stm32f1xx.h>


#include <main.h>

//#include "triacPID.h"

//int16_t remainingTriacTriggerDelayCounts;
//
//int16_t amtInductiveRepetitions;
//
//int16_t inductiveRepetitionsCounter;
//
//int8_t withinZeroCross;
//
//void enaRXIntUsart1();
//
//void disaRXIntUsart1();
//
//void initOutUart0();

uint8_t durationTimerReachead;

//uint8_t durationTimerReachedTwo;
uint16_t secondsRemainingInDurationTimer;
uint16_t secondsInDurationTimer;
//uint16_t triacFireDurationTcnt;   // centi-millis-secs, not exactly but approximate, PID will handle the rest
int8_t heaterRelaisOn;
int8_t ventiRelaisOn;

uint8_t minutesCounter;
uint8_t secondsCounter;
uint16_t hoursCounter;


void initRelais();

void controlTemperature(float* temp);

int16_t getSecondsRemainingInDurationTimer()
{
	int16_t res;
	cli();
	res = secondsRemainingInDurationTimer;
	sei();
	return res;
}

int16_t getSecondsInDurationTimer()
{
	int16_t res;
	cli();
	res = secondsInDurationTimer;
	sei();
	return res;
}


void startTriacTimer()
{
#ifdef useTimer0	
	if (withinZeroCross == 0) {
		TIMSK   |=  (1<<OCIE0) ;  //  Output Compare A Match Interrupt Enable
		TCCR0 |=  (1 <<  CS00) |  (1 <<  CS02) ;    //    set prescaler 128
	}
	TIFR  &= ~(1 << OCF0);    // clear interrupt flags if it should ever have been set by any reason...
#endif	

#ifdef useTimer3
	if (withinZeroCross == 0) {
		ETIMSK   |=  (1<<OCIE3A) ;  //  Output Compare A Match Interrupt Enable
		TCCR3B |=  (1 <<  CS30) |  (1 <<  CS31) ;    //    set prescaler 64
	}
	ETIFR  &= ~(1 << OCF3A);    // clear interrupt flags if it should ever have been set by any reason...
#endif

#ifdef useAtmega644PTimer2
	TIMSK2   = 0b00000010;  //  Output Compare A Match Interrupt Enable
	TCCR2B = 0b00000101  ; // CTC on CC2A , set clk / 128, timer 2 started
	TIFR2 = 0x00;
#endif

}

void stopTriacTimer()
{	
#ifdef useTimer0	
	TCCR0 &=  ~((1 <<  CS00) |  (1 <<  CS01) | (1 <<  CS02)) ;     // (0 prescaler )  timer stopped
	TIMSK   &=  ~(1<<OCIE0) ;  //  Output Compare A Match Interrupt disable
	TIFR  &= ~(1 << OCF0);		// clear interrupt flags
#endif	

#ifdef useTimer3
	TCCR3B &=  ~((1 <<  CS30) |  (1 <<  CS31) | (1 <<  CS32)) ;     // (0 prescaler )  timer stopped
	ETIMSK   &=  ~(1<<OCIE3A) ;  //  Output Compare A Match Interrupt disable
	ETIFR  &= ~(1 << OCF3A);		// clear interrupt flags
#endif

#ifdef useAtmega644PTimer2
	TCCR2B = 0b00000000  ;  // CTC, timer stopped
	TIMSK2  = 0x00;
	TIFR2 = (1<< OCF2A);    // cleared by writing a "logic" one to the flag
#endif
}

void resetTriacTimerFlag()
{
#ifdef useTimer0
	TIFR  &= ~(1 << OCF0);		// clear interrupt flags
#endif

#ifdef useTimer3
	ETIFR  &= ~(1 << OCF3A);		// clear interrupt flags
#endif

#ifdef useAtmega644PTimer2
	TIFR2 = (1<< OCF2A);    // cleared by writing a "logic" one to the flag
	#warning "tobe tested useAtmega644PTimer2 in resetTriacTimerFlag"
#endif

}


void setOcrDelay(int16_t newOcr)
{
	// timer0 must be stopped before running this method (was the case per 17 Feb 2017
	// timer must be stopped to set tcnt, because else, on an 
	// unprotected set, the timer itself could interfere with the *non double buffered feature" write access.
	// resulting in a more or less randomly set value.
//	stopTimer0();
	if (newOcr <= 1) { newOcr = 2; }  // .... updating avoids triggering of next clock cycle, but needs overnext.
#ifdef useTimer0		
	TCNT0 = 0;
	OCR0 = newOcr; 
#endif
#ifdef useTimer3
	TCNT3 = 0;
	OCR3A = newOcr;
#endif	 

#ifdef useAtmega644PTimer2
	OCR2A =	newOcr;
	TCCR2B = 0  ; 
#endif
}

void setTriacTriggerDelayValues(uint8_t lohi)
{
	//if (lohi == 1) {
		//setOcrDelay( 5);
		//remainingTriacTriggerDelayCounts -= 10;
	//} else {
		//if (remainingTriacTriggerDelayCounts <= triacOcrValueMax) {
			//setOcrDelay ( remainingTriacTriggerDelayCounts);
			//remainingTriacTriggerDelayCounts = 0;
			//} else {
			//setOcrDelay( triacOcrValueMax);
			//remainingTriacTriggerDelayCounts -= triacOcrValueMax;
		//}
	//}
}


void startTriacTriggerDelay( int16_t delayDuration)  
{
	//uint8_t  lohi = 0;
	//cli();
	//stopTriacTimer();
	//if (delayDuration <= 0) { 
		//delayDuration = 1;   // just a very short duration, but one that will happen in future
	//}
	//if (delayDuration > delayBetweenTriacTriggers)  {
		//lohi = 1;
	//}
	//remainingTriacTriggerDelayCounts = delayDuration;
	//setTriacTriggerDelayValues(lohi);	
	//startTriacTimer();
	//resetTriacTimerFlag();
	//sei();
}

void calcAmtInductiveRepetitions(int16_t tFDurationTcnt0)
{
	//if ( inductiveLoad == 1)  {
		//floatType amtInductiveRepetitionsF = 0.0;
		//floatType tFDurationTcnt0F = tFDurationTcnt0;
		////		amtInductiveRepetitions = ((triacFireDurationTcnt2 * ( 1  /(11.0592e+6  /128) )) * 1.0e+6  ) /  measuredRepetitionIntervalus;
////		float tcnt0TickDurationUs  = 128 / 11.0592e+6; 
////		float calculatedRepetitionInterval = delayBetweenTriacTriggers * tcnt0TickDurationUs + 5;   // trigger takes approx 3- 5us
////                  measured was 200  and calculated was 190 what is quite ok. measured should be a bit higher than calculated 
////                  for all the loss during stop and start of timer  , etc.....
		//amtInductiveRepetitionsF = (tFDurationTcnt0F * 11.63  )  /  measuredRepetitionIntervalus;
		//// always cut off modulo part when converting to int
		//amtInductiveRepetitions = amtInductiveRepetitionsF;   
	//} else {
		//amtInductiveRepetitions = 1;
	//}
}

void setTriacFireDuration(uint16_t durationTcnt)
{
	//uint16_t  tfDuration;
	//
	//if (durationTcnt < triggerDelayMaxTcnt) {
		//if (durationTcnt > 0) {
			//tfDuration = durationTcnt;}
		//else {
			//tfDuration = 0;
		//}
	//} else {
		//tfDuration = triggerDelayMaxTcnt;
	//}
	//calcAmtInductiveRepetitions(durationTcnt);
	//cli();
	//triacFireDurationTcnt = tfDuration;
	//sei();
}

//uint16_t  getTriacFireDuration()
//{
//	uint16_t res;
//	cli();
//		res =  triacFireDurationTcnt;
//	sei();
//	return res;
//}



//ISR( triacCompVect )
//{
////#if  defined( useTimer3) || defined( useTimer0)	
	////if (TCNT3 != 0) {
		////DDRA |= (1<< DDRA0);
		////PORTA  |= (1<<PORTA0);
	////}
////#endif	
////
	////cli();
	////if (remainingTriacTriggerDelayCounts <= 0) {
		////PORTE |= (1<< PORTE6) ;
		////sei();				// allow interrupts during delay
		////delay6pnt2d5us(triacTriggerLength);   // approx 5 us of triac trigger , try later half or even less, measured 7 with oscilloscope
		////cli();
		////PORTE &= ~(1<< PORTE6) ;		// handled synchronous
		////if  ((inductiveRepetitionsCounter <= 0) || (withinZeroCross == 1) ) {
			////stopTriacTimer();
		////} else {
			////--inductiveRepetitionsCounter;
			////startTriacTriggerDelay(delayBetweenTriacTriggers);  // already start interrupts and timer0
		////}
	////} else {
		////stopTriacTimer();
		////setTriacTriggerDelayValues(0);
		////startTriacTimer();
	////}
	////sei();
//}

#warning "0-x signal is currently compared on 311 comparator at VCC, which is an unprecise common voltage value, should be changed to a lower voltage range"
#if defined( useTimer3) || defined( useTimer0)
// int7 on port PE7
ISR(INT7_vect)
{
	cli();
	EIFR  &=  ~(1 << INTF7);						// clear interrupt flag
	if ((PINE & (1 << PINE7)) != 0) {				//  on	high level
		EICRB &=  ~((1<< ISC70) |  (1<< ISC71))  ; 
		EICRB  |=   (1<< ISC71)  ;                  // falling edge trigger 
		EIFR  &=  ~(1 << INTF7);					// clear interrupt flag			
		withinZeroCross = 0;
		
		DDRA |= (1<< DDRA0);
		PORTA   &= ~ (1<<PORTA0);
			
		if (triacFireDurationTcnt > 0)  {
			inductiveRepetitionsCounter = amtInductiveRepetitions;
			startTriacTriggerDelay(  triggerDelayMaxTcnt - triacFireDurationTcnt);  // does sei and startTimer0
		}	
		
	} else {										// on low level
		EICRB &=  ~((1<< ISC70) |  (1<< ISC71))  ; 
		EICRB  |=  (1<< ISC70) |  (1<< ISC71)  ;	// raising up trigger
		EIFR  &=  ~(1 << INTF7);					// clear interrupt flag  
		withinZeroCross = 1;
		stopTriacTimer();
	}
	sei();		  
}   
#elif defined (useAtmega644PTimer2)
ISR(INT0_vect)
{
	EIFR  &=  ~(1 << INTF7);
	cli();
	if ((PIND & 0x04) != 0) {
		withinZeroCross = 1;
		stopTriacTimer();
	} else {
		withinZeroCross = 0;
		
//		DDRA |= (1<< DDRA0);
//		PORTA   &= ~ (1<<PORTA0);
		
		if (triacFireDurationTcnt > 0)  {
			inductiveRepetitionsCounter = amtInductiveRepetitions;
			startTriacTriggerDelay(  triggerDelayMaxTcnt - triacFireDurationTcnt);  // does sei and startTimer0
		}
	}
	sei();
}


ISR(INT7_vect)
{
	cli();
	EIFR  &=  ~(1 << INTF7);						// clear interrupt flag
	if ((PINE & (1 << PINE7)) != 0) {				//  on	high level
		EICRB &=  ~((1<< ISC70) |  (1<< ISC71))  ;
		EICRB  |=   (1<< ISC71)  ;                  // falling edge trigger
		EIFR  &=  ~(1 << INTF7);					// clear interrupt flag
		withinZeroCross = 0;
		
		DDRA |= (1<< DDRA0);
		PORTA   &= ~ (1<<PORTA0);
		
		if (triacFireDurationTcnt > 0)  {
			inductiveRepetitionsCounter = amtInductiveRepetitions;
			startTriacTriggerDelay(  triggerDelayMaxTcnt - triacFireDurationTcnt);  // does sei and startTimer0
		}
		
		} else {										// on low level
		EICRB &=  ~((1<< ISC70) |  (1<< ISC71))  ;
		EICRB  |=  (1<< ISC70) |  (1<< ISC71)  ;	// raising up trigger
		EIFR  &=  ~(1 << INTF7);					// clear interrupt flag
		withinZeroCross = 1;
		stopTriacTimer();
	}
	sei();
}
#endif

void getTimeValues(uint16_t* hrs, uint8_t* mins, uint8_t* secs)
{
	cli();
	*hrs = hoursCounter;
	*mins = minutesCounter;
	*secs = secondsCounter;
	sei();
}


  
uint32_t overallSeconds() 
{
	uint32_t res = 0;
	GETTimeValues
	res = hrs * 3600 + mins * 60 + secs;
	return res;
}

//ISR(TIMER1_COMPA_vect)
void secTimer()
{
	if (secondsCounter == 59) { 
		++ minutesCounter;
		secondsCounter = 0;
		if (minutesCounter == 59) {
			++ hoursCounter;
			minutesCounter = 0;
		}
	} else { ++ secondsCounter; }
	
	if (secondsRemainingInDurationTimer > 0) {
		secondsRemainingInDurationTimer --;
		secondsInDurationTimer ++;
//		if (secondsInDurationTimer == 2) {
//			durationTimerReachedTwo = 1;
//		}
		if (secondsRemainingInDurationTimer == 0) {
			stopDurationTimer();
			durationTimerReachead = 1;
		} 
	}

}

void initInterruptsNValues()
{
	
///////   0-x detector input pint INT7 on PE7  
		
		//DDRE  &=  ~(1 << DDRE7 ) ;    // input pin
		//DDRE |=  (1 << DDRE6);		// ouput pin, triac trigger
//
		//EICRB   |=  (1<< ISC70) |  (1<< ISC71)  ;   // rising edge  ( each time, edge needs to be programmed in the interrupt ::::----(((((
		//EIFR  &=  ~(1 << INTF7); 
		//EIMSK |= (1 << INT7);

// Timer 1 as Duration Timer
	  


		//  use system sec timer
	  
//		TCCR1A = 0x00;  // normal mode or CTC dep.on TCCR1B
//		//TCCR1B = 0b00001101  ; // CTC on CC1A , set clk / 1024, timer started
//
//		TCCR1B = 0b00001000  ;  // CTC, timer stopped
//
//		TCCR1C = 0x00; // no Force output compare
//
//		OCR1A = 0x2A30;  // counter top value  , this value at clk/1024 will cause a delay of exact 1 sec at 11.0592 hz
//		TCNT1 = 0x00 ;
//
//
////		TIMSK  &=  ~(1 << OCIE1A)  ;// disa  Interrupt    since timsk in atmega128 is global for 3 timers
////			TIMSK1   = 0b00000010;  //  Output Compare A Match Interrupt Enable
//			TIMSK1 |= (1 << OCIE1A)  ;  //  Output Compare A Match Interrupt Enable

		secondsRemainingInDurationTimer = 0;

// Timer 0 as Triac Trigger Delay Timer
#ifdef useTimer0	  
		TCCR0 = (1 << WGM01) ;  // CTC,
		TCCR0 &=  ~((1 <<  CS00) |  (1 <<  CS01) | (1 <<  CS02)) ;     // (0 prescaler )  timer stopped
	  


		OCR0 = triacOcrValueMax;  // counter top value  , just anything for start, will later be set by PID
		TCNT0 = 0x00 ;  
		ASSR = 0x00;
		//		TIMSK2  = 0x00; // disa  Interrupt 
		TIMSK   |=  (1<<OCIE0) ;  //  Output Compare A Match Interrupt Enable 
#endif

#ifdef useTimer3
		TCCR3A = 0x00;  // normal mode or CTC dep.on TCCR1B
		TCCR3B = (1 << WGM32);  //       0b00001000  ;  // CTC, timer stopped
		TCCR3C = 0x00; // no Force output compare

		OCR3A = triacOcrValueMax;  // counter top value  
		TCNT3 = 0x00 ;
	
		TCCR3B &=  ~((1 <<  CS30) |  (1 <<  CS31) | (1 <<  CS32)) ;     // (0 prescaler )  timer stopped
		
		ETIMSK   &=  ~(1<<OCIE3A) ;  //  Output Compare A Match Interrupt Enable
#endif

#ifdef useAtmega644PTimer2
		TCCR2A = 0b00000010;  //  CTC

		//TCCR2B = 0b00000101  ; // CTC on CC0A , set clk / 128, timer started

		TCCR2B = 0b00000000  ;  // CTC, timer stopped
		ASSR = 0x00;

		OCR2A = triacOcrValueMax;  // counter top value  , just anything for start, will later be set by PID
		TCNT2 = 0x00 ;

		TIMSK2  = 0x00; // disa  Interrupt
		//		TIMSK2   = 0b00000010;  //  Output Compare A Match Interrupt Enable

#endif

		sei();  // start interrupts if not yet started
}

//void startTriacRun()
//{
////	resetPID();
////	startAmpsADC();
//	EIFR = 0x00;
//	EIMSK = 0x01;  				// start external interrupt (zero pass detection)
//}
//
//void stopTriacRun()
//{
//	cli();
//	EIMSK = 0x00;				// stop external interrupt
//	stopTriacTimer();
//	sei();
////	stopAmpsADC();
//}


/*
int16_t  valueFrom6Bit2Complement(int16_t adcV)
{
	if (adcV & 0x0200) {
		adcV--;
		adcV = ~(adcV | 0xFC00);
		adcV = - adcV;
	}
	return adcV;
}

int16_t diffADCValue()
{  
	int16_t res;
	res = ampsADCValue();
	res = valueFrom6Bit2Complement(res);
	return res;
}
*/

void startSecondTick()
{
//	TIMSK1   = 0b00000010;  //  Output Compare A Match Interrupt Enable
//	TCCR1B = 0b00001101  ; // CTC on CC1A , set clk / 24, timer started
	secondsCounter = 0;
	minutesCounter = 0;
	hoursCounter = 0;
}

void startDurationTimer(int16_t secs)
{
	cli();   // may not be interrupted by secondTick
	durationTimerReachead = 0;
	secondsRemainingInDurationTimer = secs;
	secondsInDurationTimer = 0;
	sei();
	
//	TIMSK1   = 0b00000010;  //  Output Compare A Match Interrupt Enable 
//	TCCR1B = 0b00001101  ; // CTC on CC1A , set clk / 24, timer started 
}

void stopDurationTimer()
{
//	TCCR1B = 0b00001000 ;  // CTC, timer stopped
//	TIMSK1 = 0x00;	
}

/*
void setCompletionAlarmOff()
{
	PORTD &= ~0x08; 		
}

void setCompletionAlarmOn()
{
	PORTD |= 0x08; 	
}

void toggleCompletionAlarm()
{  
	if (PORTD & 0x08) {
		setCompletionAlarmOff();
	} else {
		setCompletionAlarmOn();
	}
}
*/



void initHW()
{
//#ifndef UseStdOutForUsart0
//	initOutUart0();
//#endif
//
	initInterruptsNValues();
//	initUsart1();
//	startSecondTick();
	initRelais();

    fatalErrorOccurred = 0;
	
}




////////////////// start heating , ventilating  methods ////////////////////
 
// #define relais1PortDDR  DDRB
// #define relais1PinDDR   DDB0
// #define ventiRrelais1Port
//PORTB
// #define relais1Pin     avr code
//PB0

#define heaterPin_Pin GPIO_PIN_15
#define heaterPin_GPIO_Port GPIOC
#define ventiPin_Pin GPIO_PIN_9
#define ventiPin_GPIO_Port GPIOB

 void switchHeaterRelais(uint8_t relaisNeedsOn)
 {
	 if (relaisNeedsOn > 0) {
		 HAL_GPIO_WritePin(heaterPin_GPIO_Port, heaterPin_Pin, GPIO_PIN_SET);
		 heaterRelaisOn= 1;
	 }  else   {
		 HAL_GPIO_WritePin(heaterPin_GPIO_Port, heaterPin_Pin, GPIO_PIN_RESET);
		 heaterRelaisOn = 0;
	 }
 }

 void  switchVentiRelais(uint8_t relaisNeedsOn)
 {
	 if (relaisNeedsOn > 0) {
		 HAL_GPIO_WritePin(ventiPin_GPIO_Port, ventiPin_Pin, GPIO_PIN_SET);
		 ventiRelaisOn= 1;
	 }  else   {
		 HAL_GPIO_WritePin(ventiPin_GPIO_Port, ventiPin_Pin, GPIO_PIN_RESET);
		 ventiRelaisOn = 0;
	 }
 }


 void testRelais()
 {
	 uint8_t cnt;
	 for (cnt = 0; cnt < 3; ++ cnt)  {
		 switchHeaterRelais(1);
		 switchVentiRelais(1);
		 switchVentiRelais(0);
		 switchHeaterRelais(0);
	 }
 }


 void initRelais()
 {
	 __HAL_RCC_GPIOB_CLK_ENABLE();
	 __HAL_RCC_GPIOC_CLK_ENABLE();
	 GPIO_InitTypeDef GPIO_InitStruct = {0};

	   HAL_GPIO_WritePin(heaterPin_GPIO_Port, heaterPin_Pin, GPIO_PIN_RESET);
	   GPIO_InitStruct.Pin = heaterPin_Pin;
	   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	   GPIO_InitStruct.Pull = GPIO_NOPULL;
	   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	   HAL_GPIO_Init(heaterPin_GPIO_Port, &GPIO_InitStruct);
	   heaterRelaisOn = 0;


	   HAL_GPIO_WritePin(ventiPin_GPIO_Port, ventiPin_Pin, GPIO_PIN_RESET);
	   GPIO_InitStruct.Pin = ventiPin_Pin;
	   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	   GPIO_InitStruct.Pull = GPIO_NOPULL;
	   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	   HAL_GPIO_Init(ventiPin_GPIO_Port, &GPIO_InitStruct);
	   ventiRelaisOn = 0;

//	   testRelais();
 }


 void controlTemperature(float* temp)
 {
	 if (*temp < HeatingLowerLimit)  {
		 switchHeaterRelais(1);
	 }
	 if (*temp > HeatingUpperLimit)  {
		 switchHeaterRelais(0);
	 }
 }

void startHumidifying()
{

}

void stopHumidifying()
{
	
}

void startVentilator(uint8_t  on)
{
	if (on == 1)   {
		switchVentiRelais(1);
//		 while (1){} //  was used to test watchdog, so that something sounds a bit, worked well with asm code in "wdt_enable(val );"
	}  else
	{
		switchVentiRelais(0);
	}
}

void startVentilating()
{
	switchVentiRelais(1);
}

void stopVentilating()
{
	switchHeaterRelais(0);
}

void startHeating()
{
	switchHeaterRelais(1);
}

void stopHeating()
{
	switchVentiRelais(0);
}


void startDrying()
{
	switchVentiRelais(1);
}

void stopDrying()
{
	switchVentiRelais(0);
}



