

// TriacDefines.h contains global and some local defines, but not all of the local defines
// also it contains many of the global data definition and their persistent storage on eeprom

// Yoar Attention please, yoar attention: set Brown out level on 2.7V at least
// otherwise errors on the eeprom could be observed at startup

#ifndef TriacDefines_H
	#define TriacDefines_H


	#include <inttypes.h>

//	#define UseStdOutForUsart0 
	#define JT_F_CPU 11095200	
	
	#define printCsvData     //  if printCsvValues is set, only csv values must be printed out
	
	#define maxSecsPossible  0xFFFF
	
	
	#define IdleVentilationDelayMinutes     17
	#define IdleVentilationMinutes     3

	#define HumidifyingLowerLimit   80.0
	#define HumidifyingUpperLimit  85.0
	#define DryingUpperLimit	99.5
	#define DryingLowerLimit	99.0


	#define HeatingLowerLimit	27.0
	#define HeatingUpperLimit	27.5

//	#define HeatingLowerLimit	23.75
//	#define HeatingUpperLimit	24.25

	#define CoolingLowerLimit	29.0
	#define CoolingUpperLimit	30.0

	
//	#define useTimer0
//	#define useTimer3
	
	#ifdef useTimer0
		#define triggerDelayMaxTcnt 810     // works properly with 50 Hz Ac and 11,0592 Mhz	
		#define inductiveLoad   1      // to be set to 0 or 1
		#define triacTriggerLength   0      // delay approx ( n * 2.5 us ) + 5.5    at 11.0592  mhz
		#define delayBetweenTriacTriggers  16  //  in timer 2 tcnt values , prescaler at   128
		#define measuredRepetitionIntervalus  200;      //  approx   ( 1s  /(11.0592 E+6  /128) ) * delayBetweenTriacTriggers    [us]
		#define triacOcrValueMax 0XFC  // still to be defined
		#define triacCompVect  TIMER0_COMP_vect
	#endif
	
	#ifdef useTimer3
		#define triggerDelayMaxTcnt 1620     // works properly with 50 Hz Ac and 11 Mhz
		#define inductiveLoad   1      // to be set to 0 or 1
		#define triacTriggerLength   0      // delay approx ( n * 2.5 us ) + 5.5    at 11.0592  mhz
		#define delayBetweenTriacTriggers  32  //  in timer 2 tcnt values , prescaler at   128
		#define triacOcrValueMax  triggerDelayMaxTcnt   
		#define measuredRepetitionIntervalus  200;      //  approx   ( 1s  /(11.0592 E+6  /128) ) * delayBetweenTriacTriggers    [us]
		#define triacCompVect TIMER3_COMPA_vect 
	#endif
	
	#ifdef useAtmega644PTimer2
		#define triggerDelayMaxTcnt 810     // works properly with 50 Hz Ac and 11 Mhz
		#define inductiveLoad   1      // to be set to 0 or 1
		#define triacTriggerLength   0      // delay approx ( n * 2.5 us ) + 5.5    at 11.0592  mhz
		#define delayBetweenTriacTriggers  16  //  in timer 2 tcnt values , prescaler at   128
		#define measuredRepetitionIntervalus  200;      //  approx   ( 1s  /(11.0592 E+6  /128) ) * delayBetweenTriacTriggers    [us]
		#define triacOcrValueMax 0XFC  // still to be defined
		#define triacCompVect  TIMER2_COMPA_vect
	#endif
	
	//#define useAtmega324a
	#ifdef  useAtmega324a
	
		#define  DDA0  DDRA0
		#define  DDA1  DDRA1
		#define  DDA2  DDRA2
		#define  DDA3  DDRA3
		#define  DDA4  DDRA4
		#define  DDA5  DDRA5
		#define  DDA6  DDRA6
		#define  DDA7  DDRA7
		
		#define  DDB0  DDRB0
		#define  DDB1  DDRB1
		#define  DDB2  DDRB2
		#define  DDB3  DDRB3
		#define  DDB4  DDRB4
		#define  DDB5  DDRB5
		#define  DDB6  DDRB6
		#define  DDB7  DDRB7
		
		#define  DDC0  DDRC0
		#define  DDC1  DDRC1
		#define  DDC2  DDRC2
		#define  DDC3  DDRC3
		#define  DDC4  DDRC4
		#define  DDC5  DDRC5
		#define  DDC6  DDRC6
		#define  DDC7  DDRC7
		
		#define  DDD0  DDRD0
		#define  DDD1  DDRD1
		#define  DDD2  DDRD2
		#define  DDD3  DDRD3
		#define  DDD4  DDRD4
		#define  DDD5  DDRD5
		#define  DDD6  DDRD6
		#define  DDD7  DDRD7
		
	
	#endif
	
	
	
typedef float floatType;	

// takes approx  2 us at 10 Mhz with -o0 (no opts. !) ,with all  call/ret, push N Pop

void info_printf(const char *emsg, ...);
void csv_printf(const char *emsg, ...);

//void delayEmptyProc ();

//uint8_t  hygrosenseMsgCnt;

//void delay6pnt2d5us(unsigned int n);   // description in TriacDefines.c

extern char lastFatalErrorString [20];
extern int8_t fatalErrorOccurred;

//extern uint8_t durationTimerReachead;
//extern uint8_t runningSecondsTick;
//extern uint8_t durationTimerReachedTwo;
//extern uint16_t secondsRemainingInDurationTimer;
//extern uint16_t secondsInDurationTimer;
//
////uint16_t triacFireDurationTcnt;   // centi-millis-secs, not exactly but approximate, PID will handle the rest
//extern int8_t relais1On;


floatType  GetIdleVentilationDelayMinutes();
floatType  GetIdleVentilationMinutes();
floatType  GetHumidifyingLowerLimit();
floatType  GetHumidifyingUpperLimit();
floatType  GetDryingUpperLimit();
floatType  GetDryingLowerLimit();

void initDefines();

//#define amtOnOffIntervalsPerTimerPort   4
//#define amtOnOffTimerPorts				2       // light and fictive intofridge-ventilation,
													// vs. intoGrowbox-ventilation which is controlled via hygrosense 


//typedef struct TimeClock   {
//	uint8_t hour;
//	uint8_t minute;
//} TimeClock;
//
//
//typedef struct OnOffInterval {
//	TimeClock onClock;
//	TimeClock offClock;
//} OnOffInterval;
//
//
//typedef struct OnOffTimerPorts  {
//	uint8_t cpuPort;
//	uint8_t cpuPin;
//	OnOffInterval onOffIntervals [amtOnOffIntervalsPerTimerPort];
//} OnOffTimerPorts;
//
//uint8_t  isCurrentlyOn(TimeClock now, uint8_t timerPort);
//
//
////void sendTWIDataRequest();
////void onTWIDataReceived();
//void setClockToTime(uint8_t hours, uint8_t minutes);

#endif
