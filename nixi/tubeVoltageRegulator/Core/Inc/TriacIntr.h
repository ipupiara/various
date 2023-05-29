

#ifndef TriacIntr_H
	#define TriacIntr_H


#include "TriacDefines.h"



//#define adcRefVoltage5   5.0
//#define adcRefVoltad2d5  2.56

#define GETTimeValues  uint16_t hrs; uint8_t mins; uint8_t secs; getTimeValues(&hrs,&mins,&secs);

extern uint8_t durationTimerReachead;
extern uint8_t runningSecondsTick;
//extern uint8_t durationTimerReachedTwo;

extern uint16_t secondsRemainingInDurationTimer;

extern uint16_t secondsInDurationTimer;

//uint16_t triacFireDurationTcnt;   // centi-millis-secs, not exactly but approximate, PID will handle the rest
extern int8_t heaterRelaisOn;

extern uint8_t minutesCounter;
extern uint8_t secondsCounter;
extern uint16_t hoursCounter;

void secTimer();

void startDurationTimer(int16_t secs);
int16_t getSecondsRemainingInDurationTimer();
int16_t getSecondsInDurationTimer();

void stopDurationTimer();

void initHW();
//void setTriacFireDuration(uint16_t cmsecs);
//uint16_t  getTriacFireDuration();
//void startTriacRun();
//void stopTriacRun();

heatLevelEnum getHeatLevelFromPin();

void startHeating();
void stopHeating();

void getLatestClimateValues(floatType* pTemp,floatType* pHum);    // interface to hygrosense, called by user functions
floatType getCurrentTemperature();
floatType getCurrentHumidity();
floatType getCurrentTemperatureVoltage();


//int8_t adcTick;




//void initADC();
//uint16_t adcValue(uint8_t pos);
//floatType adcVoltage(uint8_t  pos);
//uint8_t getADCTemperature(uint8_t  pos, floatType* result);
//void startADCSequence();
//int8_t startNextADC ();
//int16_t getTriacFireDurationFromADC(uint8_t pos);
//uint16_t getLastAdcValue(uint8_t  pos);
void startSecondTick();
int16_t getSecondsDurationTimerRemaining();
int16_t getSecondsRemainingInDurationTimer();
uint32_t overallSeconds();
void getTimeValues(uint16_t* hrs, uint8_t* mins, uint8_t* secs);

void startHumidifying();
void stopHumidifying();
void startVentilating();
void stopVentilating();
void startDrying();
void stopDrying();

void controlTemperature(float* temp);

//uint16_t addToOutUart0(char* txt, uint16_t len);

#endif
