
#include <stdio.h>
//#include <avr/io.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
//#include <avr/interrupt.h>
#include "TriacIntr.h"
#include "TriacDefines.h"
//#include "twi_master.h"

//#include "TriacIntr.h"


char lastFatalErrorString [20];
int8_t fatalErrorOccurred;


void initTimerPorts();

void info_printf(const char *emsg, ...)
{
	va_list ap;
	va_start(ap,emsg);
//
////	#ifndef printCsvData
//		char buffer [0xff];
//		vsnprintf(buffer,sizeof(buffer),emsg,ap);
////		printf(buffer);
////		addToOutUart0(buffer,strlen(buffer));
//
////	#endif
//
	va_end(ap);
}


void csv_printf(const char *emsg, ...)
{
	va_list ap;
	va_start(ap,emsg);
	
//	#ifdef printCsvData
//		char buffer [0xff];
//		vsnprintf(buffer,sizeof(buffer),emsg,ap);
//		info_printf(buffer);
//	#endif
	
	va_end(ap);
}




//  Testmethod  ShowVar( "fcsi", 32.4f, 'a', "Test string", 4 ); 
//void ShowVar( char *szTypes, ... ) 
//{
	//va_list vl;
	//int i;
	//
	//va_start( vl, szTypes );
	//
	//for( i = 0; szTypes[i] != '\0'; ++i ) {
		//
		//union Printable_t {
			//int     i;
			//float   f;
			//char    c;
			//char   *s;
		//} Printable;
		//
		//switch( szTypes[i] ) {   
			//case 'i':
				//Printable.i = va_arg( vl, int );
				//printf( "%i\n", Printable.i );
				//break;
			//
			//case 'f':
				//Printable.f = va_arg( vl, double );
				//printf( "%f\n", Printable.f );
				//break;
			//
			//case 'c':
				//Printable.c = va_arg( vl, char );
				//printf_s( "%c\n", Printable.c );
				//break;
			        //
			//case 's':
				//Printable.s = va_arg( vl, char * );
				//printf_s( "%s\n", Printable.s );
				//break;
			        //
			//default:
				//break;
		//}
	//}
	//va_end( vl );
//}
//

//#define ownEepromMethods

#ifndef ownEepromMethods
//#include <avr/eeprom.h>
#else

// ATTENTION: use of EEPROM needs BOD Level of at least 2.7 V, otherwise EEPROM memory
// is likely to crash on restore when done at mcu startup


void EEPROM_write(unsigned int uiAddress, unsigned char ucData)
{
	// Wait for completion of previous write 
	while(EECR & (1<<EEPE));
	// Set up address and Data Registers 
	EEAR = uiAddress;
	EEDR = ucData;
	cli();
	/*
	// Write logical one to EEMPE 
	EECR |= (1<<EEMPE);
	// Start eeprom write by setting EEPE 
	EECR |= (1<<EEPE);
	*/

	// standard EEPROM code does not work under all optimization levels (-o directive of compiler)
    // for timing reasons (max. 4 cpu cycles between the two sbi commands for safety reasons)
	 asm volatile (
     "sbi 0x1f,0x02" "\r\n"
     "sbi 0x1f,0x01" "\r\n"
       );
	sei();
}

unsigned char EEPROM_read(unsigned int uiAddress)
{
	// Wait for completion of previous write 
	while(EECR & (1<<EEPE))
	;
	// Set up address register 
	EEAR = uiAddress;
	// Start eeprom read by writing EERE 
	EECR |= (1<<EERE);
	// Return data from Data Register 
	return EEDR;
}

void eeprom_write_byte (uint16_t adr, uint8_t val)
{
	uint8_t checkRes = 0;
	uint16_t adre = (uint16_t) adr;
	EEPROM_write(adre,*(&val));
	checkRes = EEPROM_read(adre); 
	if (val != checkRes) {
		printf("eeprom stored %X, but check returned %X\n",val,checkRes);
	} else {
//		printf("reread ok returned %X\n",checkRes);
	}
}

uint8_t eeprom_read_byte (const uint8_t *adr)
{
	unsigned char bu;
	uint16_t adre = (uint16_t) adr;
	bu =  EEPROM_read(adre); 
	return bu;
}

uint16_t eeprom_read_word (uint16_t* adr)
{	
	uint16_t res;
	unsigned char bu;
	uint16_t adre;
	adre = (uint16_t) adr; 

	res = 0;
	bu = EEPROM_read(adre); 
	*(&res) = bu;
	bu = EEPROM_read(adre + 1); 
	*((unsigned char*)&res+1) = bu;

	return res;
}

void eeprom_write_word(uint16_t* adr, uint16_t val)
{
	uint16_t adre;
	adre = (uint16_t) adr; 
	EEPROM_write(adre,*(&val));
	EEPROM_write(adre + 1,*((unsigned char*)&val + 1));
}



#endif



/*
void delayEmptyProc ()
{
}



void delay6pnt2d5us(unsigned int enn)
{
//delay approx ( n * 2.5 us ) + 6    at 11.0592  mhz  
//    6 Plus N Times  2 Dot 5  (  "6pnt2d5"  )

//n        t    tested on Simulator 2  (PN 28. May 2011)

//0		6
//1		8 - 9
//2		11
//4		16
//8		25 -26
//16	45   approx 500 cycles

//  accurate enough for our keyboard delays


#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"

uint8_t ix;  
	ix= 0;

  while(enn--){
  }
} 

#pragma GCC diagnostic pop
*/

void initDefines()
{

}



floatType  GetIdleVentilationDelayMinutes()
{
	return IdleVentilationDelayMinutes;
}

floatType  GetIdleVentilationMinutes()
{
	return IdleVentilationMinutes;
}

floatType  GetHumidifyingLowerLimit()
{
	return HumidifyingLowerLimit;
}

floatType GetHumidifyingUpperLimit()
{
	return HumidifyingUpperLimit;
}

floatType  GetDryingUpperLimit()
{
	return DryingUpperLimit;
}
floatType  GetDryingLowerLimit()
{
	return DryingLowerLimit;
}

//////// TWI  communication   //////////////

//#define rtc1307Address  0x00
//
//uint8_t rxBuffer [10];
//
//void setTimerPorts(TimeClock now);
//
//void sendTWIDataRequest()
//{
//	twi_start_rx(rtc1307Address,(uint8_t *)&rxBuffer,7);
//}
//
//void onTWIDataReceived()
//{
//	TimeClock nowTime;
//	nowTime.hour = (rxBuffer[2]  & 0b00111111);
//	nowTime.minute = (rxBuffer[1] & ~(0x80));
//	setTimerPorts(nowTime);
//}
//
//void sendSetupData()
//{
//	// init configuration of 1307
//	// set CH to 0
//	// 12/23 hour mode to 24
//	//  set 0x07 set sqwe to 0 (disabled), OUT = 0 (or 1 if you like  :-), RS1 and RS0 to 0 (1 hz)
//	//  needs only be done the first time 1307 is used and stayes then persistent in EEPROM.
//	//  therefor check that initial setup has beed done
//
//	twi_synch_rx(rtc1307Address,(uint8_t *)&rxBuffer,8);
//	if ((rxBuffer[0] & 0x80) != 0x00)  {
//		rxBuffer [0] = 0x00;
//		twi_synch_tx(rtc1307Address,(uint8_t *)&rxBuffer[0],1);
//	}
//	if ((rxBuffer[2] & 0x40) != 0x00)  {
//		rxBuffer [2] &= ~(0x40);
//		twi_synch_tx(rtc1307Address,(uint8_t *)&rxBuffer[2],1);
//	}
//
//	if ((rxBuffer[8] & 0b10010011) != 0x00 ) {
//		rxBuffer[8] &=  ~(0b10010011);
//		twi_synch_tx(rtc1307Address,(uint8_t *)&rxBuffer[8],1);
//	}
//
//}
//
//OnOffTimerPorts onOffTimerPorts [amtOnOffTimerPorts] = { {0x1B, PORTA0,
//															{ {{1,30},{1,35}},{{12,30},{12,35}},
//															{{18,30},{18,35}}, {{0xFF,30},{1,35}}  }
//															} ,
//															{0x1B, PORTA1,
//																{ {{1,30},{1,35}},{{12,30},{12,35}},
//																{{18,30},{18,35}}, {{0xFF,30},{1,35}}  }
//															}
//														};
//
//
//uint8_t amtBcd (uint8_t amt1307Formatted)
//{
//	uint8_t res =  ((amt1307Formatted >> 4)   * 10 )   + (amt1307Formatted & 0x0F);
//	return res;
//}
//
//uint16_t amtMinutesSinceMidnight(TimeClock now)  // hour minutes defined as on address 0x01 and 0x02 on 1307, so some bits need to be masked off
//{
//	uint16_t res;
//	res = (amtBcd(now.hour & ~((1<<7) | (1 << 6))) * 60)  + amtBcd(now.minute & ~(1 << 7));
//
//
//	return res;
//}
//
//uint8_t layesInInterval(TimeClock now,  OnOffInterval* intV)
//{
//	uint8_t res = 0;
//	uint16_t nowMinutes = amtMinutesSinceMidnight(now);
//	uint16_t intervalOnMinutes = amtMinutesSinceMidnight(intV->onClock);
//	uint16_t intervalOffMinutes = amtMinutesSinceMidnight(intV->offClock);
//
//	if (intervalOnMinutes <= intervalOffMinutes)  {
//		if ((nowMinutes >= intervalOnMinutes) && (nowMinutes <= intervalOffMinutes)) {
//			res = 1;
//		}
//	}  else {
//		if	 ((nowMinutes >= intervalOnMinutes) || (nowMinutes <= intervalOffMinutes)) {
//			res = 1;
//		}
//	}
//	return res;
//}
//
//
//uint8_t  isCurrentlyOn(TimeClock now, uint8_t timerPort)
//{
//	uint16_t res = 0;
//	uint8_t cnt = 0;
//	while ((cnt < amtOnOffIntervalsPerTimerPort) && (res == 0)) {
//		res = layesInInterval(now,&onOffTimerPorts[timerPort].onOffIntervals[cnt]);
//		++ cnt;
//	}
//	return res;
//}
//
//void setTimerPort(TimeClock now, uint8_t timerPort)
//{
//	if (isCurrentlyOn(now,timerPort))  {
//		_SFR_IO8(onOffTimerPorts[timerPort].cpuPort) |= (1 << onOffTimerPorts[timerPort].cpuPin);
//	} else {
//		_SFR_IO8(onOffTimerPorts[timerPort].cpuPort) &= ~(1 << onOffTimerPorts[timerPort].cpuPin);
//	}
//}
//
//void setTimerPorts(TimeClock now)
//{
//	uint8_t cnt;
//	for (cnt = 0; cnt < amtOnOffTimerPorts; ++ cnt)  {
//		setTimerPort( now, cnt);
//	}
//}
//
//
//void initTimerPorts()
//{
//	DDRA |= (1 << DDA0);
//	DDRA |= (1 << DDA1);
//	memset(&rxBuffer,0,sizeof(rxBuffer));
//	sendSetupData();
//	// init configuration of 1307
//	// set CH to 0
//	// 12/23 hour mode to 24
//	//  set 0x07 set sqwe to 0 (disabled), OUT = 0 (or 1 if you like  :-), RS1 and RS0 to 0 (1 hz)
//}
//
//void setClockToTime(uint8_t hours, uint8_t minutes)
//{
//
//}
