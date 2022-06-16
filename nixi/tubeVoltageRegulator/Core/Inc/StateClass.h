
#if !defined(CCtateClassHeader)
#define CCtateClassHeader

#include "tstatechart.h"
#include <main.h>
//#include "TriacDefines.h"

extern TStatechart	 STriacHumidityChart;
extern TStatechart* PTriacHumidityChart;
//
//TStatechart	 SGrowboxI2CChart;
//TStatechart* PGrowboxI2CChart;
//
//
//enum eEventTypes
//{
	//eReset,
	//eValueAssignement,
	//eVentilationStartTimer,
	//eVentilationStopTimer,
	//eVentilationButtonPressed,
	//eVentilationStopButtonPressed
//};

enum eEventTypes
{
	eReset,
	eValueAssignement,
	eVentilationStartTimer,
	eVentilationStopTimer,
	eVentilationButtonPressed,
	eVentilationStopButtonPressed,
	eCharEntered,
	eF1Pressed,
	ePressed,
	eAstPressed,
	eNumPressed,
	eTimeOutDurationTimer,
	eSecondsTick,
	eStartPressed,
	eStopPressed,
	eAdcTick,
	eTWIDataReceived,
	eFatalError
};


typedef struct CGrowBoxEvent {
	int evType;
	floatType humidity;
	floatType temperature;
	union evData {
		int8_t keyCode;
		struct zeroAdjustingState {			// currently not in use
			floatType   voltage;  
			int8_t  potiPos;
			int8_t  jobType;
		} zeroAdjustingState;
	}  evData;
} CGrowBoxEvent ;




void startStateCharts();


void stopStateCharts();


bool processTriacEvent(TStatechart* tStCh,CGrowBoxEvent* ev);




#endif


