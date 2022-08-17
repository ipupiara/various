
//#include <avr/io.h>
#include <stdio.h>
//#include <iostream.h>
//#include <assert.h>

#include "tstatechart.h"
#include "StateClass.h"
#include "TriacDefines.h"
#include "StateClass.h"
#include "TriacIntr.h"
#include <screen.h>
//#include "triacPID.h"
//#include "triacUI.h"
//#include "twi_master.h"

//#define debugStatechart

extern const uStInt uStIntHandlingDone;
extern const uStInt uStIntNoMatch;

CGrowBoxEvent* currentEvent;

TStatechart	 STriacHumidityChart;
TStatechart* PTriacHumidityChart;

//TStatechart	 SGrowboxI2CChart;
//TStatechart* PGrowboxI2CChart;


// This defines and names the states the class has.
#warning  attention: sequence must be the same as in xaStates (below)  !!!


// This defines and names the states the class has.
enum eHumidifyingStates
{
	eStateGrowBoxKeepingHumidity,
	eHumidifyingStartState = eStateGrowBoxKeepingHumidity,
	eStateHumidityControlRunning,
	eStateHumidifying,
	eStateIdle,
	eStateNonVentilating,
	eStateVentilating,
	eStateDrying,
	eStateFatalError,
	eNumberOfHumidifyingStates
};


uStInt evFatalErrorChecker(void)
{
	uStInt res = uStIntNoMatch;
	info_printf("check for event in State evStateIdle\n");

	return (res);
}


void entryFatalErrorState(void)
{
	info_printf("entry FatalError\n");
	info_printf("**************fatal Error: %s *************************\n",lastFatalErrorString);
	displayFatalError();
}

void exitFatalErrorState(void)
{
	info_printf("exit FatalError\n");
}




uStInt evStateGrowBoxKeepingHumidity(void)
{
//	info_printf("check for event in State evStateGrowBoxKeepingHumidity\n");
	if (currentEvent->evType == eReset)  
	{
		BEGIN_EVENT_HANDLER(PTriacHumidityChart, eStateGrowBoxKeepingHumidity );
			// No event action.
		#ifdef debugStatechart
			info_printf("eventhandler evStateGrowBoxKeepingHumidity proceeding to eStateGrowBoxKeepingHumidity\n");
		#endif	
	
		END_EVENT_HANDLER(PTriacHumidityChart);
		
///*  left this as an original example for history states
 //
		//BEGIN_EVENT_HANDLER(CHumidityStateClass,   eStateGrowBoxKeepingHumidity | u32WithHistory);
			//// No event action.
		//END_EVENT_HANDLER(CHumidityStateClass );
//*/
		return (uStIntHandlingDone);
	}
	
	if (currentEvent->evType == eFatalError)
	
	{
		BEGIN_EVENT_HANDLER(PTriacHumidityChart, eStateFatalError );
			// No event action.
		#ifdef debugStatechart
			info_printf("eventhandler evStateGrowBoxKeepingHumidity proceeding to eStateFatalError\n");
		#endif	

		END_EVENT_HANDLER(PTriacHumidityChart);
		
///*  left this as an original example for history states
 //
		//BEGIN_EVENT_HANDLER(CHumidityStateClass,   eStateGrowBoxKeepingHumidity | u32WithHistory);
			//// No event action.
		//END_EVENT_HANDLER(CHumidityStateClass );
//*/
		return (uStIntHandlingDone);
	}

	if (currentEvent->evType == eSecondsTick)
		{
			setGrowboxScreen();
	//		initScreen();
	//		++ i2cSecondCounter;
	//		if (i2cSecondCounter >= 3)  {
	//			snprintf((char *)&i2cStateString,sizeof(i2cStateString),"i2c err to");
	//
	//			BEGIN_EVENT_HANDLER(PGrowboxI2CChart, eStateI2CIdleError );
	////			No event action.
	//			END_EVENT_HANDLER(PGrowboxI2CChart );
	//			return (uStIntHandlingDone);
	//		}
	//
		}

	return (uStIntNoMatch);
}



void entryStateGrowBoxKeepingHumidity(void)
{
	info_printf("CHumidityStateClass::entryStateGrowBoxKeepingHumidity\n");
}

void exitStateGrowBoxKeepingHumidity(void)
{
	info_printf("CHumidityStateClass::exitStateGrowBoxKeepingHumidity\n");
}


uStInt evStateHumidityControlRunning(void)
{
//	info_printf("check for event in State evStateGrowBoxKeepingHumidity\n");
//	if (currentEvent->evType == eReset)  
	//{
		//BEGIN_EVENT_HANDLER(PTriacHumidityChart, eStateGrowBoxKeepingHumidity );
			//// No event action.
		//END_EVENT_HANDLER(PTriacHumidityChart);
		//
/////*  left this as an original example for history states
 ////
		////BEGIN_EVENT_HANDLER(CHumidityStateClass,   eStateGrowBoxKeepingHumidity | u32WithHistory);
			////// No event action.
		////END_EVENT_HANDLER(CHumidityStateClass );
////*/
		////return (uStIntHandlingDone);
	//}
	return (uStIntNoMatch);
}

void entryStateHumidityControlRunning(void)
{
	info_printf("CHumidityStateClass::entryStateHumidityControlRunning\n");
}


void exitStateHumidityControlRunning(void)
{
	info_printf("CHumidityStateClass::exitStateHumiditiyControlRunning\n");
}




uStInt evStateHumidifying(void)
{
//	info_printf("check for event in State evStateHumidifying\n");
	if ((currentEvent->evType == eValueAssignement) && (GetHumidifyingUpperLimit() < currentEvent->humidity))
	{
		BEGIN_EVENT_HANDLER(PTriacHumidityChart,   eStateIdle);
			// No event action.
		#ifdef debugStatechart
			info_printf("eventhandler evStateHumidifying proceeding to eStateIdle\n");
		#endif	
		END_EVENT_HANDLER(PTriacHumidityChart );
		return (uStIntHandlingDone);
	}
	return (uStIntNoMatch);
}


void entryStateHumidifying(void)
{
	info_printf("CHumidityStateClass::entryStateHumidifying\n");
	startHumidifying();
}


void exitStateHumidifying(void)
{
	info_printf("CHumidityStateClass::exitStateHumidifying\n");
	stopHumidifying();
}




uStInt evStateIdle(void)
{
//	info_printf("check for event in State evStateIdle\n");

	if (currentEvent->evType == eValueAssignement) 
	{	if (GetHumidifyingLowerLimit() > currentEvent->humidity)
		{
			BEGIN_EVENT_HANDLER(PTriacHumidityChart,   eStateHumidifying);
				// No event action.
			#ifdef debugStatechart
				info_printf("eventhandler evStateIdle proceeding to eStateHumidifying\n");
			#endif		
			END_EVENT_HANDLER(PTriacHumidityChart );
			return (uStIntHandlingDone);
		}
		if (GetDryingUpperLimit() < currentEvent->humidity)
		{
			BEGIN_EVENT_HANDLER(PTriacHumidityChart,   eStateDrying);
				// No event action.
			#ifdef debugStatechart
				info_printf("eventhandler evStateIdle proceeding to eStateDrying\n");
			#endif		
	
			END_EVENT_HANDLER(PTriacHumidityChart );
			return (uStIntHandlingDone);
		}
	}
	return (uStIntNoMatch);
}


void entryStateIdle(void)
{
	info_printf("CHumidityStateClass::entryStateIdle\n");
}


void exitStateIdle(void)
{
	info_printf("CHumidityStateClass::exitStateIdle\n");
}



uStInt evStateNonVentilating(void)
{
//	info_printf("check for event in State evStateNonVentilating\n");

	if ((currentEvent->evType == eTimeOutDurationTimer) || (currentEvent->evType ==  eVentilationButtonPressed))
	{
		BEGIN_EVENT_HANDLER(PTriacHumidityChart,   eStateVentilating);
			// No event action.
		#ifdef debugStatechart
			info_printf("eventhandler evStateNonVentilating proceeding to eStateVentilating\n");
		#endif		
	
		END_EVENT_HANDLER(PTriacHumidityChart );
		return (uStIntHandlingDone);
	
	}
	return (uStIntNoMatch);
}


void entryStateNonVentilating(void)
{
	info_printf("CHumidityStateClass::entryStateNonVentilating\n");
	startDurationTimer(GetIdleVentilationDelayMinutes() * 60);
}




void exitStateNonVentilating(void)
{
	info_printf("CHumidityStateClass::exitStateNonVentilating\n");
	stopDurationTimer();
}


uStInt evStateVentilating(void)
{
//	info_printf("check for event in State evStateVentilating\n");

	if ((currentEvent->evType == eTimeOutDurationTimer)  || (currentEvent->evType ==  eVentilationStopButtonPressed))  {
		BEGIN_EVENT_HANDLER(PTriacHumidityChart,   eStateNonVentilating);
			// No event action.
		#ifdef debugStatechart
			info_printf("eventhandler evStateVentilating proceeding to eStateNonVentilating\n");
		#endif	

		END_EVENT_HANDLER(PTriacHumidityChart );
		return (uStIntHandlingDone);
	
	}
	return (uStIntNoMatch);
}


void entryStateVentilating(void)
{
	info_printf("CHumidityStateClass::entryStateVentilating\n");
	startVentilating();
	startDurationTimer(GetIdleVentilationMinutes() * 60);
}

void exitStateVentilating(void)
{
	info_printf("CHumidityStateClass::exitStateVentilating\n");
	stopVentilating();
	stopDurationTimer();
}


uStInt evStateDrying(void)
{
//	info_printf("check for event in State evStateDrying\n");
	if ((currentEvent->evType == eValueAssignement) && (GetDryingLowerLimit() > currentEvent->humidity))
	{
		BEGIN_EVENT_HANDLER(PTriacHumidityChart,   eStateIdle);
			// No event action.
		#ifdef debugStatechart
			info_printf("eventhandler evStateDrying proceeding to eStateIdle\n");
		#endif	
	
		END_EVENT_HANDLER(PTriacHumidityChart );
		return (uStIntHandlingDone);
	}
	return (uStIntNoMatch);
}


void entryStateDrying(void)
{
	info_printf("CHumidityStateClass::entryStateDrying\n");
	startDrying();
}


void exitStateDrying(void)
{
	info_printf("CHumidityStateClass::exitStateDrying\n");
	stopDrying();
}


/*
// State transition/handling methods
// left as an example of this defEntry method

void CHumidityStateClass::defEntryStateGrowBoxKeepingHumidity(void)
{

	info_printf("CHumidityStateClass::defEntryStateGrowBoxKeepingHumidity\n");
}
*/




#ifndef  sdccNULL 

#define tfNull 0

#else

t_fvoid  tfNull;

#endif


#ifdef  sdccNULL

tfNull = (t_fvoid ) NULL;

#endif 

// attention: sequence must be the same as above enum eStates

xStateType xaHumidifyingStates[eNumberOfHumidifyingStates] = {
		{eStateGrowBoxKeepingHumidity,
			-1,
			eStateHumidityControlRunning,
			0,
			evStateGrowBoxKeepingHumidity,
			NULL,  
			entryStateGrowBoxKeepingHumidity,
			exitStateGrowBoxKeepingHumidity},
			
				{eStateHumidityControlRunning,
					eStateGrowBoxKeepingHumidity,
					eStateIdle,
					0,
					evStateHumidityControlRunning,
					NULL,
					entryStateHumidityControlRunning,
				exitStateHumidityControlRunning},

/* name						*/	{eStateHumidifying,
	/* parent					*/	eStateHumidityControlRunning,
	/* default_substate			*/	-1,
									0,     //(  keep history)
	/* event-checking func		*/	evStateHumidifying,
	/* default state entry func	*/	NULL,
	/* entering state func		*/	entryStateHumidifying,
/* exiting state func		*/		exitStateHumidifying},

/* name						*/	{eStateIdle,
	/* parent					*/	eStateHumidityControlRunning,
	/* default_substate			*/	eStateVentilating,
									0,
	/* event-checking func		*/	evStateIdle,
	/* default state entry func	*/	NULL,
	/* entering state func		*/	entryStateIdle,
/* exiting state func		*/		exitStateIdle},

		/* name						*/	{eStateNonVentilating,
			/* parent					*/	eStateIdle,
			/* default_substate			*/	-1,
											0,
			/* event-checking func		*/	evStateNonVentilating,
			/* default state entry func	*/	NULL,
			/* entering state func		*/	entryStateNonVentilating,
		/* exiting state func		*/		exitStateNonVentilating},

		/* name						*/	{eStateVentilating,
			/* parent					*/	eStateIdle,
			/* default_substate			*/	-1,
											0,
			/* event-checking func		*/	evStateVentilating,
			/* default state entry func	*/	NULL,
			/* entering state func		*/	entryStateVentilating,
		/* exiting state func		*/		exitStateVentilating},

/* name						*/	{eStateDrying,
	/* parent					*/	eStateHumidityControlRunning,
	/* default_substate			*/	-1,
									0,
	/* event-checking func		*/	evStateDrying,
	/* default state entry func	*/	NULL,
	/* entering state func		*/	entryStateDrying,
/* exiting state func		*/		exitStateDrying},
	
				{eStateFatalError,
					eStateGrowBoxKeepingHumidity,
					-1,
					0,
					evFatalErrorChecker,
					tfNull,
					entryFatalErrorState,
				exitFatalErrorState}
};





/***   I2C states  ********/

//// This defines and names the states the class has.
//enum eI2CStates
//{
//	eStateGrowboxI2C,
//	eI2CStartState = eStateGrowboxI2C,
//	eStateI2CIdle,
//	eStateI2CIdleOK,
//	eStateI2CIdleError,
//	eStateI2CWaitForResponse,
//	eNumberOfI2CStates
//};
//
//
//uint8_t i2cSecondCounter;
//char i2cStateString [15];
//
//
//void entryGrowboxI2C(void)
//{
//	info_printf("::entryStateGrowboxI2C\n");
//	snprintf((char *)&i2cStateString,sizeof(i2cStateString),"i2c ok");
//}
//
//
//
//uStInt evI2CIdleChecker(void)
//{
////	info_printf("check for event in State evStateI2CIdle\n");
//
//	if (currentEvent->evType == eSecondsTick)
//	{
//		++ i2cSecondCounter;
//		if (i2cSecondCounter >= 23)  {
//
//			sendTWIDataRequest();
//
//			BEGIN_EVENT_HANDLER(PGrowboxI2CChart, eStateI2CWaitForResponse );
////				 No event action.
//			END_EVENT_HANDLER(PGrowboxI2CChart );
//			return (uStIntHandlingDone);
//		}
//
//	}
//	return (uStIntNoMatch);
//}
//
//
//void entryStateI2CIdle(void)
//{
////	info_printf("::entryStateI2CIdle\n");
//	i2cSecondCounter = 0;
//}
//
//
//
//
//void exitStateI2CIdle(void)
//{
////	info_printf("::exitStateI2CIdle\n");
//}
//
//
//
//uStInt evStateI2CIdleOKChecker(void)
//{
////	info_printf("check for event in State evStateStateI2CIdleOK\n");
//
////	if ((currentEvent->evType == eTimeOutDurationTimer) || (currentEvent->evType ==  eVentilationButtonPressed))
////	{
////		BEGIN_EVENT_HANDLER(PTriacHumidityTemperatureChart,   eStateVentilating);
//			// No event action.
////		END_EVENT_HANDLER(PTriacHumidityTemperatureChart );
////		return (uStIntHandlingDone);
//
////	}
//	return (uStIntNoMatch);
//}
//
//
//void entryStateI2CIdleOK(void)
//{
////	info_printf("::entryStateStateI2CIdleOK\n");
//}
//
//
//
//
//void exitStateI2CIdleOK(void)
//{
////	info_printf("::exitStateStateI2CIdleOK\n");
//}
//
//
//
//uStInt evI2CIdleErrorChecker(void)
//{
////	info_printf("check for event in State evStateI2CIdleError\n");
//
////	if ((currentEvent->evType == eTimeOutDurationTimer) || (currentEvent->evType ==  eVentilationButtonPressed))
////	{
////		BEGIN_EVENT_HANDLER(PTriacHumidityTemperatureChart,   eStateVentilating);
//			// No event action.
////		END_EVENT_HANDLER(PTriacHumidityTemperatureChart );
////		return (uStIntHandlingDone);
//
////	}
//	return (uStIntNoMatch);
//}
//
//
//void entryStateI2CIdleError(void)
//{
//	info_printf("::entryStateI2CIdleError\n");
//	twi_resetAfterCrash();
//}
//
//
//void exitStateI2CIdleError(void)
//{
////	info_printf("::exitStateI2CIdleError\n");
//}
//
//
//
//
//uStInt evI2CWaitForResponseChecker(void)
//{
////	info_printf("check for event in State evStateI2CWaitForResponse\n");
//
//	if (currentEvent->evType == eSecondsTick)
//	{
//		++ i2cSecondCounter;
//		if (i2cSecondCounter >= 3)  {
//			snprintf((char *)&i2cStateString,sizeof(i2cStateString),"i2c err to");
//
//			BEGIN_EVENT_HANDLER(PGrowboxI2CChart, eStateI2CIdleError );
////			No event action.
//			END_EVENT_HANDLER(PGrowboxI2CChart );
//			return (uStIntHandlingDone);
//		}
//
//	}
//	if (currentEvent->evType == eTWIDataReceived)
//	{
//		snprintf((char *) &i2cStateString,sizeof(i2cStateString),"i2c ok");
//		onTWIDataReceived();
//		BEGIN_EVENT_HANDLER(PGrowboxI2CChart, eStateI2CIdleOK );
////		No event action.
//		END_EVENT_HANDLER(PGrowboxI2CChart );
//		return (uStIntHandlingDone);
//	}
//	return (uStIntNoMatch);
//}
//
//
//
//
//void entryStateI2CWaitForResponse(void)
//{
////	info_printf("::entryStateI2CWaitForResponse\n");
//	i2cSecondCounter = 0;
//}
//
//
//
//
//void exitStateI2CWaitForResponse(void)
//{
////	info_printf("::exitStateI2CWaitForResponse\n");
//}
//
//
//
//
//xStateType xaI2CStates[eNumberOfI2CStates] = {
//	{	eStateGrowboxI2C,
//	 	-1,
//	 	eStateI2CIdle,             // default child
//	 	0,				// keep history
//	 	tfNull,
//	 	tfNull,				// default entry ?
//	 	entryGrowboxI2C,
//	 	tfNull
//	}	,
//
//
//	{	eStateI2CIdle,
//		eStateGrowboxI2C,
//		eStateI2CIdleOK,  // default child
//		0,				// keep history
//		evI2CIdleChecker,
//		tfNull,				// default entry ?
//		entryStateI2CIdle,
//		exitStateI2CIdle
//	}	,
//
//	{   eStateI2CIdleOK ,
//		eStateI2CIdle,
//		-1,             // default child
//		0,				// keep history
//		evStateI2CIdleOKChecker,
//		tfNull,				// default entry ?
//		entryStateI2CIdleOK,
//		exitStateI2CIdleOK
//	} ,
//
//
//	{	eStateI2CIdleError,
//		eStateI2CIdle,
//		-1,             // default child
//		0,				// keep history
//		evI2CIdleErrorChecker,
//		tfNull,				// default entry ?
//		entryStateI2CIdleError,
//		exitStateI2CIdleError
//	}  ,
//
//	{	eStateI2CWaitForResponse,
//		eStateGrowboxI2C,
//		-1,             // default child
//		0,				// keep history
//		evI2CWaitForResponseChecker,
//		tfNull,				// default entry ?
//		entryStateI2CWaitForResponse,
//		exitStateI2CWaitForResponse
//	}
//
//};
//
//

void startStateCharts()
{

 	PTriacHumidityChart = & STriacHumidityChart; 
	createTStatechart (& STriacHumidityChart, xaHumidifyingStates, eNumberOfHumidifyingStates, eHumidifyingStartState);
	info_printf("TriacHumidityTemperature statechart started\n");

 	//PGrowboxI2CChart = & SGrowboxI2CChart;
 	//createTStatechart (& SGrowboxI2CChart, xaHumidifyingStates, eNumberOfI2CStates, eI2CStartState);
	//info_printf("I2CStateChart started\n");
}


void stopStateCharts()
{
	destructTStatechart(&STriacHumidityChart);
//	destructTStatechart(&SGrowboxI2CChart);
}


bool processTriacEvent(TStatechart* tStCh,CGrowBoxEvent* ev)
{
	currentEvent = ev;
	#ifdef debugStatechart
		info_printf("processTriacEvent: firing event: %i\n",ev->evType);
	#endif	
	return ProcessEvent(tStCh);
}

