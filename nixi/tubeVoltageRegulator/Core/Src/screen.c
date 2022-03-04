
#include <screen.h>
#include <cpu.h>
#include <main.h>
#include <string.h>
#include <nixi_i2c.h>

//    wip   work in progress     ,    entry on own risk :-)
/////////////////////////////  work in progress ,  access on on risk

#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_ENTRYRIGHT 0x00
#define LCD_ENTRYLEFT 0x02
#define LCD_ENTRYSHIFTINCREMENT 0x01
#define LCD_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCD_DISPLAYON 0x04
#define LCD_DISPLAYOFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00

// flags for display/cursor shift
#define LCD_DISPLAYMOVE 0x08
#define LCD_CURSORMOVE 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

// flags for function set
#define LCD_8BITMODE 0x10
#define LCD_4BITMODE 0x00
#define LCD_2LINE 0x08
#define LCD_1LINE 0x00
#define LCD_5x10DOTS 0x04
#define LCD_5x8DOTS 0x00

// flags for backlight control
#define LCD_BACKLIGHT 0x08
#define LCD_NOBACKLIGHT 0x00

//  control bytes
#define LCD_ContinuousControlByte  0x80
#define LCD_LastControlByte			0x00

//  flags for control bytes
#define LCD_AsciiControlByte		0x40
#define LCD_CommandControlByte      0x00

typedef enum {
	jobActive,
	jobInactive
} jobStateEnum;

typedef void(*t_fvoid)(void);


typedef struct {
	uint8_t waitCs;
	t_fvoid  stepMethod ;
} screenJobStepType ;

typedef struct {
	uint8_t   size;
	screenJobStepType  screenJob [];

}screenJobType;


typedef uint8_t commandLineType [];

typedef void(*t_fPar)(commandLineType* pCmdLine);

jobStateEnum jobState;
screenJobType *  currentScreenJob;
uint8_t  currentStep;
uint8_t  currentWaitCycle;


uint8_t sendI2cScreenCommand(uint8_t* cmd)
{
	uint8_t res = 0;
	sendI2cByteArray(0x3c,cmd,strlen((char*)cmd));
	return res;
}

uint8_t setNextScreenJob(screenJobType* sJob)
{
	uint8_t res = 0;  // todo  check that used inside privileged code, else no effect of method
						//  see also F103 programming manual  cpsid instruction
	CPU_IntDis();
	if ( jobState == jobInactive) {
		currentScreenJob = sJob;
		currentStep = 0;
		currentWaitCycle = 0;
		jobState = jobActive;
		res = 1;
	}
	CPU_IntEn();

	return res;
}

void  screenCentiStepExecution( uint8_t sz, screenJobStepType  sJob [sz] )
{
	uint8_t waitTime = sJob[currentStep].waitCs;
	if (currentWaitCycle < waitTime) {
		++ currentWaitCycle;
	} else {
		sJob [currentStep].stepMethod();
		currentWaitCycle = 0;
		++ currentStep;
	}
}

void screenCentiSecTimer ()
{
	screenJobType*  sJ = NULL;

	CPU_IntDis();
		if (jobState == jobActive) {
			sJ = currentScreenJob;
		}
	CPU_IntEn();

	if (sJ != NULL) {
		screenCentiStepExecution(sJ->size,sJ->screenJob);

		if (currentStep >= sJ->size) {
			CPU_IntDis();
			currentScreenJob = NULL;
			currentWaitCycle = 0;
			currentStep = 0;
			jobState = jobInactive;
			CPU_IntEn();
		}
	}
}

void initScreenFuntionSet(void)
{
	commandLineType initCommand = {LCD_LastControlByte + LCD_CommandControlByte, LCD_FUNCTIONSET + LCD_8BITMODE + LCD_2LINE + LCD_5x8DOTS,0x00};
	sendI2cScreenCommand(initCommand);
}


void initDisplayControl(void)
{
	commandLineType initCommand = {LCD_LastControlByte + LCD_CommandControlByte,LCD_DISPLAYCONTROL+ LCD_DISPLAYON,0x00 };
	sendI2cScreenCommand(initCommand);
}

void clearDisplay(void)
{
	commandLineType initCommand = {LCD_LastControlByte + LCD_CommandControlByte,LCD_CLEARDISPLAY,0x00 };
	sendI2cScreenCommand(initCommand);
}

void initEntryModeSet(void)
{
	commandLineType initCommand = {LCD_LastControlByte + LCD_CommandControlByte,LCD_ENTRYMODESET + LCD_ENTRYLEFT,0x00 };
	sendI2cScreenCommand(initCommand);
}

void helloScreen(void)
{
	commandLineType initCommand = {LCD_ContinuousControlByte + LCD_CommandControlByte,LCD_RETURNHOME,LCD_LastControlByte + LCD_AsciiControlByte,0x00};
	strcat((char*) initCommand,"tubeVoltageRegulator");
	sendI2cScreenCommand(initCommand);
}

// experimental, question is what needs more ram / flash
void anotherScreen (commandLineType** pCmdLine)
{
	commandLineType initCommand = {LCD_ContinuousControlByte + LCD_CommandControlByte,LCD_RETURNHOME,LCD_LastControlByte + LCD_AsciiControlByte,0x00};
	strcat((char*) initCommand,"tubeVoltageRegulator");
	*pCmdLine= &initCommand;
}

screenJobType  initJob = {5, {{50,initScreenFuntionSet}, {10, initDisplayControl}, {10, clearDisplay}, {10, initEntryModeSet}, {10, helloScreen}}};


void initScreen()
{
	currentScreenJob = NULL;
	currentStep = 0;
	jobState = jobInactive;
	currentWaitCycle = 0;
	setNextScreenJob(&initJob);


}

