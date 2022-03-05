
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

#define byteArrayMaxSz   80

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
	uint8_t   amtJobSteps;
	screenJobStepType  screenJobSteps [];

}screenJobType;


typedef uint8_t commandLineType [];

typedef void(*t_fPar)(commandLineType* pCmdLine);

typedef struct  {
	uint8_t len;
	uint8_t buffer [byteArrayMaxSz];
} byteArrayT;

typedef byteArrayT* pByteArrayT;

jobStateEnum jobState;
screenJobType *  currentScreenJob;
uint8_t  currentStepIndex;
uint8_t  currentWaitCycle;

void clear(pByteArrayT pBary)
{
	memset(pBary,0,byteArrayMaxSz);
}

void addToByteArray(pByteArrayT pBary,uint8_t sz, uint8_t  arr [sz])
{
	uint8_t toMove;
	if (pBary->len + sz <= byteArrayMaxSz)  {
		toMove = sz;
	}else  {
		toMove = byteArrayMaxSz - pBary->len;
	}
	memmove(pBary->buffer,arr,toMove);  //   todo   fix destination calculation now worng
}

byteArrayT byteBuffer;


uint8_t sendI2cScreenCommand()
{
	uint8_t res = 0;
	res = sendI2cByteArray(screenI2cAddress,byteBuffer.buffer ,byteBuffer.len);
	return res;
}

uint8_t setNextScreenJob(screenJobType* sJob)
{
	uint8_t res = 0;  // todo  check that used inside privileged code, else no effect of method
						//  see also F103 programming manual  cpsid instruction, also check primask values... confusings??...
	CPU_IntDis();
	if ( jobState == jobInactive) {
		currentScreenJob = sJob;
		currentStepIndex = 0;
		currentWaitCycle = 0;
		clear(&byteBuffer);
		jobState = jobActive;
		res = 1;
	}
	CPU_IntEn();

	return res;
}

void  screenCentiStepExecution( uint8_t sz, screenJobStepType  sJob [sz] )
{
	uint8_t waitTime = sJob[currentStepIndex].waitCs;
	if (currentWaitCycle < waitTime) {
		++ currentWaitCycle;
	} else {
		sJob [currentStepIndex].stepMethod();

		currentWaitCycle = 0;
		++ currentStepIndex;
	}
}

void screenCentiSecTimer ()
{
	screenJobType*  screenJob = NULL;

	CPU_IntDis();
		if (jobState == jobActive) {
			screenJob = currentScreenJob;
		}
	CPU_IntEn();

	if (screenJob != NULL) {
		screenCentiStepExecution(screenJob->amtJobSteps,screenJob->screenJobSteps);

		if (currentStepIndex >= screenJob->amtJobSteps) {
			CPU_IntDis();
			currentScreenJob = NULL;
			currentWaitCycle = 0;
			currentStepIndex = 0;
			jobState = jobInactive;
			CPU_IntEn();
		}
	}
}

void initScreenFuntionSet(void)
{
	commandLineType initCommand = {LCD_LastControlByte + LCD_CommandControlByte, LCD_FUNCTIONSET + LCD_8BITMODE + LCD_2LINE + LCD_5x8DOTS};
	addToByteArray(&byteBuffer, 2, initCommand);
}


void initDisplayControl(void)
{
	commandLineType initCommand = {LCD_LastControlByte + LCD_CommandControlByte,LCD_DISPLAYCONTROL+ LCD_DISPLAYON };
	addToByteArray(&byteBuffer, 2, initCommand);
}

void clearDisplay(void)
{
	commandLineType initCommand = {LCD_LastControlByte + LCD_CommandControlByte,LCD_CLEARDISPLAY};
	addToByteArray(&byteBuffer, 2, initCommand);
}

void initEntryModeSet(void)
{
	commandLineType initCommand = {LCD_LastControlByte + LCD_CommandControlByte,LCD_ENTRYMODESET + LCD_ENTRYLEFT };
	addToByteArray(&byteBuffer, 2, initCommand);
}

void helloScreen(void)
{
	commandLineType initCommand = {LCD_ContinuousControlByte + LCD_CommandControlByte,LCD_RETURNHOME,LCD_LastControlByte + LCD_AsciiControlByte};
	addToByteArray(&byteBuffer, 3, initCommand);
	char* stri = "tubeVoltageRegulator";
	addToByteArray(&byteBuffer, strlen(stri), (uint8_t*) stri);
}
//  experimental   question: how much can be sent together with i2c without performing wait states,
//   handling of needed waitStates could as well be done on display itself (encapsulation principle) ...  ??
void anotherScreen (commandLineType** pCmdLine)
{
	commandLineType initCommand = {LCD_ContinuousControlByte + LCD_CommandControlByte,LCD_RETURNHOME,LCD_LastControlByte + LCD_AsciiControlByte};
	addToByteArray(&byteBuffer, 3, initCommand);
	char* stri = "tubeVoltageRegulator";
	addToByteArray(&byteBuffer, strlen(stri), (uint8_t*) stri);
}


screenJobType  initJob = {5, {{50,initScreenFuntionSet}, {10, initDisplayControl}, {10, clearDisplay}, {10, initEntryModeSet}, {10, helloScreen}}};


void initScreen()
{
	currentScreenJob = NULL;
	currentStepIndex = 0;
	jobState = jobInactive;
	currentWaitCycle = 0;
	clear(&byteBuffer);
	setNextScreenJob(&initJob);


}

