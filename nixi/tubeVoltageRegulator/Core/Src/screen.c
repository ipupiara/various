
#include <screen.h>
#include <cpu.h>
#include <main.h>
#include <string.h>
#include <nixi_i2c.h>

//    wip   work in progress     ,    entry on own risk :-)
/////////////////////////////  work in progress ,  access on on risk
#define screenI2cAddress 0x3c


#define LCD_CLEARDISPLAY 0x01
#define LCD_RETURNHOME 0x02
#define LCD_ENTRYMODESET 0x04
#define LCD_DISPLAYCONTROL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAMADDR 0x40
#define LCD_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCD_CURSORLEFTADRDEC 0x00
#define LCD_CURSORRIGHTADRINC 0x02
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

#define waitShortCs   	20
#define waitMediumCs		50
#define waitLongCs		1200

#define byteArrayMaxSz   80

typedef enum {
	jobActive,
	jobInactive
} jobStateEnum;

typedef void(*t_fvoid)(void);
typedef void(*t_fPar)(void* pCmdLine);

typedef struct {
	uint16_t waitS1ms;
	uint8_t xPos;
	uint8_t yPos;
	t_fvoid  stepMethod ;
} screenJobStepType ;

typedef struct {
	uint16_t waitS1ms;
	union {
		struct {
			uint8_t xPos;
			uint8_t yPos;
		} pos;
		void* param;
	} uni1;
	union {
		t_fvoid  stepMethod ;
		t_fPar   stepParMethod;
	} uni2;
} screenJobStepT ;

typedef struct {
	uint8_t   amtJobSteps;
	screenJobStepType  screenJobSteps [];

}screenJobType;

typedef struct {
	uint8_t   amtJobSteps;
	screenJobStepT  screenJobSteps [];

}screenJobT;

typedef uint8_t commandLineType [];

typedef struct  {
	uint8_t len;
	uint8_t buffer [byteArrayMaxSz];
} byteArrayT;

typedef byteArrayT* pByteArrayT;

jobStateEnum jobState;
screenJobType *  currentScreenJob;
uint8_t  currentStepIndex;
uint8_t  currentWaitCycle;

uint8_t waitCycleCounter;		//  variables used for debugging
uint8_t useWaitCycleCounter;    //  variables used for debugging

void setHelloPaintJob();

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
		toMove = byteArrayMaxSz - pBary->len ;
	}
//	uint8_t i1;
//	for (i1 = 0; i1 < toMove; ++ i1) {
//		pBary->buffer[pBary->len] = arr[i1];
//		++pBary->len;
//	}

	memmove(&(pBary->buffer[pBary->len]),arr,toMove);
	pBary->len += toMove;
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
	CPU_IntDis();  //  todo check that own assembler uses same constants as code in HAL (not only same value)
	if ( jobState == jobInactive) {
		currentScreenJob = sJob;
		currentStepIndex = 0;
		currentWaitCycle = 0;
		clear(&byteBuffer);  //  could be omitted here after debugging
		jobState = jobActive;
		res = 1;
	}
	CPU_IntEn();

	return res;
}

void  screenCentiStepExecution( uint8_t sz, screenJobStepType  sJob [sz] )
{
	uint16_t waitTime = sJob[currentStepIndex].waitS1ms;
	if (currentWaitCycle < waitTime) {
		if (currentWaitCycle == 0) {
			clear(&byteBuffer);
			sJob [currentStepIndex].stepMethod();
			sendI2cScreenCommand();
		}
		++ currentWaitCycle;
	} else {
		currentWaitCycle = 0;
		++ currentStepIndex;
	}
}

void screenS1msTimer ()
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

	if (useWaitCycleCounter == 1) {
		if (waitCycleCounter == 50)  {
			waitCycleCounter = 0;
			useWaitCycleCounter = 0;
			setHelloPaintJob();
		} else  {
			++ waitCycleCounter;
		}
	}
}

uint8_t lines [4] = {0x00, 0x40, 0x14, 0x54};

uint8_t getDdramAdr(uint8_t xPos, uint8_t yPos)
{
	uint8_t res = lines [yPos - 1];
	res += xPos - 1;
	return res;
}

void initScreenFuntionSet(void)
{
	commandLineType cmd = {LCD_LastControlByte + LCD_CommandControlByte, LCD_FUNCTIONSET + LCD_8BITMODE + LCD_2LINE + LCD_5x8DOTS};
	addToByteArray(&byteBuffer, 2, cmd);
}

void initDisplayControl(void)
{
	commandLineType cmd = {LCD_LastControlByte + LCD_CommandControlByte,LCD_DISPLAYCONTROL+ LCD_DISPLAYON + LCD_CURSOROFF + LCD_BLINKOFF};
	addToByteArray(&byteBuffer, 2, cmd);
}

void clearDisplay(void)
{
	commandLineType cmd = {LCD_LastControlByte + LCD_CommandControlByte,LCD_CLEARDISPLAY};
	addToByteArray(&byteBuffer, 2, cmd);
}

void initEntryModeSet(void)
{
	commandLineType cmd = {LCD_LastControlByte + LCD_CommandControlByte,LCD_ENTRYMODESET + LCD_CURSORRIGHTADRINC };
	addToByteArray(&byteBuffer, 2, cmd);
}

void addCursorToByteArray(uint8_t xPos, uint8_t yPos)
{
	uint8_t adr = 0x00;
	adr = getDdramAdr(xPos, yPos);
	commandLineType cmd = {LCD_LastControlByte + LCD_CommandControlByte,LCD_SETDDRAMADDR + adr };
	addToByteArray(&byteBuffer, 2, cmd);
}

void setCursor(void)
{
	addCursorToByteArray(currentScreenJob->screenJobSteps[currentStepIndex].xPos,
			currentScreenJob->screenJobSteps[currentStepIndex].yPos);
//	uint8_t adr = 0x00;
//	adr = getDdramAdr(currentScreenJob->screenJobSteps[currentStepIndex].xPos, currentScreenJob->screenJobSteps[currentStepIndex].yPos);
//	commandLineType cmd = {LCD_LastControlByte + LCD_CommandControlByte,LCD_SETDDRAMADDR + adr };
//	addToByteArray(&byteBuffer, 2, cmd);
}

void returnHome(void)
{
	commandLineType cmd = {LCD_LastControlByte + LCD_CommandControlByte,LCD_RETURNHOME};
	addToByteArray(&byteBuffer, 2, cmd);
}

void helloScreen(void)
{
	commandLineType cmd = {LCD_ContinuousControlByte + LCD_CommandControlByte,LCD_RETURNHOME,LCD_LastControlByte + LCD_AsciiControlByte};
	addToByteArray(&byteBuffer, 3, cmd);
	char* stri = "tubeVoltageRegulator";
	addToByteArray(&byteBuffer, strlen(stri), (uint8_t*) stri);
}
//  experimental   question: how much can be sent together with i2c without performing wait states,
//   handling of needed waitStates could as well be done on display itself (encapsulation principle) ...  ??
void anotherScreen (commandLineType** pCmdLine)
{
	commandLineType cmd = {LCD_LastControlByte + LCD_AsciiControlByte};
	addToByteArray(&byteBuffer, 1, cmd);
	char* stri = "tubeVoltageRegulator";
	addToByteArray(&byteBuffer, strlen(stri), (uint8_t*) stri);
}

void emptyWait()
{

}

void paintHello()
{
	commandLineType cmd = {LCD_LastControlByte + LCD_AsciiControlByte};
	addToByteArray(&byteBuffer, 1, cmd);
	char* stri = "screen initialized";
	addToByteArray(&byteBuffer, strlen(stri), (uint8_t*) stri);
}

void paintCanScr()
{

}

screenJobType  initJob = {6, {{waitLongCs,0,0,initScreenFuntionSet}, {waitShortCs,0,0, initDisplayControl},
								{waitShortCs,0,0, clearDisplay},{waitShortCs,0,0, initEntryModeSet},
								{waitShortCs,0,0,returnHome},{waitShortCs,0,0, paintHello}}};

screenJobType testPaint = {8, {{waitShortCs,1,1,setCursor}, {waitShortCs,0,0, paintHello}
							, {waitShortCs,2,1,setCursor}, {waitShortCs,0,0, paintHello}
							, {waitShortCs,3,1,setCursor}, {waitShortCs,0,0, paintHello}
							, {waitShortCs,4,1,setCursor}, {waitShortCs,0,0, paintHello}}};

screenJobType halloPaint = {2, {{waitShortCs,1,1,setCursor}, {waitShortCs,0,0, paintHello}}};

screenJobType canScreen = {3, {{waitShortCs,0,0, clearDisplay},{waitShortCs,1,1,setCursor},{waitShortCs,1,1,paintCanScr}}};


void paintCanScreen()
{
	setNextScreenJob(&canScreen);
}

void setDebugScreenJob()
{
	setNextScreenJob(&testPaint);
}

void setHelloPaintJob()
{
	setNextScreenJob(&halloPaint);
}


void displayFatalError()
{

}

void initScreen()
{
	currentScreenJob = NULL;
	currentStepIndex = 0;
	jobState = jobInactive;
	currentWaitCycle = 0;
	waitCycleCounter = 0;
	useWaitCycleCounter = 0;
	clear(&byteBuffer);
	setNextScreenJob(&initJob);
}

