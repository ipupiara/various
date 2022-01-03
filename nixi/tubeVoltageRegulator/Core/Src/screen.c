#include <main.h>
#include <string.h>

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


//#define cmd(REG)
#define expanderWrite(REG)
#define write(REG)
#define bool uint8_t

void cmd(uint8_t comm)
{

}


void delayMicroseconds()
{

}

	uint8_t _displayfunction;
	uint8_t _displaycontrol;
	uint8_t _displaymode;


uint8_t _addr;
uint8_t 	_cols;
uint8_t 	_rows ;
uint8_t 	_charsize;
uint8_t 	_backlightval;
void  clear(){
	cmd(LCD_CLEARDISPLAY);// clear display, set cursor position to zero
	delayMicroseconds(2000);  // this cmd takes a long time!
}

void  home(){
	cmd(LCD_RETURNHOME);  // set cursor position to zero
	delayMicroseconds(2000);  // this cmd takes a long time!
}

void  setCursor(uint8_t col, uint8_t row){
	int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	if (row > _rows) {
		row = _rows-1;    // we count rows starting w/0
	}
	cmd(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

// Turn the display on/off (quickly)
void  noDisplay() {
	_displaycontrol &= ~LCD_DISPLAYON;
	cmd(LCD_DISPLAYCONTROL | _displaycontrol);
}
void  display() {
	_displaycontrol |= LCD_DISPLAYON;
	cmd(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void  noCursor() {
	_displaycontrol &= ~LCD_CURSORON;
	cmd(LCD_DISPLAYCONTROL | _displaycontrol);
}
void  cursor() {
	_displaycontrol |= LCD_CURSORON;
	cmd(LCD_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void  noBlink() {
	_displaycontrol &= ~LCD_BLINKON;
	cmd(LCD_DISPLAYCONTROL | _displaycontrol);
}
void  blink() {
	_displaycontrol |= LCD_BLINKON;
	cmd(LCD_DISPLAYCONTROL | _displaycontrol);
}

// These cmds scroll the display without changing the RAM
void  scrollDisplayLeft(void) {
	cmd(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVELEFT);
}
void  scrollDisplayRight(void) {
	cmd(LCD_CURSORSHIFT | LCD_DISPLAYMOVE | LCD_MOVERIGHT);
}

// This is for text that flows Left to Right
void  leftToRight(void) {
	_displaymode |= LCD_ENTRYLEFT;
	cmd(LCD_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void  rightToLeft(void) {
	_displaymode &= ~LCD_ENTRYLEFT;
	cmd(LCD_ENTRYMODESET | _displaymode);
}

// This will 'right justify' text from the cursor
void  autoscroll(void) {
	_displaymode |= LCD_ENTRYSHIFTINCREMENT;
	cmd(LCD_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void  noAutoscroll(void) {
	_displaymode &= ~LCD_ENTRYSHIFTINCREMENT;
	cmd(LCD_ENTRYMODESET | _displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void  createChar(uint8_t location, uint8_t charmap[]) {
	location &= 0x7; // we only have 8 locations 0-7
	cmd(LCD_SETCGRAMADDR | (location << 3));
	for (int i=0; i<8; i++) {
		write(charmap[i]);
	}
}

// Turn the (optional) backlight off/on
void  noBacklight(void) {
	_backlightval=LCD_NOBACKLIGHT;
	expanderWrite(0);
}

void  backlight(void) {
	_backlightval=LCD_BACKLIGHT;
	expanderWrite(0);
}
bool  getBacklight() {
  return _backlightval == LCD_BACKLIGHT;
}



void initScreen()
{

}

