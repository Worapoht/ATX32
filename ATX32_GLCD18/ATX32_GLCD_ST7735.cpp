/***************************************************
  This is a library for the Adafruit 1.8" SPI display.

This library works with the Adafruit 1.8" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/358
The 1.8" TFT shield
  ----> https://www.adafruit.com/product/802
The 1.44" TFT breakout
  ----> https://www.adafruit.com/product/2088
as well as Adafruit raw 1.8" TFT display
  ----> http://www.adafruit.com/products/618

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include "ATX32_GLCD_ST7735.h"
#include "glcdfont.c"
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>

SPIClass SPI_3(3); //Create an instance of the SPI Class called SPI_3 that uses the 3rd SPI Port

static unsigned char GCLCD_chipmode=0;
static unsigned char GCLCD_screenAlignment=0;
static unsigned char GCLCD_BGR=1;

inline uint16_t swapcolor(uint16_t x) { 
  return (x << 11) | (x & 0x07E0) | (x >> 11);
}

#if defined (SPI_HAS_TRANSACTION)
  static SPISettings mySPISettings;
#elif defined (__AVR__)
  static uint8_t SPCRbackup;
  static uint8_t mySPCR;
#endif


// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
ATX32_GLCD_ST7735::ATX32_GLCD_ST7735(int8_t cs, int8_t rs, int8_t rst) 
  : Adafruit_GFX_AS(ST7735_TFTWIDTH, ST7735_TFTHEIGHT_18) {
  _cs   = cs;
  _rs   = rs;
  _rst  = rst;
  hwSPI = true;
  _sid  = _sclk = 0;
}

#if defined(CORE_TEENSY) && !defined(__AVR__)
#define __AVR__
#endif

inline void ATX32_GLCD_ST7735::spiwrite(uint8_t c) {

  //Serial.println(c, HEX);

  if (hwSPI) {
#if defined (SPI_HAS_TRANSACTION)
      SPI_3.transfer(c);
#elif defined (__AVR__)
      SPCRbackup = SPCR;
      SPCR = mySPCR;
      SPI_3.transfer(c);
      SPCR = SPCRbackup;
//      SPDR = c;
//      while(!(SPSR & _BV(SPIF)));
#elif defined (__STM32F1__)||(__ATX32__)
      SPI_3.write(c);
#elif defined (__arm__)
      SPI_3.setClockDivider(21); //4MHz
      SPI_3.setDataMode(SPI_MODE0);
      SPI_3.transfer(c);
#endif
  } else {
    // Fast SPI bitbang swiped from LPD8806 library
    for(uint8_t bit = 0x80; bit; bit >>= 1) {
      if(c & bit) *dataport |=  datapinmask;
      else        *dataport &= ~datapinmask;
      *clkport |=  clkpinmask;
      *clkport &= ~clkpinmask;
    }
  }
}


void ATX32_GLCD_ST7735::writecommand(uint8_t c) {
#if defined (SPI_HAS_TRANSACTION)
  SPI_3.beginTransaction(mySPISettings);
#endif
  *rsport &= ~rspinmask;
  *csport &= ~cspinmask;

  //Serial.print("C ");
  spiwrite(c);

  *csport |= cspinmask;
#if defined (SPI_HAS_TRANSACTION)
    SPI_3.endTransaction();
#endif
}


void ATX32_GLCD_ST7735::writedata(uint8_t c) {
#if defined (SPI_HAS_TRANSACTION)
    SPI_3.beginTransaction(mySPISettings);
#endif
  *rsport |=  rspinmask;
  *csport &= ~cspinmask;
    
  //Serial.print("D ");
  spiwrite(c);

  *csport |= cspinmask;
#if defined (SPI_HAS_TRANSACTION)
    SPI_3.endTransaction();
#endif
}

// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80
static const uint8_t PROGMEM
  Bcmd[] = {                  // Initialization commands for 7735B screens
    18,                       // 18 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, no args, w/delay
      50,                     //     50 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, no args, w/delay
      255,                    //     255 = 500 ms delay
    ST7735_COLMOD , 1+DELAY,  //  3: Set color mode, 1 arg + delay:
      0x05,                   //     16-bit color
      10,                     //     10 ms delay
    ST7735_FRMCTR1, 3+DELAY,  //  4: Frame rate control, 3 args + delay:
      0x00,                   //     fastest refresh
      0x06,                   //     6 lines front porch
      0x03,                   //     3 lines back porch
      10,                     //     10 ms delay
    ST7735_MADCTL , 1      ,  //  5: Memory access ctrl (directions), 1 arg:
      0x08,                   //     Row addr/col addr, bottom to top refresh
    ST7735_DISSET5, 2      ,  //  6: Display settings #5, 2 args, no delay:
      0x15,                   //     1 clk cycle nonoverlap, 2 cycle gate
                              //     rise, 3 cycle osc equalize
      0x02,                   //     Fix on VTL
    ST7735_INVCTR , 1      ,  //  7: Display inversion control, 1 arg:
      0x0,                    //     Line inversion
    ST7735_PWCTR1 , 2+DELAY,  //  8: Power control, 2 args + delay:
      0x02,                   //     GVDD = 4.7V
      0x70,                   //     1.0uA
      10,                     //     10 ms delay
    ST7735_PWCTR2 , 1      ,  //  9: Power control, 1 arg, no delay:
      0x05,                   //     VGH = 14.7V, VGL = -7.35V
    ST7735_PWCTR3 , 2      ,  // 10: Power control, 2 args, no delay:
      0x01,                   //     Opamp current small
      0x02,                   //     Boost frequency
    ST7735_VMCTR1 , 2+DELAY,  // 11: Power control, 2 args + delay:
      0x3C,                   //     VCOMH = 4V
      0x38,                   //     VCOML = -1.1V
      10,                     //     10 ms delay
    ST7735_PWCTR6 , 2      ,  // 12: Power control, 2 args, no delay:
      0x11, 0x15,
    ST7735_GMCTRP1,16      ,  // 13: Magical unicorn dust, 16 args, no delay:
      0x09, 0x16, 0x09, 0x20, //     (seriously though, not sure what
      0x21, 0x1B, 0x13, 0x19, //      these config values represent)
      0x17, 0x15, 0x1E, 0x2B,
      0x04, 0x05, 0x02, 0x0E,
    ST7735_GMCTRN1,16+DELAY,  // 14: Sparkles and rainbows, 16 args + delay:
      0x0B, 0x14, 0x08, 0x1E, //     (ditto)
      0x22, 0x1D, 0x18, 0x1E,
      0x1B, 0x1A, 0x24, 0x2B,
      0x06, 0x06, 0x02, 0x0F,
      10,                     //     10 ms delay
    ST7735_CASET  , 4      ,  // 15: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 2
      0x00, 0x81,             //     XEND = 129
    ST7735_RASET  , 4      ,  // 16: Row addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 1
      0x00, 0x81,             //     XEND = 160
    ST7735_NORON  ,   DELAY,  // 17: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,   DELAY,  // 18: Main screen turn on, no args, w/delay
      255 },                  //     255 = 500 ms delay

  Rcmd1[] = {                 // Init for 7735R, part 1 (red or green tab)
    15,                       // 15 commands in list:
    ST7735_SWRESET,   DELAY,  //  1: Software reset, 0 args, w/delay
      150,                    //     150 ms delay
    ST7735_SLPOUT ,   DELAY,  //  2: Out of sleep mode, 0 args, w/delay
      255,                    //     500 ms delay
    ST7735_FRMCTR1, 3      ,  //  3: Frame rate ctrl - normal mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR2, 3      ,  //  4: Frame rate control - idle mode, 3 args:
      0x01, 0x2C, 0x2D,       //     Rate = fosc/(1x2+40) * (LINE+2C+2D)
    ST7735_FRMCTR3, 6      ,  //  5: Frame rate ctrl - partial mode, 6 args:
      0x01, 0x2C, 0x2D,       //     Dot inversion mode
      0x01, 0x2C, 0x2D,       //     Line inversion mode
    ST7735_INVCTR , 1      ,  //  6: Display inversion ctrl, 1 arg, no delay:
      0x07,                   //     No inversion
    ST7735_PWCTR1 , 3      ,  //  7: Power control, 3 args, no delay:
      0xA2,
      0x02,                   //     -4.6V
      0x84,                   //     AUTO mode
    ST7735_PWCTR2 , 1      ,  //  8: Power control, 1 arg, no delay:
      0xC5,                   //     VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
    ST7735_PWCTR3 , 2      ,  //  9: Power control, 2 args, no delay:
      0x0A,                   //     Opamp current small
      0x00,                   //     Boost frequency
    ST7735_PWCTR4 , 2      ,  // 10: Power control, 2 args, no delay:
      0x8A,                   //     BCLK/2, Opamp current small & Medium low
      0x2A,  
    ST7735_PWCTR5 , 2      ,  // 11: Power control, 2 args, no delay:
      0x8A, 0xEE,
    ST7735_VMCTR1 , 1      ,  // 12: Power control, 1 arg, no delay:
      0x0E,
    ST7735_INVOFF , 0      ,  // 13: Don't invert display, no args, no delay
    ST7735_MADCTL , 1      ,  // 14: Memory access control (directions), 1 arg:
      0xC8,                   //     row addr/col addr, bottom to top refresh
    ST7735_COLMOD , 1      ,  // 15: set color mode, 1 arg, no delay:
      0x05 },                 //     16-bit color

  Rcmd2green[] = {            // Init for 7735R, part 2 (green tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x02,             //     XSTART = 0
      0x00, 0x7F+0x02,        //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x01,             //     XSTART = 0
      0x00, 0x9F+0x01 },      //     XEND = 159
  Rcmd2red[] = {              // Init for 7735R, part 2 (red tab only)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x9F },           //     XEND = 159

  Rcmd2green144[] = {              // Init for 7735R, part 2 (green 1.44 tab)
    2,                        //  2 commands in list:
    ST7735_CASET  , 4      ,  //  1: Column addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F,             //     XEND = 127
    ST7735_RASET  , 4      ,  //  2: Row addr set, 4 args, no delay:
      0x00, 0x00,             //     XSTART = 0
      0x00, 0x7F },           //     XEND = 127

  Rcmd3[] = {                 // Init for 7735R, part 3 (red or green tab)
    4,                        //  4 commands in list:
    ST7735_GMCTRP1, 16      , //  1: Magical unicorn dust, 16 args, no delay:
      0x02, 0x1c, 0x07, 0x12,
      0x37, 0x32, 0x29, 0x2d,
      0x29, 0x25, 0x2B, 0x39,
      0x00, 0x01, 0x03, 0x10,
    ST7735_GMCTRN1, 16      , //  2: Sparkles and rainbows, 16 args, no delay:
      0x03, 0x1d, 0x07, 0x06,
      0x2E, 0x2C, 0x29, 0x2D,
      0x2E, 0x2E, 0x37, 0x3F,
      0x00, 0x00, 0x02, 0x10,
    ST7735_NORON  ,    DELAY, //  3: Normal display on, no args, w/delay
      10,                     //     10 ms delay
    ST7735_DISPON ,    DELAY, //  4: Main screen turn on, no args w/delay
      100 };                  //     100 ms delay


// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void ATX32_GLCD_ST7735::commandList(const uint8_t *addr) {

  uint8_t  numCommands, numArgs;
  uint16_t ms;

  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  while(numCommands--) {                 // For each command...
    writecommand(pgm_read_byte(addr++)); //   Read, issue command
    numArgs  = pgm_read_byte(addr++);    //   Number of args to follow
    ms       = numArgs & DELAY;          //   If hibit set, delay follows args
    numArgs &= ~DELAY;                   //   Mask out delay bit
    while(numArgs--) {                   //   For each argument...
      writedata(pgm_read_byte(addr++));  //     Read, issue argument
    }

    if(ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      delay(ms);
    }
  }
}


// Initialization code common to both 'B' and 'R' type displays
void ATX32_GLCD_ST7735::commonInit(const uint8_t *cmdList) {
  colstart  = rowstart = 0; // May be overridden in init func

  pinMode(_rs, OUTPUT);
  pinMode(_cs, OUTPUT);
  csport    = portOutputRegister(digitalPinToPort(_cs));
  rsport    = portOutputRegister(digitalPinToPort(_rs));
  cspinmask = digitalPinToBitMask(_cs);
  rspinmask = digitalPinToBitMask(_rs);

  if(hwSPI) { // Using hardware SPI
#if defined (SPI_HAS_TRANSACTION)
    SPI_3.begin();
    mySPISettings = SPISettings(8000000, MSBFIRST, SPI_MODE0);
#elif defined (__AVR__)
    SPCRbackup = SPCR;
    SPI_3.begin();
    SPI_3.setClockDivider(SPI_CLOCK_DIV4);
    SPI_3.setDataMode(SPI_MODE0);
    mySPCR = SPCR; // save our preferred state
    //Serial.print("mySPCR = 0x"); Serial.println(SPCR, HEX);
    SPCR = SPCRbackup;  // then restore
#elif defined (__SAM3X8E__)
    SPI_3.begin();
    SPI_3.setClockDivider(21); //4MHz
    SPI_3.setDataMode(SPI_MODE0);
#elif defined (__STM32F1__)||(__ATX32__)
    SPI_3.begin();
    SPI_3.setClockDivider(SPI_CLOCK_DIV2);
    SPI_3.setDataMode(SPI_MODE0);
    SPI_3.setBitOrder(MSBFIRST);
#endif
  } else {
    pinMode(_sclk, OUTPUT);
    pinMode(_sid , OUTPUT);
    clkport     = portOutputRegister(digitalPinToPort(_sclk));
    dataport    = portOutputRegister(digitalPinToPort(_sid));
    clkpinmask  = digitalPinToBitMask(_sclk);
    datapinmask = digitalPinToBitMask(_sid);
    *clkport   &= ~clkpinmask;
    *dataport  &= ~datapinmask;
  }

  // toggle RST low to reset; CS low so it'll listen to us
  *csport &= ~cspinmask;
  if (_rst) {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, HIGH);
    delay(500);
    digitalWrite(_rst, LOW);
    delay(500);
    digitalWrite(_rst, HIGH);
    delay(500);
  }

  if(cmdList) commandList(cmdList);
  madctl = 0xc8;
}


// Initialization for ST7735B screens
void ATX32_GLCD_ST7735::initB(void) {
  commonInit(Bcmd);
}


// Initialization for ST7735R screens (green or red tabs)
void ATX32_GLCD_ST7735::initR(uint8_t options) {
  commonInit(Rcmd1);
  
 
  if(options == INITR_GREENTAB) {
    commandList(Rcmd2green);
    colstart = 2;
    rowstart = 1;
  } else {
    // colstart, rowstart left at default '0' values
    commandList(Rcmd2red);
  }
  commandList(Rcmd3);

  // if black, change MADCTL color filter
  if (options == INITR_BLACKTAB) {
    writecommand(ST7735_MADCTL);
    writedata(0xC0);
    madctl = 0xc0;
  }

  tabcolor = options;
}


void ATX32_GLCD_ST7735::setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1,
 uint8_t y1) {

  if (hwSPI) {
#if defined (__STM32F1__)||(__ATX32__)
    writecommand(ST7735_CASET);
    *rsport |=  rspinmask;
    *csport &= ~cspinmask;
    SPI_3.setDataSize (SPI_CR1_DFF);
    SPI_3.write(x0+colstart);
    SPI_3.write(x1+colstart);
  
    writecommand(ST7735_RASET);
    *rsport |=  rspinmask;
    *csport &= ~cspinmask;
  
    SPI_3.write(y0+rowstart);
    SPI_3.write(y1+rowstart);
    SPI_3.setDataSize(0);
  
    writecommand(ST7735_RAMWR);
 #endif 
  } else {    
    writecommand(ST7735_CASET); // Column addr set
    writedata(0x00);
    writedata(x0+colstart);     // XSTART 
    writedata(0x00);
    writedata(x1+colstart);     // XEND
  
    writecommand(ST7735_RASET); // Row addr set
    writedata(0x00);
    writedata(y0+rowstart);     // YSTART
    writedata(0x00);
    writedata(y1+rowstart);     // YEND
  
    writecommand(ST7735_RAMWR); // write to RAM
  } // end else
}


void ATX32_GLCD_ST7735::pushColor(uint16_t color) {
#if defined (SPI_HAS_TRANSACTION)
    SPI_3.beginTransaction(mySPISettings);
#endif
  *rsport |=  rspinmask;
  *csport &= ~cspinmask;

  spiwrite(color >> 8);
  spiwrite(color);

  *csport |= cspinmask;
#if defined (SPI_HAS_TRANSACTION)
    SPI_3.endTransaction();
#endif
}

void ATX32_GLCD_ST7735::drawPixel(int16_t x, int16_t y, uint16_t color) {

  if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

  setAddrWindow(x,y,x+1,y+1);

#if defined (SPI_HAS_TRANSACTION)
    SPI_3.beginTransaction(mySPISettings);
#endif
  *rsport |=  rspinmask;
  *csport &= ~cspinmask;
  
  spiwrite(color >> 8);
  spiwrite(color);

  *csport |= cspinmask;
#if defined (SPI_HAS_TRANSACTION)
    SPI_3.endTransaction();
#endif
}


void ATX32_GLCD_ST7735::drawFastVLine(int16_t x, int16_t y, int16_t h,
 uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((y+h-1) >= _height) h = _height-y;
  setAddrWindow(x, y, x, y+h-1);
    
#if defined (SPI_HAS_TRANSACTION)
    SPI_3.beginTransaction(mySPISettings);
#endif
  *rsport |=  rspinmask;
  *csport &= ~cspinmask;

  if (hwSPI) {
#if defined (__STM32F1__)||(__ATX32__)
    SPI_3.setDataSize (SPI_CR1_DFF); // Set SPI 16bit mode
    lineBuffer[0] = color;
    SPI_3.dmaSend(lineBuffer, h, 0);
    SPI_3.setDataSize (0);
#endif
  } else {
    uint8_t hi = color >> 8, lo = color;
    while (h--) {
      spiwrite(hi);
      spiwrite(lo);
    } // end while
  } // end else  
  
  *csport |= cspinmask;
#if defined (SPI_HAS_TRANSACTION)
    SPI_3.endTransaction();
#endif
}


void ATX32_GLCD_ST7735::drawFastHLine(int16_t x, int16_t y, int16_t w,
  uint16_t color) {

  // Rudimentary clipping
  if((x >= _width) || (y >= _height)) return;
  if((x+w-1) >= _width)  w = _width-x;
  setAddrWindow(x, y, x+w-1, y);

#if defined (SPI_HAS_TRANSACTION)
    SPI_3.beginTransaction(mySPISettings);
#endif
  *rsport |=  rspinmask;
  *csport &= ~cspinmask;
  
  if (hwSPI) {
#if defined (__STM32F1__)||(__ATX32__)
    SPI_3.setDataSize (SPI_CR1_DFF); // Set spi 16bit mode
    lineBuffer[0] = color;
    SPI_3.dmaSend(lineBuffer, w, 0);
    SPI_3.setDataSize (0);
#endif
  } else {    
    uint8_t hi = color >> 8, lo = color;
      while (w--) {
      spiwrite(hi);
      spiwrite(lo); 
      }
  } // end else
  *csport |= cspinmask;
#if defined (SPI_HAS_TRANSACTION)
    SPI_3.endTransaction();
#endif
}

// bresenham's algorithm - thx wikpedia
/*void ATX32_GLCD_ST7735::drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, 
		      uint16_t color) {
  uint16_t steep = abs(y1 - y0) > abs(x1 - x0);
  if (steep) {
    swap(x0, y0);
    swap(x1, y1);
  }

  if (x0 > x1) {
    swap(x0, x1);
    swap(y0, y1);
  }

  uint16_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int16_t err = dx / 2;
  int16_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;}

  for (; x0<=x1; x0++) {
    if (steep) {
      drawPixel(y0, x0, color);
    } else {
      drawPixel(x0, y0, color);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}
*/

void ATX32_GLCD_ST7735::drawLine(int16_t x0, int16_t y0,int16_t x1, int16_t y1, uint16_t color)
{
	if ((y0 < 0 && y1 <0) || (y0 > _height && y1 > _height)) return;
	if ((x0 < 0 && x1 <0) || (x0 > _width && x1 > _width)) return;
	if (x0 < 0) x0 = 0;
	if (x1 < 0) x1 = 0;
	if (y0 < 0) y0 = 0;
	if (y1 < 0) y1 = 0;

	if (y0 == y1) {
		if (x1 > x0) {
			drawFastHLine(x0, y0, x1 - x0 + 1, color);
		}
		else if (x1 < x0) {
			drawFastHLine(x1, y0, x0 - x1 + 1, color);
		}
		else {
			drawPixel(x0, y0, color);
		}
		return;
	}
	else if (x0 == x1) {
		if (y1 > y0) {
			drawFastVLine(x0, y0, y1 - y0 + 1, color);
		}
		else {
			drawFastVLine(x0, y1, y0 - y1 + 1, color);
		}
		return;
	}

	bool steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		swap(x0, y0);
		swap(x1, y1);
	}
	if (x0 > x1) {
		swap(x0, x1);
		swap(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1) {
		ystep = 1;
	}
	else {
		ystep = -1;
	}

	int16_t xbegin = x0;
	lineBuffer[0] = color;
//	*csport &= ~cspinmask;
	if (steep) {
		for (; x0 <= x1; x0++) {
			err -= dy;
			if (err < 0) {
				int16_t len = x0 - xbegin;
				if (len) {
					drawFastVLine (y0, xbegin, len + 1, color);
					//writeVLine_cont_noCS_noFill(y0, xbegin, len + 1);
				}
				else {
					drawPixel(y0, x0, color);
					//writePixel_cont_noCS(y0, x0, color);
				}
				xbegin = x0 + 1;
				y0 += ystep;
				err += dx;
			}
		}
		if (x0 > xbegin + 1) {
			//writeVLine_cont_noCS_noFill(y0, xbegin, x0 - xbegin);
			drawFastVLine(y0, xbegin, x0 - xbegin, color);
		}

	}
	else {
		for (; x0 <= x1; x0++) {
			err -= dy;
			if (err < 0) {
				int16_t len = x0 - xbegin;
				if (len) {
					drawFastHLine(xbegin, y0, len + 1, color);
					//writeHLine_cont_noCS_noFill(xbegin, y0, len + 1);
				}
				else {
					drawPixel(x0, y0, color);
					//writePixel_cont_noCS(x0, y0, color);
				}
				xbegin = x0 + 1;
				y0 += ystep;
				err += dx;
			}
		}
		if (x0 > xbegin + 1) {
			//writeHLine_cont_noCS_noFill(xbegin, y0, x0 - xbegin);
			drawFastHLine(xbegin, y0, x0 - xbegin, color);
		}
	}
//	*csport |= cspinmask;
}

void ATX32_GLCD_ST7735::drawString(uint8_t x, uint8_t y, char *c, 
			uint16_t color,uint16_t bgcolor, uint8_t size) {
	x = x*size;
	y = y*size+1;
  while (c[0] != 0) {
    drawChar(x, y, c[0], color,bgcolor, size);
    x += size*6;
    c++;
    if ((x + (size*5)) >= _width) {
//      y += 10;
      y += size*10;
      x = 0;
    }
  }
}
// draw a character
void ATX32_GLCD_ST7735::drawChar(uint8_t x, uint8_t y, char c, 
		      uint16_t color, uint16_t bgcolor,uint8_t size) {
  for (uint8_t i =0; i<5; i++ ) {
    uint8_t line = pgm_read_byte(font+(c*5)+i);
    for (uint8_t j = 0; j<8; j++) {
      if (line & 0x1) 
      {
	     if (size == 1) // default size
		 {
	        drawPixel(x+i, y+j, color);
		 }
	     else 
         {  // big size
	        fillRect(x+(i*size), y+(j*size), size, size, color);
	     } 
      }
      else  // Addition test
      {
         if (size == 1) // default size
		{
	      drawPixel(x+i, y+j, bgcolor);
			}
	     else 
         {  // big size
	        fillRect(x+(i*size), y+(j*size), size, size, bgcolor);
	     } 
      }
      line >>= 1;
    }
  }
   for (uint8_t j = 0; j<8; j++)      // draw space vertical line 
   {
         if (size == 1) // default size
		 {
	        drawPixel(x+5, y+j, bgcolor);
		 }
	     else 
         {  // big size
	        fillRect(x+(5*size), y+(j*size), size, size, bgcolor);
	     } 

   }
}

void ATX32_GLCD_ST7735::drawStringTransparent(uint8_t x, uint8_t y, char *c, 
			uint16_t color, uint8_t size) {
	x = x*size;
	y = y*size+1;
  while (c[0] != 0) {
    drawCharTransparent(x, y, c[0], color, size);
    x += size*6;
    c++;
    if ((x + (size*5)) >= _width) {
//      y += 10;
      y += size*10;
      x = 0;
    }
  }
}
// draw a character transparent mode
void ATX32_GLCD_ST7735::drawCharTransparent(uint8_t x, uint8_t y, char c, 
		      uint16_t color,uint8_t size) {
  for (uint8_t i =0; i<5; i++ ) {
    uint8_t line = pgm_read_byte(font+(c*5)+i);
    for (uint8_t j = 0; j<8; j++) {
      if (line & 0x1) 
      {
	     if (size == 1) // default size
		 {
	        drawPixel(x+i, y+j, color);
		 }
	     else 
         {  // big size
	        fillRect(x+(i*size), y+(j*size), size, size, color);
	     } 
      }
      line >>= 1;
    }
  }
}

// fill a circle
void ATX32_GLCD_ST7735::fillCircle(uint8_t x0, uint8_t y0, uint8_t r, uint16_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  drawFastVLine(x0, y0-r, 2*r+1, color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
  
    drawFastVLine(x0+x, y0-y, 2*y+1, color);
    drawFastVLine(x0-x, y0-y, 2*y+1, color);
    drawFastVLine(x0+y, y0-x, 2*x+1, color);
    drawFastVLine(x0-y, y0-x, 2*x+1, color);
  }
}

// draw a circle outline
void ATX32_GLCD_ST7735::drawCircle(uint8_t x0, uint8_t y0, uint8_t r, 
			uint16_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  drawPixel(x0, y0+r, color);
  drawPixel(x0, y0-r, color);
  drawPixel(x0+r, y0, color);
  drawPixel(x0-r, y0, color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;
  
    drawPixel(x0 + x, y0 + y, color);
    drawPixel(x0 - x, y0 + y, color);
    drawPixel(x0 + x, y0 - y, color);
    drawPixel(x0 - x, y0 - y, color);
    
    drawPixel(x0 + y, y0 + x, color);
    drawPixel(x0 - y, y0 + x, color);
    drawPixel(x0 + y, y0 - x, color);
    drawPixel(x0 - y, y0 - x, color);
    
  }
}

uint16_t __q2Pow(uint16_t radius)
{
  uint16_t __q2PowResult=1;
  uint16_t i; 
  if (radius!=0)
  {
	for (i=0;i<radius;i++)
	{
		__q2PowResult=__q2PowResult<<1;
	}
  }
  return __q2PowResult;
}

uint16_t __qSqrt(uint16_t rSqr)
{
  uint16_t __qSqrtTrialNumber=128;
  uint16_t __qSqrtPowTrialNumber;
  uint8_t __qSqrtTrialCounter=6;
  uint8_t i;
  if (rSqr < 0)
  {
	return 0;
  }
  else
  {
	for (i=0;i<7;i++)
	{
		__qSqrtPowTrialNumber=__qSqrtTrialNumber*__qSqrtTrialNumber;
		if ((__qSqrtPowTrialNumber)==rSqr)
		{
			return __qSqrtTrialNumber;
		}
		else
		{
			if ((__qSqrtPowTrialNumber)>rSqr)
			{
				__qSqrtTrialNumber=__qSqrtTrialNumber-__q2Pow(__qSqrtTrialCounter);
			}
			else
			{
				__qSqrtTrialNumber=__qSqrtTrialNumber+__q2Pow(__qSqrtTrialCounter);
			}
			__qSqrtTrialCounter--;			
		}
	}
	if ((__qSqrtTrialNumber*__qSqrtTrialNumber) > rSqr)
	{
	  __qSqrtTrialNumber=__qSqrtTrialNumber-1;
	}
	return __qSqrtTrialNumber;
  }
}

void ATX32_GLCD_ST7735::drawArc(uint8_t h,uint8_t k,uint8_t r,int16_t start_angle,int16_t end_angle,uint16_t color)
{
	  int16_t angle,angleTmp;
	  int16_t x;//,y;
	  float y;
	  angle=start_angle;

      while(angle<=end_angle)
      {
		y=(r*(sin((angle%180)*(M_PI/180))));
		x=(__qSqrt((r*r)-(y*y)));
		angleTmp=angle%360;
		if (angleTmp<90)
		{
		  drawPixel((int)(h+x),(int)(k-y),color);
		}
		else if (angleTmp<180)
		{
		  drawPixel((int)(h-x),(int)(k-y),color);
		}
		else if (angleTmp<270)
		{
		  drawPixel((int)(h-x),(int)(k+y),color);
		}
		else// if (angleTmp<360)
		{
		  drawPixel((int)(h+x),(int)(k+y),color);
		}
		
	
        angle++;
      }
}
/*
void ATX32_GLCD_ST7735::drawPie(uint8_t h,uint8_t k,uint8_t r,int16_t start_angle,int16_t end_angle,uint16_t color)
{
	  int16_t angle,angleTmp;
	  int16_t x;//,y;
	  int16_t radius;
	  float y;
 	  angle=start_angle;

      while(angle<=end_angle)
      {
		y=(r*(sin((angle%180)*(M_PI/180))));
		x=(__qSqrt((r*r)-(y*y)));
		angleTmp=angle%360;
		if (angleTmp<90)
		{
		  drawLine(h,k,(int)(h+x),(int)(k-y),color);
		}
		else if (angleTmp<180)
		{
		  drawLine(h,k,(int)(h-x),(int)(k-y),color);
		}
		else if (angleTmp<270)
		{
		  drawLine(h,k,(int)(h-x),(int)(k+y),color);
		}
		else// if (angleTmp<360)
		{
		  drawLine(h,k,(int)(h+x),(int)(k+y),color);
		}
        angle++;
      }

	  radius=0;
	  while(++radius<=r)
	  {
	  angle=start_angle;
       while(angle<=end_angle)
       {
		y=(radius*(sin((angle%180)*(M_PI/180))));
		x=(__qSqrt((radius*radius)-(y*y)));
		angleTmp=angle%360;
		if (angleTmp<90)
		{
		  drawPixel((int)(h+x),(int)(k-y),color);
		}
		else if (angleTmp<180)
		{
		  drawPixel((int)(h-x),(int)(k-y),color);
		}
		else if (angleTmp<270)
		{
		  drawPixel((int)(h-x),(int)(k+y),color);
		}
		else// if (angleTmp<360)
		{
		  drawPixel((int)(h+x),(int)(k+y),color);
		}
        angle++;
	   }
      }
}*/

/*void ATX32_GLCD_ST7735::drawPie(uint8_t h,uint8_t k,uint8_t r,int16_t start_angle,int16_t end_angle,uint16_t color)
{
	  int16_t angle,angleTmp;
	  int16_t x;//,y;
	  float y;
	  angle=start_angle;

      while(angle<=end_angle)
      {
		y=(r*(sin((angle%180)*(M_PI/180))));
		x=(__qSqrt((r*r)-(y*y)));
		angleTmp=angle%360;
		if (angleTmp<90)
		{
		  drawLine(h,k,(int)(h+x),(int)(k-y),color);
		}
		else if (angleTmp<180)
		{
		  drawLine(h,k,(int)(h-x),(int)(k-y),color);
		}
		else if (angleTmp<270)
		{
		  drawLine(h,k,(int)(h-x),(int)(k+y),color);
		}
		else// if (angleTmp<360)
		{
		  drawLine(h,k,(int)(h+x),(int)(k+y),color);
		}
        angle++;
      }
}
*/
/*
void ATX32_GLCD_ST7735::drawArc(uint8_t h,uint8_t k,uint8_t r,int16_t start_angle,int16_t end_angle,uint16_t color)
{

       float angle=(((start_angle<=end_angle)?start_angle:end_angle)*(M_PI/180));
       float range=(((end_angle>start_angle)?end_angle:start_angle)*(M_PI/180));

       float x=(r*cos(angle));
       float y=(r*sin(angle));

      while(angle<=range)
      {
         drawPixel((int)(h+x+0.5),(int)(k-y+0.5),color);

         angle+=0.002;

         x=(r*cos(angle));
         y=(r*sin(angle));
      }
}
*/
/*void ATX32_GLCD_ST7735::drawPie(uint8_t h,uint8_t k,uint8_t r,int16_t start_angle,int16_t end_angle,uint16_t color)
{

       float angle=(((start_angle<=end_angle)?start_angle:end_angle)*(M_PI/180));
       float range=(((end_angle>start_angle)?end_angle:start_angle)*(M_PI/180));

       float x=(r*cos(angle));
       float y=(r*sin(angle));

      while(angle<=range)
      {
         drawLine(h,k,(int)(h+x+0.5),(int)(k-y+0.5),color);
         angle+=0.0005;

         x=(r*cos(angle));
         y=(r*sin(angle));
      }
}
*/

void ATX32_GLCD_ST7735::fillScreen(uint16_t color) {  
  if (hwSPI) {
#if defined (__STM32F1__)||(__ATX32__)
    setAddrWindow(0, 0, _width - 1, _height - 1);
    
    *rsport |=  rspinmask;
    *csport &= ~cspinmask;
    SPI_3.setDataSize (SPI_CR1_DFF); // Set spi 16bit mode
    lineBuffer[0] = color;
    SPI_3.dmaSend(lineBuffer, (65535), 0);
    SPI_3.dmaSend(lineBuffer, ((_width * _height) - 65535), 0);
    SPI_3.setDataSize (0);  
#endif
  } else {
    fillRect(0, 0,  _width, _height, color);
  } // end else 
}

// draw a rectangle
void ATX32_GLCD_ST7735::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, 
		      uint16_t color) {
  // smarter version
  drawFastHLine(x, y, w, color);
  drawFastHLine(x, y+h-1, w, color);
  drawFastVLine(x, y, h, color);
  drawFastVLine(x+w-1, y, h, color);
}

// fill a rectangle
void ATX32_GLCD_ST7735::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
  uint16_t color) {

  // rudimentary clipping (drawChar w/big text requires this)
  if((x >= _width) || (y >= _height)) return;
  if((x + w - 1) >= _width)  w = _width  - x;
  if((y + h - 1) >= _height) h = _height - y;

  setAddrWindow(x, y, x+w-1, y+h-1);
    
#if defined (SPI_HAS_TRANSACTION)
    SPI_3.beginTransaction(mySPISettings);
#endif
  *rsport |=  rspinmask;
  *csport &= ~cspinmask;

  if (hwSPI) {
#if defined (__STM32F1__)
    SPI_3.setDataSize (SPI_CR1_DFF); // Set spi 16bit mode
    lineBuffer[0] = color;
    if (w*h <= 65535) {
    SPI_3.dmaSend(lineBuffer, (w*h), 0);
    }
    else {
    SPI_3.dmaSend(lineBuffer, (65535), 0);
    SPI_3.dmaSend(lineBuffer, ((w*h) - 65535), 0);
    }
    SPI_3.setDataSize (0);
#endif
  } else {    
    uint8_t hi = color >> 8, lo = color;
    for(y=h; y>0; y--) {
      for(x=w; x>0; x--) {
        spiwrite(hi);
        spiwrite(lo); 
      } // end for
    } // end for
  } // end else
  *csport |= cspinmask;
#if defined (SPI_HAS_TRANSACTION)
    SPI_3.endTransaction();
#endif
}


// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t ATX32_GLCD_ST7735::Color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04


uint8_t ATX32_GLCD_ST7735::getRotation() {
  return madctl;
}

/*void ATX32_GLCD_ST7735::setRotation(uint8_t m) {

  writecommand(ST7735_MADCTL);
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
   case 0:
     if (tabcolor == INITR_BLACKTAB) {
       writedata(MADCTL_MX | MADCTL_MY | MADCTL_RGB);
       madctl = 0xc0;
     } else {
       writedata(MADCTL_MX | MADCTL_MY | MADCTL_BGR);
       madctl = 0xc8;
     }
     _width  = ST7735_TFTWIDTH;

     if (tabcolor == INITR_144GREENTAB) 
       _height = ST7735_TFTHEIGHT_144;
     else
       _height = ST7735_TFTHEIGHT_18;

     break;
   case 1:
     if (tabcolor == INITR_BLACKTAB) {
       writedata(MADCTL_MY | MADCTL_MV | MADCTL_RGB);
     } else {
       writedata(MADCTL_MY | MADCTL_MV | MADCTL_BGR);
     }

     if (tabcolor == INITR_144GREENTAB) 
       _width = ST7735_TFTHEIGHT_144;
     else
       _width = ST7735_TFTHEIGHT_18;

     _height = ST7735_TFTWIDTH;
     break;
  case 2:
     if (tabcolor == INITR_BLACKTAB) {
       writedata(MADCTL_RGB);
     } else {
       writedata(MADCTL_BGR);
     }
     _width  = ST7735_TFTWIDTH;
     if (tabcolor == INITR_144GREENTAB) 
       _height = ST7735_TFTHEIGHT_144;
     else
       _height = ST7735_TFTHEIGHT_18;

    break;
   case 3:
     if (tabcolor == INITR_BLACKTAB) {
       writedata(MADCTL_MX | MADCTL_MV | MADCTL_RGB);
     } else {
       writedata(MADCTL_MX | MADCTL_MV | MADCTL_BGR);
     }
     if (tabcolor == INITR_144GREENTAB) 
       _width = ST7735_TFTHEIGHT_144;
     else
       _width = ST7735_TFTHEIGHT_18;

     _height = ST7735_TFTWIDTH;
     break;
  }
}
*/
void ATX32_GLCD_ST7735::setRotationReg(uint8_t m) {
  madctl = m;
  writecommand(GCLCD_MADCTL);  // memory access control (directions)
  writedata(madctl);  // row address/col address, bottom to top refresh
}

void ATX32_GLCD_ST7735::setRotation(uint8_t m) {
  switch (m>>4)
  {
	case 0x0c:
			madctl=(MADCTL_MX | MADCTL_BGR);
			_width = 128;
			_height = 160;
			break;
	case 0x0a:
			madctl=(MADCTL_MV | MADCTL_BGR);
			_width = 160;
			_height = 128;
			break;
	case 0x01:
			madctl=(MADCTL_MY | MADCTL_BGR);
			_width = 128;
			_height = 160;
			break;
	case 0x07:
			madctl=(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
			_width = 160;
			_height = 128;
			break;
  }

  writecommand(GCLCD_MADCTL);  // memory access control (directions)
  writedata(madctl);  // row address/col address, bottom to top refresh
 }


void ATX32_GLCD_ST7735::invertDisplay(boolean i) {
  writecommand(i ? ST7735_INVON : ST7735_INVOFF);
}


////////// stuff not actively being used, but kept for posterity
/*

 uint8_t ATX32_GLCD_ST7735::spiread(void) {
 uint8_t r = 0;
 if (_sid > 0) {
 r = shiftIn(_sid, _sclk, MSBFIRST);
 } else {
 //SID_DDR &= ~_BV(SID);
 //int8_t i;
 //for (i=7; i>=0; i--) {
 //  SCLK_PORT &= ~_BV(SCLK);
 //  r <<= 1;
 //  r |= (SID_PIN >> SID) & 0x1;
 //  SCLK_PORT |= _BV(SCLK);
 //}
 //SID_DDR |= _BV(SID);
 
 }
 return r;
 }
 
 
 void ATX32_GLCD_ST7735::dummyclock(void) {
 
 if (_sid > 0) {
 digitalWrite(_sclk, LOW);
 digitalWrite(_sclk, HIGH);
 } else {
 // SCLK_PORT &= ~_BV(SCLK);
 //SCLK_PORT |= _BV(SCLK);
 }
 }
 uint8_t ATX32_GLCD_ST7735::readdata(void) {
 *portOutputRegister(rsport) |= rspin;
 
 *portOutputRegister(csport) &= ~ cspin;
 
 uint8_t r = spiread();
 
 *portOutputRegister(csport) |= cspin;
 
 return r;
 
 } 
 
 uint8_t ATX32_GLCD_ST7735::readcommand8(uint8_t c) {
 digitalWrite(_rs, LOW);
 
 *portOutputRegister(csport) &= ~ cspin;
 
 spiwrite(c);
 
 digitalWrite(_rs, HIGH);
 pinMode(_sid, INPUT); // input!
 digitalWrite(_sid, LOW); // low
 spiread();
 uint8_t r = spiread();
 
 
 *portOutputRegister(csport) |= cspin;
 
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 
 uint16_t ATX32_GLCD_ST7735::readcommand16(uint8_t c) {
 digitalWrite(_rs, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 uint16_t r = spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 uint32_t ATX32_GLCD_ST7735::readcommand32(uint8_t c) {
 digitalWrite(_rs, LOW);
 if (_cs)
 digitalWrite(_cs, LOW);
 spiwrite(c);
 pinMode(_sid, INPUT); // input!
 
 dummyclock();
 dummyclock();
 
 uint32_t r = spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 r <<= 8;
 r |= spiread();
 if (_cs)
 digitalWrite(_cs, HIGH);
 
 pinMode(_sid, OUTPUT); // back to output
 return r;
 }
 
 */
