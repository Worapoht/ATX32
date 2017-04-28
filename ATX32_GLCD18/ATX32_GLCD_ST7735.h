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

#ifndef _ATX32_GLCD_ST7735_H_STM_
#define _ATX32_GLCD_ST7735_H_STM_

#if ARDUINO >= 100
 #include "Arduino.h"
 #include "Print.h"
#else
 #include "WProgram.h"
#endif

#if defined (__STM32F1__)||(__ATX32__)
  #include <Adafruit_GFX_AS.h>    // Core graphics library
#endif
#if !defined (__STM32F1__)
  #include <Adafruit_GFX.h>    // Core graphics library
#endif

#if defined(__SAM3X8E__)
  #include <include/pio.h>
  #define PROGMEM
  #define pgm_read_byte(addr) (*(const unsigned char *)(addr))
  #define pgm_read_word(addr) (*(const unsigned short *)(addr))
  typedef unsigned char prog_uchar;
#elif defined(__AVR__)
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif

#if defined(__SAM3X8E__)
    #undef __FlashStringHelper::F(string_literal)
    #define F(string_literal) string_literal
#endif

// some flags for initR() :(
#define INITR_GREENTAB 0x0
#define INITR_REDTAB   0x1
#define INITR_BLACKTAB   0x2

#define INITR_18GREENTAB    INITR_GREENTAB
#define INITR_18REDTAB      INITR_REDTAB
#define INITR_18BLACKTAB    INITR_BLACKTAB
#define INITR_144GREENTAB   0x1

#define ST7735_TFTWIDTH  128
// for 1.44" display
#define ST7735_TFTHEIGHT_144 128
// for 1.8" display
#define ST7735_TFTHEIGHT_18  160

#define GCLCD_NOP 0x0
#define GCLCD_SWRESET 0x01
#define GCLCD_RDDID 0x04
#define GCLCD_RDDST 0x09

#define GCLCD_SLPIN  0x10
#define GCLCD_SLPOUT  0x11
#define GCLCD_PTLON  0x12
#define GCLCD_NORON  0x13

#define GCLCD_INVOFF 0x20
#define GCLCD_INVON 0x21

#define GCLCD_DISPOFF 0x28
#define GCLCD_DISPON 0x29
#define GCLCD_CASET 0x2A
#define GCLCD_RASET 0x2B
#define GCLCD_RAMWR 0x2C
#define GCLCD_RAMRD 0x2E

#define GCLCD_COLMOD 0x3A
#define GCLCD_MADCTL 0x36


#define GCLCD_FRMCTR1 0xB1
#define GCLCD_FRMCTR2 0xB2
#define GCLCD_FRMCTR3 0xB3
#define GCLCD_INVCTR 0xB4
#define GCLCD_DISSET5 0xB6

#define GCLCD_PWCTR1 0xC0
#define GCLCD_PWCTR2 0xC1
#define GCLCD_PWCTR3 0xC2
#define GCLCD_PWCTR4 0xC3
#define GCLCD_PWCTR5 0xC4
#define GCLCD_VMCTR1 0xC5

#define GCLCD_RDID1 0xDA
#define GCLCD_RDID2 0xDB
#define GCLCD_RDID3 0xDC
#define GCLCD_RDID4 0xDD

#define GCLCD_PWCTR6 0xFC

#define GCLCD_GMCTRP1 0xE0
#define GCLCD_GMCTRN1 0xE1

#define ST7735_NOP     0x00
#define ST7735_SWRESET 0x01
#define ST7735_RDDID   0x04
#define ST7735_RDDST   0x09

#define ST7735_SLPIN   0x10
#define ST7735_SLPOUT  0x11
#define ST7735_PTLON   0x12
#define ST7735_NORON   0x13

#define ST7735_INVOFF  0x20
#define ST7735_INVON   0x21
#define ST7735_DISPOFF 0x28
#define ST7735_DISPON  0x29
#define ST7735_CASET   0x2A
#define ST7735_RASET   0x2B
#define ST7735_RAMWR   0x2C
#define ST7735_RAMRD   0x2E

#define ST7735_PTLAR   0x30
#define ST7735_COLMOD  0x3A
#define ST7735_MADCTL  0x36

#define ST7735_FRMCTR1 0xB1
#define ST7735_FRMCTR2 0xB2
#define ST7735_FRMCTR3 0xB3
#define ST7735_INVCTR  0xB4
#define ST7735_DISSET5 0xB6

#define ST7735_PWCTR1  0xC0
#define ST7735_PWCTR2  0xC1
#define ST7735_PWCTR3  0xC2
#define ST7735_PWCTR4  0xC3
#define ST7735_PWCTR5  0xC4
#define ST7735_VMCTR1  0xC5

#define ST7735_RDID1   0xDA
#define ST7735_RDID2   0xDB
#define ST7735_RDID3   0xDC
#define ST7735_RDID4   0xDD

#define ST7735_PWCTR6  0xFC

#define ST7735_GMCTRP1 0xE0
#define ST7735_GMCTRN1 0xE1

// Color definitions
#define	ST7735_BLACK   0x0000
#define	ST7735_BLUE    0x001F
#define	ST7735_RED     0xF800
#define	ST7735_GREEN   0x07E0
#define ST7735_CYAN    0x07FF
#define ST7735_MAGENTA 0xF81F
#define ST7735_YELLOW  0xFFE0
#define ST7735_WHITE   0xFFFF

uint16_t __qSqrt(uint16_t rSqr);
static unsigned char GCLCD_TransparentBackground=0; // Default=0 solid background, =1 transparent

class ATX32_GLCD_ST7735 : public Adafruit_GFX_AS {

 public:

  ATX32_GLCD_ST7735(int8_t CS, int8_t RS, int8_t RST = -1);

  void     initB(void),                             // for ST7735B displays
           initR(uint8_t options = INITR_GREENTAB), // for ST7735R
           setAddrWindow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1),
           pushColor(uint16_t color),
           fillScreen(uint16_t color),
           drawPixel(int16_t x, int16_t y, uint16_t color),
           drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color),
           drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color),
		   drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color),
           fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color),
		   drawCircle(uint8_t x0, uint8_t y0, uint8_t r, uint16_t color),
		   fillCircle(uint8_t x0, uint8_t y0, uint8_t r, uint16_t color),
		   drawArc(uint8_t h,uint8_t k,uint8_t r,int16_t start_angle,int16_t end_angle,uint16_t color),
//		   drawPie(uint8_t h,uint8_t k,uint8_t r,int16_t start_angle,int16_t end_angle,uint16_t color),

           setRotationReg(uint8_t m),
		   setRotation(uint8_t r),
           invertDisplay(boolean i),
           writecommand(uint8_t c),
           writedata(uint8_t d),
           commandList(const uint8_t *addr),
           commonInit(const uint8_t *cmdList);

		   uint8_t getRotation(void);
 
  void drawLine(int16_t x, int16_t y, int16_t x1, int16_t y1, uint16_t color);
  void drawString(uint8_t x, uint8_t y, char *c, uint16_t color,uint16_t bgcolor, uint8_t size=1);
  void drawChar(uint8_t x, uint8_t y, char c, uint16_t color,uint16_t bgcolor, uint8_t size=1);
  void drawStringTransparent(uint8_t x, uint8_t y, char *c, uint16_t color, uint8_t size=1);
  void drawCharTransparent(uint8_t x, uint8_t y, char c, uint16_t color, uint8_t size=1);
  
  uint16_t Color565(uint8_t r, uint8_t g, uint8_t b);

  /* These are not for current use, 8-bit protocol only!
  uint8_t  readdata(void),
           readcommand8(uint8_t);
  uint16_t readcommand16(uint8_t);
  uint32_t readcommand32(uint8_t);
  void     dummyclock(void);
  */

 private:
  uint8_t  tabcolor;

  void     spiwrite(uint8_t);
//uint8_t  spiread(void);
  uint8_t madctl;

  boolean  hwSPI;
  
#if defined (__STM32F1__)||(__ATX32__)
  volatile uint32 *dataport, *clkport, *csport, *rsport;
  uint32_t _cs, _rs, _rst, _sid, _sclk,
           datapinmask, clkpinmask, cspinmask, rspinmask,
           colstart, rowstart; // some displays need this changed
  uint16_t lineBuffer[ST7735_TFTHEIGHT_18]; // DMA buffer. 16bit color data per pixel
#elif defined(__AVR__) || defined(CORE_TEENSY)
  volatile uint8_t *dataport, *clkport, *csport, *rsport;
  uint8_t  _cs, _rs, _rst, _sid, _sclk,
           datapinmask, clkpinmask, cspinmask, rspinmask,
           colstart, rowstart; // some displays need this changed
#elif defined(__arm__)
  volatile RwReg  *dataport, *clkport, *csport, *rsport;
  uint32_t  _cs, _rs, _sid, _sclk,
            datapinmask, clkpinmask, cspinmask, rspinmask,
            colstart, rowstart; // some displays need this changed
  int32_t   _rst;  // Must use signed type since a -1 sentinel is assigned.
#endif

};

#endif
