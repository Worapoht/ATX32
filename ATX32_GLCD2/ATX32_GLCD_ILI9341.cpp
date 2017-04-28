/*
See rights and use declaration in License.h
This library has been modified for the Maple Mini.
Includes DMA transfers on DMA1 CH2 and CH3.
*/
#include <ATX32_GLCD_ILI9341.h>
#include "glcdfont.c"
#include <avr/pgmspace.h>
#include <limits.h>
#include <libmaple/dma.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h> // Using library SPI in folder: D:\Documents\Arduino\hardware\STM32\STM32F1XX\libraries\SPI

SPIClass SPI_3(3); //Create an instance of the SPI Class called SPI_3 that uses the 3rd SPI Port

// Constructor when using software SPI.  All output pins are configurable.
ATX32_GLCD_ILI9341::ATX32_GLCD_ILI9341(int8_t cs, int8_t dc, int8_t mosi,
    int8_t sclk, int8_t rst, int8_t miso) : Adafruit_GFX_AS(ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT) {
  _cs   = cs;
  _dc   = dc;
  _mosi  = mosi;
  _miso = miso;
  _sclk = sclk;
  _rst  = rst;
  hwSPI = false;
}


// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
ATX32_GLCD_ILI9341::ATX32_GLCD_ILI9341(int8_t cs, int8_t dc, int8_t rst) : Adafruit_GFX_AS(ILI9341_TFTWIDTH, ILI9341_TFTHEIGHT) {
  _cs   = cs;
  _dc   = dc;
  _rst  = rst;
  hwSPI = true;
  _mosi  = _sclk = 0;
}


void ATX32_GLCD_ILI9341::spiwrite(uint8_t c) {

  //Serial.print("0x"); Serial.print(c, HEX); Serial.print(", ");

  if (hwSPI)
  {
#if defined (__AVR__)
    uint8_t backupSPCR = SPCR;
    SPCR = mySPCR;
    SPDR = c;
    while (!(SPSR & _BV(SPIF)));
    SPCR = backupSPCR;
#elif defined(TEENSYDUINO)
    SPI_3.transfer(c);
#elif defined (__STM32F1__)||(__ATX32__)
    SPI_3.write(c);
#elif defined (__arm__)
    SPI_3.setClockDivider(11); // 8-ish MHz (full! speed!)
    SPI_3.setBitOrder(MSBFIRST);
    SPI_3.setDataMode(SPI_MODE0);
    SPI_3.transfer(c);

#endif
  } else {
    // Fast SPI bitbang swiped from LPD8806 library
    for (uint8_t bit = 0x80; bit; bit >>= 1) {
      if (c & bit) {
        //digitalWrite(_mosi, HIGH);
        *mosiport |=  mosipinmask;
      } else {
        //digitalWrite(_mosi, LOW);
        *mosiport &= ~mosipinmask;
      }
      //digitalWrite(_sclk, HIGH);
      *clkport |=  clkpinmask;
      //digitalWrite(_sclk, LOW);
      *clkport &= ~clkpinmask;
    }
  }
}


void ATX32_GLCD_ILI9341::writecommand(uint8_t c) {
  *dcport &=  ~dcpinmask;
  *csport &= ~cspinmask;

  spiwrite(c);

  *csport |= cspinmask;
}


void ATX32_GLCD_ILI9341::writedata(uint8_t c) {
  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;

  spiwrite(c);

  *csport |= cspinmask;
}

// If the SPI library has transaction support, these functions
// establish settings and protect from interference from other
// libraries.  Otherwise, they simply do nothing.
#ifdef SPI_HAS_TRANSACTION
static inline void spi_begin(void) __attribute__((always_inline));
static inline void spi_begin(void) {
#ifdef (__STM32F1__)||(__ATX32__)
  SPI_3.beginTransaction(SPISettings(36000000, MSBFIRST, SPI_MODE0));
#else
  SPI_3.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
#endif
}
static inline void spi_end(void) __attribute__((always_inline));
static inline void spi_end(void) {
  SPI_3.endTransaction();
}
#else
#define spi_begin()
#define spi_end()
#endif

// Rather than a bazillion writecommand() and writedata() calls, screen
// initialization commands and arguments are organized in these tables
// stored in PROGMEM.  The table may look bulky, but that's mostly the
// formatting -- storage-wise this is hundreds of bytes more compact
// than the equivalent code.  Companion function follows.
#define DELAY 0x80


// Companion code to the above tables.  Reads and issues
// a series of LCD commands stored in PROGMEM byte array.
void ATX32_GLCD_ILI9341::commandList(uint8_t *addr) {

  uint8_t  numCommands, numArgs;
  uint16_t ms;

  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  while (numCommands--) {                // For each command...
    writecommand(pgm_read_byte(addr++)); //   Read, issue command
    numArgs  = pgm_read_byte(addr++);    //   Number of args to follow
    ms       = numArgs & DELAY;          //   If hibit set, delay follows args
    numArgs &= ~DELAY;                   //   Mask out delay bit
    while (numArgs--) {                  //   For each argument...
      writedata(pgm_read_byte(addr++));  //     Read, issue argument
    }

    if (ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if (ms == 255) ms = 500;    // If 255, delay for 500 ms
      delay(ms);
    }
  }
}


void ATX32_GLCD_ILI9341::begin(void) {
  if (_rst > 0) {
    pinMode(_rst, OUTPUT);
    digitalWrite(_rst, LOW);
  }

  pinMode(_dc, OUTPUT);
  pinMode(_cs, OUTPUT);
  csport    = portOutputRegister(digitalPinToPort(_cs));
  cspinmask = digitalPinToBitMask(_cs);
  dcport    = portOutputRegister(digitalPinToPort(_dc));
  dcpinmask = digitalPinToBitMask(_dc);

  if (hwSPI) { // Using hardware SPI
#if defined (__AVR__)
    SPI_3.begin();
    SPI_3.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
    SPI_3.setBitOrder(MSBFIRST);
    SPI_3.setDataMode(SPI_MODE0);
    mySPCR = SPCR;
#elif defined(TEENSYDUINO)
    SPI_3.begin();
    SPI_3.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz (full! speed!)
    SPI_3.setBitOrder(MSBFIRST);
    SPI_3.setDataMode(SPI_MODE0);
#elif defined (__STM32F1__)||(__ATX32__)
    SPI_3.begin();
    SPI_3.setClockDivider(SPI_CLOCK_DIV2);
    SPI_3.setBitOrder(MSBFIRST);
    SPI_3.setDataMode(SPI_MODE0);

#elif defined (__arm__)
    SPI_3.begin();
    SPI_3.setClockDivider(11); // 8-ish MHz (full! speed!)
    SPI_3.setBitOrder(MSBFIRST);
    SPI_3.setDataMode(SPI_MODE0);
#endif
  } else {
    pinMode(_sclk, OUTPUT);
    pinMode(_mosi, OUTPUT);
    pinMode(_miso, INPUT);
    clkport     = portOutputRegister(digitalPinToPort(_sclk));
    clkpinmask  = digitalPinToBitMask(_sclk);
    mosiport    = portOutputRegister(digitalPinToPort(_mosi));
    mosipinmask = digitalPinToBitMask(_mosi);
    *clkport   &= ~clkpinmask;
    *mosiport  &= ~mosipinmask;
  }

  // toggle RST low to reset
  if (_rst > 0) {
    digitalWrite(_rst, HIGH);
    delay(5);
    digitalWrite(_rst, LOW);
    delay(20);
    digitalWrite(_rst, HIGH);
    delay(150);
  }

  /*
  uint8_t x = readcommand8(ILI9341_RDMODE);
  Serial.print("\nDisplay Power Mode: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDMADCTL);
  Serial.print("\nMADCTL Mode: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDPIXFMT);
  Serial.print("\nPixel Format: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDIMGFMT);
  Serial.print("\nImage Format: 0x"); Serial.println(x, HEX);
  x = readcommand8(ILI9341_RDSELFDIAG);
  Serial.print("\nSelf Diagnostic: 0x"); Serial.println(x, HEX);
  */
  //if(cmdList) commandList(cmdList);

  if (hwSPI) spi_begin();
  writecommand(0xEF);
  writedata(0x03);
  writedata(0x80);
  writedata(0x02);

  writecommand(0xCF);
  writedata(0x00);
  writedata(0XC1);
  writedata(0X30);

  writecommand(0xED);
  writedata(0x64);
  writedata(0x03);
  writedata(0X12);
  writedata(0X81);

  writecommand(0xE8);
  writedata(0x85);
  writedata(0x00);
  writedata(0x78);

  writecommand(0xCB);
  writedata(0x39);
  writedata(0x2C);
  writedata(0x00);
  writedata(0x34);
  writedata(0x02);

  writecommand(0xF7);
  writedata(0x20);

  writecommand(0xEA);
  writedata(0x00);
  writedata(0x00);

  writecommand(ILI9341_PWCTR1);    //Power control
  writedata(0x23);   //VRH[5:0]

  writecommand(ILI9341_PWCTR2);    //Power control
  writedata(0x10);   //SAP[2:0];BT[3:0]

  writecommand(ILI9341_VMCTR1);    //VCM control
  writedata(0x3e); //�Աȶȵ���
  writedata(0x28);

  writecommand(ILI9341_VMCTR2);    //VCM control2
  writedata(0x86);  //--

  writecommand(ILI9341_MADCTL);    // Memory Access Control
  writedata(0x48);

  writecommand(ILI9341_PIXFMT);
  writedata(0x55);

  writecommand(ILI9341_FRMCTR1);
  writedata(0x00);
  writedata(0x18);

  writecommand(ILI9341_DFUNCTR);    // Display Function Control
  writedata(0x08);
  writedata(0x82);
  writedata(0x27);

  writecommand(0xF2);    // 3Gamma Function Disable
  writedata(0x00);

  writecommand(ILI9341_GAMMASET);    //Gamma curve selected
  writedata(0x01);

  writecommand(ILI9341_GMCTRP1);    //Set Gamma
  writedata(0x0F);
  writedata(0x31);
  writedata(0x2B);
  writedata(0x0C);
  writedata(0x0E);
  writedata(0x08);
  writedata(0x4E);
  writedata(0xF1);
  writedata(0x37);
  writedata(0x07);
  writedata(0x10);
  writedata(0x03);
  writedata(0x0E);
  writedata(0x09);
  writedata(0x00);

  writecommand(ILI9341_GMCTRN1);    //Set Gamma
  writedata(0x00);
  writedata(0x0E);
  writedata(0x14);
  writedata(0x03);
  writedata(0x11);
  writedata(0x07);
  writedata(0x31);
  writedata(0xC1);
  writedata(0x48);
  writedata(0x08);
  writedata(0x0F);
  writedata(0x0C);
  writedata(0x31);
  writedata(0x36);
  writedata(0x0F);

  writecommand(ILI9341_SLPOUT);    //Exit Sleep
  if (hwSPI) spi_end();
  delay(120);
  if (hwSPI) spi_begin();
  writecommand(ILI9341_DISPON);    //Display on
  if (hwSPI) spi_end();

  madctl = 0xc8;
}


void ATX32_GLCD_ILI9341::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1,
                                        uint16_t y1) {
#if defined (__STM32F1__)||(__ATX32__)  
  writecommand(ILI9341_CASET); // Column addr set
  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;
  SPI_3.setDataSize (SPI_CR1_DFF);
  SPI_3.write(x0);
  SPI_3.write(x1);
//  SPI_3.setDataSize (0);
  
  writecommand(ILI9341_PASET); // Row addr set
  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;
//  SPI_3.setDataSize (SPI_CR1_DFF);
  SPI_3.write(y0);
  SPI_3.write(y1);
  SPI_3.setDataSize (0);

  writecommand(ILI9341_RAMWR); // write to RAM

#else										
  writecommand(ILI9341_CASET); // Column addr set
  writedata(x0 >> 8);
  writedata(x0 & 0xFF);     // XSTART
  writedata(x1 >> 8);
  writedata(x1 & 0xFF);     // XEND

  writecommand(ILI9341_PASET); // Row addr set
  writedata(y0 >> 8);
  writedata(y0);     // YSTART
  writedata(y1 >> 8);
  writedata(y1);     // YEND

  writecommand(ILI9341_RAMWR); // write to RAM
#endif 
}


void ATX32_GLCD_ILI9341::pushColor(uint16_t color) {
  if (hwSPI) spi_begin();
  //digitalWrite(_dc, HIGH);
  *dcport |=  dcpinmask;
  //digitalWrite(_cs, LOW);
  *csport &= ~cspinmask;

  spiwrite(color >> 8);
  spiwrite(color);

  *csport |= cspinmask;
  //digitalWrite(_cs, HIGH);
  if (hwSPI) spi_end();
}

void ATX32_GLCD_ILI9341::drawPixel(int16_t x, int16_t y, uint16_t color) {

  if ((x < 0) || (x >= _width) || (y < 0) || (y >= _height)) return;

  if (hwSPI) spi_begin();
  setAddrWindow(x, y, x + 1, y + 1);

  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;

  spiwrite(color >> 8);
  spiwrite(color);

  *csport |= cspinmask;
  if (hwSPI) spi_end();
}


void ATX32_GLCD_ILI9341::drawFastVLine(int16_t x, int16_t y, int16_t h,
                                        uint16_t color) {

  // Rudimentary clipping
  if ((x >= _width) || (y >= _height || h < 1)) return;

  if ((y + h - 1) >= _height)
    h = _height - y;
  if (h < 2 ) {
	drawPixel(x, y, color);
	return;
  }

  //  if (hwSPI) spi_begin();
  setAddrWindow(x, y, x, y + h - 1);

  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;
  
#if defined (__STM32F1__)||(__ATX32__)
  SPI_3.setDataSize (SPI_CR1_DFF); // Set SPI 16bit mode
  lineBuffer[0] = color;
  SPI_3.dmaSend(lineBuffer, h, 0);
  SPI_3.setDataSize (0);
 #else
  uint8_t hi = color >> 8, lo = color;
  while (h--) {
    spiwrite(hi);
    spiwrite(lo);
  }
#endif
  *csport |= cspinmask;
}


void ATX32_GLCD_ILI9341::drawFastHLine(int16_t x, int16_t y, int16_t w,
                                        uint16_t color) {

  
  // Rudimentary clipping
  if ((x >= _width) || (y >= _height || w < 1)) return;
  if ((x + w - 1) >= _width)  w = _width - x;
  if (w < 2 ) {
	drawPixel(x, y, color);
	return;
  }

//  if (hwSPI) spi_begin();
  setAddrWindow(x, y, x + w - 1, y);
  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;

#if defined (__STM32F1__)||(__ATX32__)
  SPI_3.setDataSize (SPI_CR1_DFF); // Set spi 16bit mode
  lineBuffer[0] = color;
  SPI_3.dmaSend(lineBuffer, w, 0);
  SPI_3.setDataSize (0);
#else
	uint8_t hi = color >> 8, lo = color;
    while (w--) {
      spiwrite(hi);
      spiwrite(lo);
    }
#endif
  *csport |= cspinmask;
  //digitalWrite(_cs, HIGH);
//  if (hwSPI) spi_end();
}



void ATX32_GLCD_ILI9341::fillScreen(uint16_t color) {
#if defined (__STM32F1__)||(__ATX32__)
  setAddrWindow(0, 0, _width - 1, _height - 1);
  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;
  SPI_3.setDataSize (SPI_CR1_DFF); // Set spi 16bit mode
  lineBuffer[0] = color;
  SPI_3.dmaSend(lineBuffer, (65535), 0);
  SPI_3.dmaSend(lineBuffer, ((_width * _height) - 65535), 0);
  SPI_3.setDataSize (0);

#else
  fillRect(0, 0,  _width, _height, color);
#endif 
}

// draw a rectangle
void ATX32_GLCD_ILI9341::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, 
		      uint16_t color) {
  // smarter version
  drawFastHLine(x, y, w, color);
  drawFastHLine(x, y+h-1, w, color);
  drawFastVLine(x, y, h, color);
  drawFastVLine(x+w-1, y, h, color);
}

// fill a rectangle
void ATX32_GLCD_ILI9341::fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                   uint16_t color) {

  // rudimentary clipping (drawChar w/big text requires this)
  if ((x >= _width) || (y >= _height || h < 1 || w < 1)) return;
  if ((x + w - 1) >= _width)  w = _width  - x;
  if ((y + h - 1) >= _height) h = _height - y;
  if (w == 1 && h == 1) {
    drawPixel(x, y, color);
    return;
  }
  
  if (hwSPI) spi_begin();
  setAddrWindow(x, y, x + w - 1, y + h - 1);

  *dcport |=  dcpinmask;
  *csport &= ~cspinmask;
#if defined (__STM32F1__)||(__ATX32__)
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
#else
  uint8_t hi = color >> 8, lo = color;
  for(y=h; y>0; y--) 
  {
    for(x=w; x>0; x--)
	{
      SPI_3.write(hi);
      SPI_3.write(lo);
    }
  }
#endif
  *csport |= cspinmask;
  if (hwSPI) spi_end();
}

/*
* Draw lines faster by calculating straight sections and drawing them with fastVline and fastHline.
*/
#if defined (__STM32F1__)||(__ATX32__)
void ATX32_GLCD_ILI9341::drawLine(int16_t x0, int16_t y0,int16_t x1, int16_t y1, uint16_t color)
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
	*csport &= ~cspinmask;
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
	*csport |= cspinmask;
}
#endif



void ATX32_GLCD_ILI9341::drawString(int16_t x, int16_t y, char *c, 
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
void ATX32_GLCD_ILI9341::drawChar(int16_t x, int16_t y, char c, 
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

void ATX32_GLCD_ILI9341::drawStringTransparent(int16_t x, int16_t y, char *c, 
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
void ATX32_GLCD_ILI9341::drawCharTransparent(int16_t x, int16_t y, char c, 
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
void ATX32_GLCD_ILI9341::fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
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
void ATX32_GLCD_ILI9341::drawCircle(int16_t x0, int16_t y0, int16_t r, 
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

void ATX32_GLCD_ILI9341::drawArc(int16_t h,int16_t k,int16_t r,int16_t start_angle,int16_t end_angle,uint16_t color)
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
void ATX32_GLCD_ILI9341::drawPie(int16_t h,int16_t k,int16_t r,int16_t start_angle,int16_t end_angle,uint16_t color)
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
/*
void ATX32_GLCD_ILI9341::drawPie(int16_t h,int16_t k,int16_t r,int16_t start_angle,int16_t end_angle,uint16_t color)
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
}*/

/*
void ATX32_GLCD_ILI9341::drawArc(int16_t h,int16_t k,int16_t r,int16_t start_angle,int16_t end_angle,uint16_t color)
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
void ATX32_GLCD_ILI9341::drawPie(int16_t h,int16_t k,int16_t r,int16_t start_angle,int16_t end_angle,uint16_t color)
{

       float angle=(((start_angle<=end_angle)?start_angle:end_angle)*(M_PI/180));
       float range=(((end_angle>start_angle)?end_angle:start_angle)*(M_PI/180));

       float x=(r*cos(angle));
       float y=(r*sin(angle));

      while(angle<=range)
      {
         drawLine(h,k,(int)(h+x+0.5),(int)(k-y+0.5),color);
//         angle+=0.0005;
         angle+=0.01;

         x=(r*cos(angle));
         y=(r*sin(angle));
      }
}



void ATX32_GLCD_ILI9341::drawRadius(int16_t h,int16_t k,int16_t r,int16_t end_angle,uint16_t color)
{

       float angle=(((0<=end_angle)?0:end_angle)*(M_PI/180));
       float range=(((end_angle>0)?end_angle:0)*(M_PI/180));

       float x=(r*cos(angle));
       float y=(r*sin(angle));

      while(angle<=range)
      {
         drawLine(h,k,(int)(h+x+0.5),(int)(k-y+0.5),color);

         angle+=0.01;

         x=(r*cos(angle));
         y=(r*sin(angle));
      }
}















// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t ATX32_GLCD_ILI9341::color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04


uint8_t ATX32_GLCD_ILI9341::getRotation() {
  return madctl;
}


void ATX32_GLCD_ILI9341::setRotation(uint8_t m) {
  if (hwSPI) spi_begin();
  rotation = m % 4; // can't be higher than 3
  switch (rotation) {
    case 0:
      madctl=(MADCTL_MX | MADCTL_BGR);
      _width  = ILI9341_TFTWIDTH;
      _height = ILI9341_TFTHEIGHT;
      break;
    case 1:
      madctl=(MADCTL_MV | MADCTL_BGR);
      _width  = ILI9341_TFTHEIGHT;
      _height = ILI9341_TFTWIDTH;
      break;
    case 2:
      madctl=(MADCTL_MY | MADCTL_BGR);
      _width  = ILI9341_TFTWIDTH;
      _height = ILI9341_TFTHEIGHT;
      break;
    case 3:
      madctl=(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
      _width  = ILI9341_TFTHEIGHT;
      _height = ILI9341_TFTWIDTH;
      break;
  }
 writecommand(ILI9341_MADCTL);  // memory access control (directions)
 writedata(madctl);  // row address/col address, bottom to top refresh
 if (hwSPI) spi_end();
}

void ATX32_GLCD_ILI9341::setRotationReg(uint8_t m) {
  madctl = m;
  if (hwSPI) spi_begin();
  writecommand(ILI9341_MADCTL);  // memory access control (directions)
  writedata(madctl);  // row address/col address, bottom to top refresh
  if (hwSPI) spi_end();
/*  switch (m>>4)
  {
	case 0x0c:
			_width = 128;
			_height = 160;
			break;
	case 0x0a:
			_width = 160;
			_height = 128;
			break;
	case 0x01:
			_width = 128;
			_height = 160;
			break;
	case 0x07:
			_width = 160;
			_height = 128;
			break;
  }
*/
}

void ATX32_GLCD_ILI9341::invertDisplay(boolean i) {
  if (hwSPI) spi_begin();
  writecommand(i ? ILI9341_INVON : ILI9341_INVOFF);
  if (hwSPI) spi_end();
}


////////// stuff not actively being used, but kept for posterity


uint8_t ATX32_GLCD_ILI9341::spiread(void) {
  uint8_t r = 0;

  if (hwSPI) {
#if defined (__AVR__)
    uint8_t backupSPCR = SPCR;
    SPCR = mySPCR;
    SPDR = 0x00;
    while (!(SPSR & _BV(SPIF)));
    r = SPDR;
    SPCR = backupSPCR;
#elif defined(TEENSYDUINO)
    r = SPI_3.transfer(0x00);
#elif defined (__STM32F1__)||(__ATX32__)
    r = SPI_3.transfer(0x00);
#elif defined (__arm__)
    SPI_3.setClockDivider(11); // 8-ish MHz (full! speed!)
    SPI_3.setBitOrder(MSBFIRST);
    SPI_3.setDataMode(SPI_MODE0);
    r = SPI_3.transfer(0x00);
#endif
  } else {

    for (uint8_t i = 0; i < 8; i++) {
      digitalWrite(_sclk, LOW);
      digitalWrite(_sclk, HIGH);
      r <<= 1;
      if (digitalRead(_miso))
        r |= 0x1;
    }
  }
  //Serial.print("read: 0x"); Serial.print(r, HEX);

  return r;
}

uint8_t ATX32_GLCD_ILI9341::readdata(void) {
  digitalWrite(_dc, HIGH);
  digitalWrite(_cs, LOW);
  uint8_t r = spiread();
  digitalWrite(_cs, HIGH);

  return r;
}


uint8_t ATX32_GLCD_ILI9341::readcommand8(uint8_t c, uint8_t index) {
  if (hwSPI) spi_begin();
  digitalWrite(_dc, LOW); // command
  digitalWrite(_cs, LOW);
  spiwrite(0xD9);  // woo sekret command?
  digitalWrite(_dc, HIGH); // data
  spiwrite(0x10 + index);
  digitalWrite(_cs, HIGH);

  digitalWrite(_dc, LOW);
  //if(hwSPI) digitalWrite(_sclk, LOW);
  digitalWrite(_cs, LOW);
  spiwrite(c);

  digitalWrite(_dc, HIGH);
  uint8_t r = spiread();
  digitalWrite(_cs, HIGH);
  if (hwSPI) spi_end();
  return r;
}



/*

 uint16_t ATX32_GLCD_ILI9341::readcommand16(uint8_t c) {
 digitalWrite(_dc, LOW);
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

 uint32_t ATX32_GLCD_ILI9341::readcommand32(uint8_t c) {
 digitalWrite(_dc, LOW);
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
