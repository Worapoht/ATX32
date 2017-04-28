#ifndef _ATX32_GLCD18_H_
#define _ATX32_GLCD18_H_

#if defined(ARDUINO) || (ARDUINO >= 100)
  #include "Arduino.h"
#endif

#include "WString.h"
#include "itoa.h"
#include "avr/dtostrf.h"

#include "ATX32_GLCD_ILI9341.h"
#include <SPI.h>

#define uchar unsigned char
#define uint unsigned int

#define GLCD_RED		0xf800
#define GLCD_GREEN		0x07e0
#define GLCD_BLUE		0x001f
#define GLCD_YELLOW		0xffe0
#define GLCD_BLACK		0x0000
#define GLCD_WHITE		0xffff
#define GLCD_CYAN		0x07ff
#define GLCD_MAGENTA	0xf81f
//new color
#define GLCD_ORANGE	0b1111110100000000
#define GLCD_LIME	0b1010011111100000
#define GLCD_VIOLET	0x88ff
#define GLCD_PINK	0xf41c
#define GLCD_DOLLAR	0xb796
#define GLCD_SKY	0x2d7f
#define GLCD_BROWN	0x8aa0
#define GLCD_DARKGREEN	0x04E0
#define GLCD_NAVY	0x0014

#define GLCD_GRAY	0b1000010000010000
#define GLCD_DARKGRAY	0b0100001000001000

#define GLCD_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define GLCD_MAROON      0x7800      /* 128,   0,   0 */
#define GLCD_PURPLE      0x780F      /* 128,   0, 128 */
#define GLCD_OLIVE       0x7BE0      /* 128, 128,   0 */
#define GLCD_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define GLCD_GREENYELLOW 0xAFE5      /* 173, 255,  47 */

//#define sclk 23  // ATX32 pin
//#define mosi 21  // ATX32 pin
#define cs PD4  // ATX32 pin
#define dc PD3 // ATX32 pin
#define rst PD7 // ATX32 pin

#define GLCD_WIDTH 128
#define GLCD_HEIGHT 160
#define CHAR_PER_ROW 25

//-------------------- Condition  test parameter data type for display ---------------//
#ifndef TEST_CHAR_TYPE(x)
#define TEST_CHAR_TYPE(x) *x=='%' && (*(x+1)=='c' || *(x+1)=='C')
#endif

#ifndef TEST_BIN_TYPE(x)
#define TEST_BIN_TYPE(x) *x=='%' && (*(x+1)=='b' || *(x+1)=='B')
#endif

#ifndef TEST_HEX_TYPE(x)
#define TEST_HEX_TYPE(x) *x=='%' && (*(x+1)=='h' || *(x+1)=='H')
#endif

#ifndef TEST_INT_TYPE(x)
#define TEST_INT_TYPE(x) *x=='%' && (*(x+1)=='d' || *(x+1)=='D')
#endif

#ifndef TEST_LONG_TYPE(x)
#define TEST_LONG_TYPE(x) *x=='%' && (*(x+1)=='l' || *(x+1)=='L')
#endif

#ifndef TEST_FLOAT_TYPE(x)
#define TEST_FLOAT_TYPE(x) *x=='%' && (*(x+1)=='f' || *(x+1)=='F')
#endif

/*#ifndef TEST_CLEAR_NEXT_TYPE(x)
#define TEST_CLEAR_NEXT_TYPE(x) *x== '#' && *(x+1)=='c' // Clear display next time
#endif

#ifndef TEST_NEW_LINE_TYPE(x)
#define TEST_NEW_LINE_TYPE(x) *x== '#' && *(x+1)=='n' 	// Go to line 2
#endif
*/

#ifndef SET_FLOAT_PREC
#define SET_FLOAT_PREC 1
unsigned int _floatPrecision=3;
void setFloatPrecision(unsigned int prec)
{
	_floatPrecision=prec;
}
#endif

char _glcd_ini = 0;
char _glcd_clear_display=0;

char _glcd_bgr = 1;

uint color[]=	{	GLCD_RED,
					GLCD_GREEN,
					GLCD_BLUE,
					GLCD_YELLOW,
					GLCD_BLACK,
					GLCD_WHITE,
					GLCD_CYAN,
					GLCD_MAGENTA,
					GLCD_ORANGE,
					GLCD_LIME,
					GLCD_VIOLET,
					GLCD_PINK,
					GLCD_DOLLAR,
					GLCD_SKY,
					GLCD_BROWN,
					GLCD_DARKGREEN,
					GLCD_NAVY,
					GLCD_DARKGRAY,
					GLCD_GRAY
};

volatile uint _textColor = GLCD_WHITE;
volatile uint _textBackgroundColor = GLCD_BLACK;
volatile uint8_t _textSize = 1;
ATX32_GLCD_ILI9341 __tft = ATX32_GLCD_ILI9341(cs, dc, rst); // Remark Hardware SPI Mode for avoid Timer2 conflict

////GCLCD __tft = GCLCD(cs, dc, mosi, sclk); 

void LCD_init()
{
  __tft.begin();   // initialize
  __tft.fillScreen(GLCD_BLACK);
  _glcd_ini=1;
}
void glcdConfig()
{
	if(_glcd_ini==0)
	{
		_glcd_bgr=1;
		LCD_init();
		_glcd_ini=1;
	}
}

void glcdSetColorWordBGR()
{
	unsigned int oldMode;
	if(_glcd_ini==0)
	{
		_glcd_bgr=1;
		LCD_init();
		_glcd_ini=1;
	}
	oldMode=__tft.getRotation();
	_glcd_bgr=1;
	oldMode&=0xf7;
	__tft.setRotationReg(oldMode);
}
void glcdSetColorWordRGB()
{
	unsigned int oldMode;
	if(_glcd_ini==0)
	{
		_glcd_bgr=0;
		LCD_init();
		_glcd_ini=1;
	}
	oldMode=__tft.getRotation();
	_glcd_bgr=0;
	oldMode|=0x08;
	__tft.setRotationReg(oldMode);
}

void glcdConfig(int ColorMode)
{
	if (ColorMode==1)
	{
	  _glcd_bgr=1;
	}
	else
	{
	  _glcd_bgr=0;
	}
	if(_glcd_ini==0)
	{
		LCD_init();
		_glcd_ini=1;
	}
}

void __glcdCharX(int16_t x, int16_t y ,uint8_t size, char c)
{
//   glcdConfig();
    __tft.drawCharTransparent(x*6, y*10, c, _textColor ,size);
}
void __glcdStringX(int16_t x, int16_t y ,uint8_t size,char *text) {
  glcdConfig();
   if (GCLCD_TransparentBackground==0)
   {
     __tft.drawString(x*(6*size), y*(10*size), text, _textColor,_textBackgroundColor,size);
   }
   else
   {
     __tft.drawString(x*(6*size), y*(10*size), text, _textColor,size);
   }
}

void glcdChar(int16_t x, int16_t y, char c)
{
   glcdConfig();
//   __tft.drawChar(x*5, y*10, c, _textColor ,_textBackgroundColor);
   if (GCLCD_TransparentBackground==0)
   {
     __tft.drawChar(x*6, y*10, c, _textColor ,_textBackgroundColor,_textSize);
   }
   else
   {
     __tft.drawCharTransparent(x*6, y*10, c, _textColor ,_textSize);
   }
}
void glcdString(uchar x, uchar y ,char *text) {
  glcdConfig();
//  __tft.drawString(x*5, y*10, text, _textColor,_textBackgroundColor);
   if (GCLCD_TransparentBackground==0)
   {
     __tft.drawString(x*6, y*10, text, _textColor,_textBackgroundColor,_textSize);
   }
   else
   {
     __tft.drawStringTransparent(x*6, y*10, text, _textColor,_textSize);
   }
}

uint8_t glcdMode(uint8_t modeset)
{
 uint8_t __cmdResult=1;
 glcdConfig();
 switch (modeset)
  {
	case 0:
			if (_glcd_bgr==0)
			{ __tft.setRotationReg(0xc8);}
			else
			{ __tft.setRotationReg(0xc0);}
			break;
	case 1:
			if (_glcd_bgr==0)
			{ __tft.setRotationReg(0xa8);}
			else
			{ __tft.setRotationReg(0xa0);}
			break;
	case 2:
			if (_glcd_bgr==0)
			{ __tft.setRotationReg(0x18);}
			else
			{ __tft.setRotationReg(0x10);}
			break;
	case 3:
			if (_glcd_bgr==0)
			{ __tft.setRotationReg(0x78);}
			else
			{ __tft.setRotationReg(0x70);}
			break;
	default:
			__cmdResult=0;
			break;
  }
  return __cmdResult;
}

uint8_t glcdGetMode(void)
{
  uint8_t __modeResult;
  __modeResult=(__tft.getRotation())>>4;

  if (__modeResult==0x0c)
  {
	return 0;
  }
  else if (__modeResult==0x0A)
  {
	return 1;
  }
  else if (__modeResult==0x01)
  {
	return 2;
  }
  else if (__modeResult==0x07)
  {
	return 3;
  }
  else 
  {
	return 0xff;
  }
}

uint8_t glcdFlip(uint16_t degree)
{
  uint8_t __cmdResult=1;
  glcdConfig();
  switch (degree)
  {
	case 90:
			__tft.setRotationReg(0xc8);
			break;
	case 0:
			__tft.setRotationReg(0xA8);
			break;
	case 270:
			__tft.setRotationReg(0x18);
			break;
	case 180:
			__tft.setRotationReg(0x78);
			break;
	default:
			__cmdResult=0;
			break;
  }
  return __cmdResult;
}

uint16_t glcdGetFlip(void)
{
  uint8_t __modeResult;
  __modeResult=(__tft.getRotation())>>4;

  if (__modeResult==0x0c)
  {
	return 90;
  }
  else if (__modeResult==0x0A)
  {
	return 0;
  }
  else if (__modeResult==0x01)
  {
	return 270;
  }
  else if (__modeResult==0x07)
  {
	return 180;
  }
  else 
  {
	return 0xffff;
  }
}

//-----------------------------------------------------------------------------------------------------------------------------//
//------------------------------------ Function LCD display and %d support(Beta version) ----------------------------------------//
//-----------------------------------------------------------------------------------------------------------------------------//

// Override with new method function call
void glcd(uchar _x, uchar _y ,char *p,...)
{
	int char_count=0,x,y;
    char *arg,**pp;	// Poter of point
	char *ptr,buff[16];
	glcdConfig();
	pp = &p;
	ptr = p;	// Copy address
	arg = (char *)pp;	// Copy address of p point	
	
	/*if(_lcd_clear_display)
	{
		//_lcd_clear_display = 0;
		//lcd_clear();
	}*/
	arg += 2;	// Cross 2 time go to Origin of first parameter 

	_x = _x*_textSize;
	_y = _y*_textSize;
	
	while(*ptr)                 // Check data pointer = 0?
	{
	 	if(TEST_CHAR_TYPE(ptr))
		{
//			x = (_x*(CHAR_PER_ROW*_textSize) + _y + (char_count*_textSize))%(CHAR_PER_ROW*_textSize);
//			y = (_x*(CHAR_PER_ROW*_textSize) + _y + (char_count*_textSize))/(CHAR_PER_ROW*_textSize);
			x = (_x*(CHAR_PER_ROW*_textSize))%(CHAR_PER_ROW*_textSize);
			y = (_y + (char_count*_textSize))/(CHAR_PER_ROW*_textSize);
			char_count++;
       
       	    __glcdCharX(x,y,_textSize,toascii(*arg));
			arg+=2;	// Cross address char type
			ptr++;	// Cross %d parameter
		}
		else if(TEST_BIN_TYPE(ptr))
		{
             p = ultoa(*(unsigned int *)arg,&buff[0],2);
			while(*p)           // Check data pointer = 0?
			{
				x = (_x*(CHAR_PER_ROW*_textSize) + _y + (char_count*_textSize))%(CHAR_PER_ROW*_textSize);
				y = (_x*(CHAR_PER_ROW*_textSize) + _y + (char_count*_textSize))/(CHAR_PER_ROW*_textSize);
                char_count++;
			
            	__glcdCharX(x,y,_textSize,*p);
	 			p++;                     // Increase address 1 time 
			}
			arg+=2;	// Cross address int type
			ptr++;	// Cross %d parameter
		}
		else if(TEST_HEX_TYPE(ptr))
		{
             p = ultoa(*(unsigned int *)arg,&buff[0],16);
			while(*p)           // Check data pointer = 0?
			{
				x = (_x*(CHAR_PER_ROW*_textSize) + _y + (char_count*_textSize))%(CHAR_PER_ROW*_textSize);
				y = (_x*(CHAR_PER_ROW*_textSize) + _y + (char_count*_textSize))/(CHAR_PER_ROW*_textSize);
                char_count++;
			
            	__glcdCharX(x,y,_textSize,*p);
	 			p++;                     // Increase address 1 time 
			}
			arg+=2;	// Cross address int type
			ptr++;	// Cross %d parameter
		}
		else if(TEST_INT_TYPE(ptr))
		{
             p = ltoa(*(int *)arg,&buff[0],10);
			while(*p)           // Check data pointer = 0?
			{
				x = (_x*(CHAR_PER_ROW*_textSize) + _y + (char_count*_textSize))%(CHAR_PER_ROW*_textSize);
				y = (_x*(CHAR_PER_ROW*_textSize) + _y + (char_count*_textSize))/(CHAR_PER_ROW*_textSize);
                char_count++;
			
            	__glcdCharX(x,y,_textSize,*p);
	 			p++;                     // Increase address 1 time 
			}
			arg+=2;	// Cross address int type
			ptr++;	// Cross %d parameter
		}
		else if(TEST_LONG_TYPE(ptr))
		{
			p = ltoa(*(long *)arg,&buff[0],10);
			while(*p)           // Check data pointer = 0?
			{
				x = (_x*(CHAR_PER_ROW*_textSize) + _y + (char_count*_textSize))%(CHAR_PER_ROW*_textSize);
				y = (_x*(CHAR_PER_ROW*_textSize) + _y + (char_count*_textSize))/(CHAR_PER_ROW*_textSize);
                char_count++;
			
            	__glcdCharX(x,y,_textSize,*p);
	 			p++;                     // Increase address 1 time 
			}
			arg+=4;	// Cross address long type
			ptr++;	// Cross %l parameter
		}
		else if(TEST_FLOAT_TYPE(ptr))
		{
			p = dtostrf(*(float *)arg,2,_floatPrecision,&buff[0]);	// Convert float to string(used libm.a)
			while(*p)           // Check data pointer = 0?
			{
				x = (_x*(CHAR_PER_ROW*_textSize) + _y + (char_count*_textSize))%(CHAR_PER_ROW*_textSize);
				y = (_x*(CHAR_PER_ROW*_textSize) + _y + (char_count*_textSize))/(CHAR_PER_ROW*_textSize);
                char_count++;
		
        		__glcdCharX(x,y,_textSize,*p);
	 			p++;                     // Increase address 1 time 
			}
			arg+=4;	// Cross address long type
			ptr++;	// Cross %l parameter
		}
		/*else if(TEST_CLEAR_NEXT_TYPE(ptr))
		{
			_lcd_clear_display = 1;	// Clear display next time 
			
			ptr++;	// Cross %l parameter
		}
		else if(TEST_NEW_LINE_TYPE(ptr))
		{
			ptr++;	// Cross %l parameter
		}*/
		else
		{
				x = (_x*(CHAR_PER_ROW*_textSize) + _y + (char_count*_textSize))%(CHAR_PER_ROW*_textSize);
				y = (_x*(CHAR_PER_ROW*_textSize) + _y + (char_count*_textSize))/(CHAR_PER_ROW*_textSize);
			char_count++;
	
    		__glcdCharX(x,y,_textSize,*ptr);
	 	}
		ptr++;                      // Increase address 1 time 
	}
}

/*
void glcd(uchar _x, uchar _y ,char *p,...)
{
	int char_count=0,x,y;
    char *arg,**pp;	// Poter of point
	char *ptr,buff[16];
	pp = &p;
	ptr = p;	// Copy address
	arg = (char *)pp;	// Copy address of p point	
	
//	if(_lcd_clear_display)
//	{
//		//_lcd_clear_display = 0;
//		//lcd_clear();
//	}
	arg += 2;	// Cross 2 time go to Origin of first parameter 
	
	while(*ptr)                 // Check data pointer = 0?
	{
	 	if(TEST_CHAR_TYPE(ptr))
		{
			x = (_x*CHAR_PER_ROW + _y + char_count)%CHAR_PER_ROW;
			y = (_x*CHAR_PER_ROW + _y + char_count)/CHAR_PER_ROW;
			char_count++;
       
       	    glcdChar(x,y,toascii(*arg));
			arg+=2;	// Cross address char type
			ptr++;	// Cross %d parameter
		}
		else if(TEST_INT_TYPE(ptr))
		{
             p = ltoa(*(int *)arg,&buff[0],10);
			while(*p)           // Check data pointer = 0?
			{
	 			x = (_x*CHAR_PER_ROW + _y + char_count)%CHAR_PER_ROW;
			    y = (_x*CHAR_PER_ROW + _y + char_count)/CHAR_PER_ROW;
                char_count++;
			
            	glcdChar(x,y,*p);
	 			p++;                     // Increase address 1 time 
			}
			arg+=2;	// Cross address int type
			ptr++;	// Cross %d parameter
		}
		else if(TEST_LONG_TYPE(ptr))
		{
			p = ltoa(*(long *)arg,&buff[0],10);
			while(*p)           // Check data pointer = 0?
			{
	 			x = (_x*CHAR_PER_ROW + _y + char_count)%CHAR_PER_ROW;
			    y = (_x*CHAR_PER_ROW + _y + char_count)/CHAR_PER_ROW;
                char_count++;
			
            	glcdChar(x,y,*p);
	 			p++;                     // Increase address 1 time 
			}
			arg+=4;	// Cross address long type
			ptr++;	// Cross %l parameter
		}
		else if(TEST_FLOAT_TYPE(ptr))
		{
			p = dtostrf(*(float *)arg,2,F_PREC,&buff[0]);	// Convert float to string(used libm.a)
			while(*p)           // Check data pointer = 0?
			{
	 			x = (_x*CHAR_PER_ROW + _y + char_count)%CHAR_PER_ROW;
			    y = (_x*CHAR_PER_ROW + _y + char_count)/CHAR_PER_ROW;
                char_count++;
		
        		glcdChar(x,y,*p);
	 			p++;                     // Increase address 1 time 
			}
			arg+=4;	// Cross address long type
			ptr++;	// Cross %l parameter
		}
//		else if(TEST_CLEAR_NEXT_TYPE(ptr))
//		{
//			_lcd_clear_display = 1;	// Clear display next time 
//			
//			ptr++;	// Cross %l parameter
//		}
//		else if(TEST_NEW_LINE_TYPE(ptr))
//		{
//			ptr++;	// Cross %l parameter
//		}
		else
		{
			x = (_x*CHAR_PER_ROW + _y + char_count)%CHAR_PER_ROW;
			y = (_x*CHAR_PER_ROW + _y + char_count)/CHAR_PER_ROW;
			char_count++;
	
    		glcdChar(x,y,*ptr);
	 	}
		ptr++;                      // Increase address 1 time 
	}
}
*/
uint colorRGB(uint red,uint green,uint blue)	// Generate custom color format RGB565 bit 
{
	if(red>31)	// Support RED range 0-31(5 bit color)
	{
		red = 31;
	}
	if(green>63)	// Support GREEN range 0-63(6 bit color)
	{
		green=63;
	}
	if(blue>31)	// Support BLUE range 0-31(5 bit color)
	{
		blue=31;
	}
	return((red<<11)+(green<<5)+(blue));
}
void setTextColor(uint newColor)
{
	_textColor = newColor;
}
void setTextBackgroundColor(uint newColor)
{
	GCLCD_TransparentBackground=0;
	_textBackgroundColor = newColor;
}
void setTextBackgroundTransparent()
{
	GCLCD_TransparentBackground=1;
}
void setTextSize(uint newSize)
{
	_textSize = newSize;
}
uint getTextColor()
{
	return(_textColor);
}
uint getTextBackgroundColor()
{
	return(_textBackgroundColor);
}
uint8_t getTextSize()
{
	return(_textSize);
}

void glcdFillScreen(uint color)
{
	glcdConfig();
    __tft.fillScreen(color);
}
void glcdClear(void)
{
	glcdConfig();
    __tft.fillScreen(_textBackgroundColor);
}
void glcdPixel(uint x,uint y,uint color)
{
	glcdConfig();
    __tft.drawPixel(x,y,color);
}
void glcdRect(uint x1,uint y1,uint width,uint height,uint color)	// rect fill inner
{
	 glcdConfig();
     __tft.drawRect(x1,y1,width,height,color);
}
void glcdFillRect(uint x1,uint y1,uint width,uint height,uint color)	// rect fill inner
{
	 glcdConfig();
     __tft.fillRect(x1,y1,width,height,color);
}
void glcdLine(uint x1, uint y1, uint x2, uint y2,uint color)
{
	glcdConfig();
    __tft.drawLine(x1,y1,x2,y2,color);
}

void glcdCircle(uint x, uint y, uint radius,uint color)
{
	glcdConfig();
    __tft.drawCircle(x,y,radius,color);  
}
void glcdFillCircle(uint x, uint y, uint radius,uint color)
{
	glcdConfig();
   	__tft.fillCircle(x,y,radius,color);  
}

void glcdArc(int16_t h,int16_t k,int16_t r,int16_t start_angle,int16_t end_angle,uint16_t color)
{
	glcdConfig();
	__tft.drawArc(h,k,r,start_angle,end_angle,color);
}

/*void glcdPie(int16_t h,int16_t k,int16_t r,int16_t start_angle,int16_t end_angle,uint16_t color)
{
	glcdConfig();
	__tft.drawPie(h,k,r,start_angle,end_angle,color);
}
*/
#endif
