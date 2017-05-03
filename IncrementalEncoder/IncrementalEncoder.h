//------------------------------------------------------------------------------------------------//
// Incremental Encoder
//------------------------------------------------------------------------------------------------//
#ifndef _INCREMENTAL_ENCODER_H_
#define _INCREMENTAL_ENCODER_H_

#if defined(ARDUINO) || (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

volatile int pinEncoder1A=PD11;
volatile int pinEncoder1B=PD10;
volatile int pinEncoder2A=PD9;
volatile int pinEncoder2B=PD8;

volatile signed int encoder1Cnt,encoder2Cnt;

volatile bool __encoder1attached=false;
volatile bool __encoder2attached=false;

volatile int __encoder1previousState;
volatile int __encoder2previousState;

volatile int __encoder1presentState;
volatile int __encoder2presentState;

volatile int __encoder1Aread;
volatile int __encoder1Bread;
volatile int __encoder2Aread;
volatile int __encoder2Bread;

void encoder1pin(int pinA,int PinB)
{
	pinEncoder1A=pinA;
	pinEncoder1B=pinB;
}

void encoder2pin(int pinA,int PinB)
{
	pinEncoder2A=pinA;
	pinEncoder2B=pinB;
}

void encoder1Changed()
{
	__encoder1presentState=((digitalRead(pinEncoder1A))<<1)|(digitalRead(pinEncoder1B));
	switch(__encoder1previousState)
	{
		case 0b0000:
						if (__encoder1presentState==0b0001)
						{ encoder1Cnt++; }
						else
						{ encoder1Cnt--; }
						break;
		case 0b0001:
						if (__encoder1presentState==0b0011)
						{ encoder1Cnt++; }
						else
						{ encoder1Cnt--; }
						break;
		case 0b0011:
						if (__encoder1presentState==0b0010)
						{ encoder1Cnt++; }
						else
						{ encoder1Cnt--; }
						break;
		case 0b0010:
						if (__encoder1presentState==0b0000)
						{ encoder1Cnt++; }
						else
						{ encoder1Cnt--; }
						break;
	}
	__encoder1previousState=__encoder1presentState;
}

void encoder2Changed()
{
	__encoder2presentState=((digitalRead(pinEncoder2A))<<1)|(digitalRead(pinEncoder2B));
	switch(__encoder2previousState)
	{
		case 0b0000:
						if (__encoder2presentState==0b0001)
						{ encoder2Cnt++; }
						else
						{ encoder2Cnt--; }
						break;
		case 0b0001:
						if (__encoder2presentState==0b0011)
						{ encoder2Cnt++; }
						else
						{ encoder2Cnt--; }
						break;
		case 0b0011:
						if (__encoder2presentState==0b0010)
						{ encoder2Cnt++; }
						else
						{ encoder2Cnt--; }
						break;
		case 0b0010:
						if (__encoder2presentState==0b0000)
						{ encoder2Cnt++; }
						else
						{ encoder2Cnt--; }
						break;
	}
	__encoder2previousState=__encoder2presentState;
}

signed int encoder(int channel)
{
	switch(channel)
	{
	  case 1: if (!__encoder1attached)
		{
	    __encoder1attached=true;
		// encoder 1
		encoder1Cnt=0; // Clear slot counter
		pinMode(pinEncoder1A,INPUT_PULLUP);
		pinMode(pinEncoder1B,INPUT_PULLUP);
		attachInterrupt(pinEncoder1A, encoder1Changed, CHANGE);
		attachInterrupt(pinEncoder1B, encoder1Changed, CHANGE);
		__encoder1previousState=((digitalRead(pinEncoder1A))<<1)|(digitalRead(pinEncoder1B));
		// end encoder
		}
		return encoder1Cnt;
		break;
	  case 2: if (!__encoder2attached)
	    {
	    __encoder2attached=true;
		// encoder 2
		encoder2Cnt=0; // Clear slot counter
		pinMode(pinEncoder2A,INPUT_PULLUP);
		pinMode(pinEncoder2B,INPUT_PULLUP);
		attachInterrupt(pinEncoder2A, encoder2Changed, CHANGE);
		attachInterrupt(pinEncoder2B, encoder2Changed, CHANGE);		// end encoder
		__encoder2previousState=((digitalRead(pinEncoder2A))<<1)|(digitalRead(pinEncoder2B));
		}
		return encoder2Cnt;
		break;
	  default:
		return 0;
	}
}

void encoder(int channel,int preloadValue)
{
	switch(channel)
	{
	  case 1: if (!__encoder1attached)
		{
	    __encoder1attached=true;
		// encoder 1
		encoder1Cnt=0; // Clear slot counter
		pinMode(pinEncoder1A,INPUT_PULLUP);
		pinMode(pinEncoder1B,INPUT_PULLUP);
		attachInterrupt(pinEncoder1A, encoder1Changed, CHANGE);
		attachInterrupt(pinEncoder1B, encoder1Changed, CHANGE);
		// end encoder
		}
		encoder1Cnt=preloadValue; // Clear slot counter
		break;
	  case 2: if (!__encoder2attached)
	    {
	    __encoder2attached=true;
		// encoder 2
		encoder2Cnt=0; // Clear slot counter
		pinMode(pinEncoder2A,INPUT_PULLUP);
		pinMode(pinEncoder2B,INPUT_PULLUP);
		attachInterrupt(pinEncoder2A, encoder2Changed, CHANGE);
		attachInterrupt(pinEncoder2B, encoder2Changed, CHANGE);		// end encoder
		}
		encoder2Cnt=preloadValue; // Clear slot counter
		break;
	}
}

#endif
