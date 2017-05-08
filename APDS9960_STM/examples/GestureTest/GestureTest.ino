/***************************************************
ATX-32 with APDS9960 Gesture Test

Tests the gesture sensing abilities of the APDS-9960. Configures
APDS-9960 over I2C and waits for gesture events. Calculates the
direction of the swipe (up, down, left, right) and displays it
on a GLCD

To perform a NEAR gesture, hold your hand
far above the sensor and move it close to the sensor (within 2
inches). Hold your hand there for at least 1 second and move it
away.

To perform a FAR gesture, hold your hand within 2 inches of the
sensor for at least 1 second and then move it above (out of
range) of the sensor.

Hardware Connections:

 ATX-32 Pin   APDS-9960 Board  Function
 
 3.3V         VCC              Power
 GND          GND              Ground
 PB7          SDA              I2C Data
 PB6          SCL              I2C Clock
 PE0          INT              Interrupt

Distributed as-is; no warranty is given.
****************************************************/

#include <Adafruit_GFX_AS.h>    // Core graphics library
#include <Adafruit_ST7735_STM.h> // Hardware-specific library
#include <SPI.h>

// For the breakout, you can use any 2 or 3 pins
// These pins will also work for the 1.8" TFT shield

#define TFT_CS     PD5
#define TFT_RST    PD6  // you can also connect this to the Arduino reset
// in which case, set this #define pin to 0!
#define TFT_DC     PD7

// Option 1 (recommended): must use the hardware SPI pins
// (for UNO thats sclk = 13 and sid = 11) and pin 10 must be
// an output. This is much faster - also required if you want
// to use the microSD card (see the image drawing example)
Adafruit_ST7735_STM tft = Adafruit_ST7735_STM(TFT_CS,  TFT_DC, TFT_RST);


#include <HardWire.h>
#include <APDS9960_STM.h>

// Pins
int APDS9960_INT = PE0; // Needs to be an interrupt pin
volatile int APDS_LastGesture;
volatile int APDS_GestureTrig;
// Global Variables
APDS9960_STM apds = APDS9960_STM();
HardWire HWire(1, I2C_FAST_MODE); // I2c1

int isr_flag = 0;

void setup(void) {

  // Set interrupt pin as input
  pinMode(APDS9960_INT, INPUT);
  pinMode(BOARD_LED_PIN, OUTPUT);

  // Use this initializer if you're using a 1.8" TFT
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab

  // Use this initializer (uncomment) if you're using a 1.44" TFT
  //tft.initR(INITR_144GREENTAB);   // initialize a ST7735S chip, black tab

  tft.fillScreen(ST7735_BLACK);

  Serial.begin(9600);
  Serial.println();
  delay(3000);

  // Initialize interrupt service routine
  attachInterrupt(APDS9960_INT, interruptRoutine, FALLING);

  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.init() ) {
    Serial.println(F("APDS-9960 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9960 init!"));
  }

  // Start running the APDS-9960 gesture sensor engine
  if ( apds.enableGestureSensor(true) ) {
    Serial.println(F("Gesture sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during gesture sensor init!"));
  }

  // Wait for initialization and calibration to finish
  delay(500);
  APDS_GestureTrig=0;
}

void loop() {
  int i, j;
  digitalWrite(BOARD_LED_PIN, HIGH);
  delay(100);
  tft.setCursor(0, 8);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.println("APDS-9960 Gesture");

  tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
  for (;;)
  {
    if ( isr_flag == 1 ) {
      detachInterrupt(APDS9960_INT);
      handleGesture();
      isr_flag = 0;
      attachInterrupt(APDS9960_INT, interruptRoutine, FALLING);
    }
    if (APDS_GestureTrig == 1) {
      APDS_GestureTrig = 0;
      digitalWrite(BOARD_LED_PIN, LOW);
      tft.setCursor(10, 30);
      tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);
      switch ( APDS_LastGesture ) {
        case DIR_UP:
          tft.println("UP   ");
          break;
        case DIR_DOWN:
          tft.println("DOWN ");
          break;
        case DIR_LEFT:
          tft.println("LEFT ");
          break;
        case DIR_RIGHT:
          tft.println("RIGHT");
          break;
        case DIR_NEAR:
          tft.println("NEAR ");
          break;
        case DIR_FAR:
          tft.println("FAR  ");
          break;
        default:
          tft.println("NONE ");
      }
      delay(100);
      digitalWrite(BOARD_LED_PIN, HIGH);
      delay(100);
    }
  }
}

void interruptRoutine() {
  isr_flag = 1;
}

void handleGesture() {
  if ( apds.isGestureAvailable() ) {
    APDS_GestureTrig = 1;
    APDS_LastGesture = apds.readGesture();
  }
}