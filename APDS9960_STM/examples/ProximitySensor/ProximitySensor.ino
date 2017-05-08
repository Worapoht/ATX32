/****************************************************************
  ATX-32 with APDS9960 Proximity Test

  Tests the proximity sensing abilities of the APDS-9960.
  Configures the APDS-9960 over I2C and polls for the distance to
  the object nearest the sensor.

  displays it on a GLCD

  Hardware Connections:

  ATX-32 Pin   APDS-9960 Board  Function

  3.3V         VCC              Power
  GND          GND              Ground
  PB7          SDA              I2C Data
  PB6          SCL              I2C Clock

  Distributed as-is; no warranty is given.
****************************************************************/


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

// Global Variables
APDS9960_STM apds = APDS9960_STM();
HardWire HWire(1, I2C_FAST_MODE); // I2c1
uint8_t proximity_data = 0;

void setup() {


  // Use this initializer if you're using a 1.8" TFT
  tft.initR(INITR_BLACKTAB);   // initialize a ST7735S chip, black tab

  // Use this initializer (uncomment) if you're using a 1.44" TFT
  //tft.initR(INITR_144GREENTAB);   // initialize a ST7735S chip, black tab

  tft.fillScreen(ST7735_BLACK);

  // Initialize Serial port
  Serial.begin(9600);
  Serial.println();
  delay(3000);

  // Initialize APDS-9960 (configure I2C and initial values)
  if ( apds.init() ) {
    Serial.println(F("APDS-9960 initialization complete"));
  } else {
    Serial.println(F("Something went wrong during APDS-9960 init!"));
  }

  // Adjust the Proximity sensor gain
  if ( !apds.setProximityGain(PGAIN_2X) ) {
    Serial.println(F("Something went wrong trying to set PGAIN"));
  }

  // Start running the APDS-9960 proximity sensor (no interrupts)
  if ( apds.enableProximitySensor(false) ) {
    Serial.println(F("Proximity sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during sensor init!"));
  }
}

void loop() {

  tft.setCursor(0, 8);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.println("APDS-9960 Proximity");

  for (;;)
  {
    tft.setCursor(0, 30);
    // Read the proximity value
    if ( !apds.readProximity(proximity_data) ) {
      Serial.println("Error reading proximity value");
      tft.setTextColor(ST7735_RED, ST7735_BLACK);
      tft.println("Error ");
      tft.println("      ");
    } else {
      Serial.print("Proximity: ");
      Serial.println(proximity_data);
      tft.setCursor(0, 30);
      tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
      tft.println("Data: ");
      tft.print(proximity_data, DEC);
      tft.println("      ");
    }

    // Wait 250 ms before next reading
    delay(250);
  }
}