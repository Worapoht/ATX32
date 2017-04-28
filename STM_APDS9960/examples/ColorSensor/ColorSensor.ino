/****************************************************************
  ATX-32 with APDS9960 ColorSensor

  Tests the color and ambient light sensing abilities of the
  APDS-9960. Configures APDS-9960 over I2C and polls the sensor for
  ambient light and color levels, which are displayed over the
  serial console.

  displays it on a GLCD

  Hardware Connections:

  ATX-32 Pin   APDS-9960 Board  Function

  3.3V         VCC              Power
  GND          GND              Ground
  PB7          SDA              I2C Data
  PB6          SCL              I2C Clock
  PE0          INT              Interrupt

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
#include <STM_APDS9960.h>


// Global Variables
STM_APDS9960 apds = STM_APDS9960();
HardWire HWire(1, I2C_FAST_MODE); // I2c1

uint16_t ambient_light = 0;
uint16_t red_light = 0;
uint16_t green_light = 0;
uint16_t blue_light = 0;

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

  // Start running the APDS-9960 light sensor (no interrupts)
  if ( apds.enableLightSensor(false) ) {
    Serial.println(F("Light sensor is now running"));
  } else {
    Serial.println(F("Something went wrong during light sensor init!"));
  }

  // Wait for initialization and calibration to finish
  delay(500);
}

void loop() {

  tft.setCursor(0, 8);
  tft.setTextColor(ST7735_WHITE, ST7735_BLACK);
  tft.println("APDS-9960 Color");
  tft.setCursor(12, 16);
  tft.println("Sensor");

  for (;;)
  {
    // Read the light levels (ambient, red, green, blue)
    tft.setCursor(0, 30);
    tft.setTextColor(ST7735_YELLOW, ST7735_BLACK);
    if (  !apds.readAmbientLight(ambient_light) ||
          !apds.readRedLight(red_light) ||
          !apds.readGreenLight(green_light) ||
          !apds.readBlueLight(blue_light) ) {
      Serial.println("Error reading light values");
      tft.println("Error ");
      tft.println("      ");
      tft.println("      ");
    } else {
      Serial.print("Ambient: ");
      Serial.print(ambient_light);
      Serial.print(" Red: ");
      Serial.print(red_light);
      Serial.print(" Green: ");
      Serial.print(green_light);
      Serial.print(" Blue: ");
      Serial.println(blue_light);
      tft.setCursor(0, 30);
      tft.setTextColor(ST7735_RED, ST7735_BLACK);
      tft.print("R:");
      tft.print(red_light, DEC);
      tft.println("    ");
      tft.setTextColor(ST7735_GREEN, ST7735_BLACK);
      tft.print("G:");
      tft.print(green_light, DEC);
      tft.println("    ");
      tft.setTextColor(ST7735_BLUE, ST7735_BLACK);
      tft.print("B:");
      tft.print(blue_light, DEC);
      tft.println("    ");
    }
    // Wait 1 second before next reading
    delay(1000);
  }
}