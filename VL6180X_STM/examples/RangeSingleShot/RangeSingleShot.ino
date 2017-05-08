/* This minimal example shows how to get single-shot range
measurements from the VL6180X.

The range readings are in units of mm. */

#include <HardWire.h>
#include <VL6180X_STM.h>

HardWire HWire(1, I2C_FAST_MODE); // I2c1
VL6180X_STM sensor;

void setup() 
{
  Serial.begin(9600);
  HWire.begin();
  
  sensor.init();
  sensor.configureDefault();
  sensor.setTimeout(500);
}

void loop() 
{ 
  Serial.print(sensor.readRangeSingleMillimeters());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
  
  Serial.println();
}
