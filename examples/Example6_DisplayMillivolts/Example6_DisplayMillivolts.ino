/*
  Using the SparkFun Qwiic 12 Bit ADC - 4 Channel ADS1015
  By: Pete Lewis, Original flex-glove library by: Andy England
  SparkFun Electronics
  Date: May 9, 2019
  License: This code is public domain but you can buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Please buy a board from SparkFun!
  https://www.sparkfun.com/products/15334

  This example shows how to output ADC values on one single-ended channel (A3).
  It will also print out the voltage of the reading in mV.
  Note, the conversion multiplier needs to be corrected for gain settings,
  And this is done automatically in the library, using .getMultiplier(),
  as shown below.
  
  *at gain setting of 1, like in this example (and 3.3V VCC), 0-3.3V will read 0-1652.

  Hardware Connections and initial setup:
  Plug in your controller board (e.g. Redboard Qwiic) into your computer with USB cable.
  Connect your Qwiic 12 Bit ADC board to your controller board via a qwiic cable.
  Select TOOLS>>BOARD>>"Arduino/Genuino Uno"
  Select TOOLS>>PORT>> "COM 3" (note, yours may be different)
  Click upload, and watch streaming data over serial monitor at 9600.

*/

#include <SparkFun_ADS1015_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_ADS1015
#include <Wire.h>

ADS1015 adcSensor;

void setup() {
  Wire.begin();
  Serial.begin(9600);
  if (adcSensor.begin() == true)
  {
    Serial.println("Device found. I2C connections are good.");
  }
  else
  {
    Serial.println("Device not found. Check wiring.");
    while (1); // stall out forever
  }
  adcSensor.setGain(ADS1015_CONFIG_PGA_1);
}

void loop() {
  uint16_t channel_A3 = adcSensor.getSingleEnded(3);
  Serial.print("A3:");
  Serial.print(channel_A3);
  Serial.print("\t(");
  float multiplier = adcSensor.getMultiplier(); // used to convert readings to actual voltages (in mV units)
  // the private varaible _multiplierToVolts is auto-updated each time setGain is called
  Serial.print(channel_A3 * multiplier);
  Serial.println("mV)");
  delay(50); // avoid bogging up serial monitor
}
