/*
  Using the SparkFun Qwiic 12 Bit ADC - 4 Channel ADS1015
  By: Pete Lewis, Original flex-glove library by: Andy England
  SparkFun Electronics
  Date: May 9, 2019
  License: This code is public domain but you can buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Please buy a board from SparkFun!
  https://www.sparkfun.com/products/15334

  This example shows how to output ADC values on one single-ended channel (A3) with a PGA GAIN of 1. 
  At this gain setting (and 3.3V VCC), 0-3.3V will read 0-1652.

  Other possible gain settings are as follows. 
  Note, changing the gain effects the range of the sensor (aka the max and min voltages it can read).
  Also note, to avoid damaging your ADC, never exceed VDD (3.3V for a qwiic system).

  ADS1015_CONFIG_PGA_TWOTHIRDS  +/- 6.144v
  ADS1015_CONFIG_PGA_1          +/- 4.096v (used in this example)
  ADS1015_CONFIG_PGA_2          +/- 2.048v
  ADS1015_CONFIG_PGA_4          +/- 1.024v
  ADS1015_CONFIG_PGA_8          +/- 0.512v
  ADS1015_CONFIG_PGA_16         +/- 0.256v

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
  adcSensor.setGain(ADS1015_CONFIG_PGA_1); // PGA gain set to 1
}

void loop() {
  uint16_t channel_A3 = adcSensor.getSingleEnded(3);
  Serial.print("A3:");
  Serial.println(channel_A3);
  delay(50); // avoid bogging up serial monitor
}
