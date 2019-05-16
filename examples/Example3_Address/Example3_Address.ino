/*
  Using the SparkFun Qwiic 12 Bit ADC - 4 Channel ADS1015
  By: Pete Lewis, Original flex-glove library by: Andy England
  SparkFun Electronics
  Date: May 9, 2019
  License: This code is public domain but you can buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Please buy a board from SparkFun!
  https://www.sparkfun.com/products/15334

  This example shows how to output ADC values on one single-ended channel (A3) with a NON-default address.
  This is useful if you'd like to connect multiple ADS1015 boards on the same bus.
  Note, you must cut a trace on the product hardware and solder to "0x49" jumper for this code to work.

  Hardware Connections and initial setup:
  On the bottom of the board, cut default addr trace, and close desired address jumper.
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
  if (adcSensor.begin(0x49) == true) // connect to device at address 0x49 (default is 0x48)
    // **note, you must cut a trace and close the "0x49" jumper for this to work.
  {
    Serial.println("Device found. I2C connections are good.");
  }
  else
  {
    Serial.println("Device not found. Check wiring.");
    while (1); // stall out forever
  }
}

void loop() {
  uint16_t channel_A3 = adcSensor.getSingleEnded(3);
  Serial.print("A3:");
  Serial.println(channel_A3);
  delay(50); // avoid bogging up serial monitor
}
