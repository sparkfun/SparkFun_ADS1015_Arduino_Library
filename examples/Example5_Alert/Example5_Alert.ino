/*
  Using the SparkFun Qwiic 12 Bit ADC - 4 Channel ADS1015
  By: Pete Lewis, Original flex-glove library by: Andy England
  SparkFun Electronics
  Date: May 9, 2019
  License: This code is public domain but you can buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Please buy a board from SparkFun!
  https://www.sparkfun.com/products/15334

  This example shows how to setup the ALERT pin as active LOW with a threshold of 1000 (3V) reactive to A3.
  This means that when A3 sees a voltage of 3V, it will drive the ALERT pin low.
  Then you must read the last conversion, to reset it.
  If A3 sees a voltage below the threshold, it will remain driven HIGH.

  Note, this example prints out the status of the ALERT pin (being read by arduino pin D2).
  It also prints out the value on A3, for reference.

  Hardware Connections and initial setup:
  Plug in your controller board (e.g. Redboard Qwiic) into your computer with USB cable.
  Connect your Qwiic 12 Bit ADC board to your controller board via a qwiic cable.
  Connect ALERT pin to arduino pin D2.
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

  // Setup 3V comparator on channel A3
  adcSensor.setComparatorSingleEnded(3, 1000);
  // setup Arduino pin D2 as input pullup; this should be connected to ALERT pin.
  pinMode(2, INPUT_PULLUP);
}

void loop() {
  uint16_t channel_A3;
  int alert = digitalRead(2);
  Serial.print("ALERT:");
  Serial.print(alert);
  channel_A3 = adcSensor.getLastConversionResults();
  Serial.print("\t\tA3:");
  Serial.print(channel_A3);
  if (alert == LOW)
  {
    Serial.print("\t\tALERT!");
  }
  Serial.println();
  delay(50); // avoid bogging up serial monitor
}