/*
  Using the SparkFun Qwiic 12 Bit ADC - 4 Channel ADS1015
  By: Paul Clark
  Based on original examples by: Pete Lewis, Original flex-glove library by: Andy England
  SparkFun Electronics
  Date: October 28th, 2022
  License: MIT
  
  Feel like supporting our work? Please buy a board from SparkFun!
  https://www.sparkfun.com/products/15334

  This example shows how to output ADC values on multiple channels.
  
  It is based on the example provided by @StefanThoresson in Issue #6:
  https://github.com/sparkfun/SparkFun_ADS1015_Arduino_Library/issues/6
  Thank you Stefan.

  We used this example to determine the delay values for conversionDelay.

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

void setup()
{
  delay(1000); // Power up delay (for Serial)
  
  Wire.begin();
  Serial.begin(115200);                             //changed to 115200 instead of 9600
  
  if (adcSensor.begin(0x48) == true) // 0x48 is default
  //if (adcSensor.begin(0x48) == true) // ADDR connected to GND
  //if (adcSensor.begin(0x49) == true) // ADDR connected to VCC
  //if (adcSensor.begin(0x4A) == true) // ADDR connected to SDA. Not recommended due to timing in datasheet..
  //if (adcSensor.begin(0x4B) == true) // ADDR connected to SCL
  {
    Serial.println("Device found. I2C connections are good.");
  }
  else
  {
    Serial.println("Device not found. Check wiring.");
    while (1); // stall out forever
  }

  // Set the Gain
  // Possible values are:
  // ADS1015_CONFIG_PGA_TWOTHIRDS - 0x0000 - gain:2/3, input range:± 6.144V
  // ADS1015_CONFIG_PGA_1 -         0X0200 - gain:1, input range: ± 4.096V
  // ADS1015_CONFIG_PGA_2 -         0X0400 - gain:2, input range: ± 2.048V (Default)
  // ADS1015_CONFIG_PGA_4 -         0X0600 - gain:4, input range: ± 1.024V
  // ADS1015_CONFIG_PGA_8 -         0X0800 - gain:8, input range: ± 0.512V
  // ADS1015_CONFIG_PGA_16 -        0X0A00 - gain:16, input range: ± 0.256V 
  adcSensor.setGain(ADS1015_CONFIG_PGA_2);

  // Set the Sample Rate
  // Possible values are:
  // ADS1015_CONFIG_RATE_128HZ  - 0x0000 : 128Hz
  // ADS1015_CONFIG_RATE_250HZ  - 0X0020 : 250Hz
  // ADS1015_CONFIG_RATE_490HZ  - 0X0040 : 490Hz
  // ADS1015_CONFIG_RATE_920HZ  - 0X0060 : 920Hz
  // ADS1015_CONFIG_RATE_1600HZ - 0X0080 : 1600Hz (Default)
  // ADS1015_CONFIG_RATE_2400HZ - 0X00A0 : 2400Hz
  // ADS1015_CONFIG_RATE_3300HZ - 0X00C0 : 3300Hz
  adcSensor.setSampleRate(ADS1015_CONFIG_RATE_1600HZ);

  // For the fastest conversion timing, we need to check the Config Register Operational Status bit
  // to see when the conversion is complete - instead of using a fixed delay.
  // However, we can only do this if we use single-shot mode.
  // Because this breaks backward-compatibility, _useConversionReady is disabled by default.
  // To enable it, call:
  adcSensor.useConversionReady(true);
}

void loop()
{
  int16_t channel_A0 = adcSensor.getSingleEndedSigned(0);
  int16_t channel_A1 = adcSensor.getSingleEndedSigned(1);

  float channel_A0_mV = adcSensor.getSingleEndedMillivolts(0);
  float channel_A1_mV = adcSensor.getSingleEndedMillivolts(1);

  // getDifferential options are:
  // ADS1015_CONFIG_MUX_DIFF_P0_N1 : A0-A1 (Default)
  // ADS1015_CONFIG_MUX_DIFF_P0_N3 : A0-A3
  // ADS1015_CONFIG_MUX_DIFF_P1_N3 : A1-A3
  // ADS1015_CONFIG_MUX_DIFF_P2_N3 : A2-A3
  int16_t channel_A0_A1 = adcSensor.getDifferential(ADS1015_CONFIG_MUX_DIFF_P0_N1);

  float channel_A0_A1_mV = adcSensor.getDifferentialMillivolts(ADS1015_CONFIG_MUX_DIFF_P0_N1);
  
  Serial.print("A0: ");
  Serial.print(channel_A0);
  Serial.print(" (");
  Serial.print(channel_A0_mV);
  Serial.print("mV)\t");
 
  Serial.print("A1: ");
  Serial.print(channel_A1);
  Serial.print(" (");
  Serial.print(channel_A1_mV);
  Serial.print("mV)\t");

  Serial.print("A0-A1: "); 
  Serial.print(channel_A0_A1);
  Serial.print(" (");
  Serial.print(channel_A0_A1_mV);
  Serial.print("mV)\t");

  Serial.println();

  //Don't delay here. Go as fast as possible. The Serial.print's will set the rate...
}
