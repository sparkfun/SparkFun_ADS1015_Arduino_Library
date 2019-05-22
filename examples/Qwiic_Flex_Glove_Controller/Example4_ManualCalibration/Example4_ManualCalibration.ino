/*
  Using the Qwiic Flex Glvoe Controller
  By: Andy England
  SparkFun Electronics
  Date: July 17, 2018
  License: This code is public domain but you buy me a beer if you use this and we meet someday (Beerware license).

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14666

  This example shows how to output accelerometer values

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 115200 baud to serial monitor.
*/

#include <SparkFun_ADS1015_Arduino_Library.h>
#include <Wire.h>

ADS1015 pinkySensor;
ADS1015 indexSensor;
float hand[4] = {0, 0, 0, 0};
//Calibration Array
uint16_t handCalibration[4][2] = {
//{low, hi} switch these to reverse which end is 1 and which is 0
  {722, 1080}, //index
  {600, 980},  //middle
  {680, 900},  //ring
  {736, 907}   //pinky
};

void setup() {
  
  Wire.begin();
  Serial.begin(115200);
  
  //Begin our finger sensors, change addresses as needed.
  if (pinkySensor.begin(ADS1015_ADDRESS_SDA) == false) 
  {
     Serial.println("Pinky not found. Check wiring.");
     while (1);
  }
  if (indexSensor.begin(ADS1015_ADDRESS_GND) == false) 
  {
     Serial.println("Index not found. Check wiring.");
     while (1);
  }
  
  pinkySensor.setGain(ADS1015_CONFIG_PGA_TWOTHIRDS); // Gain of 2/3 to works well with flex glove board voltage swings (default is gain of 2)
  indexSensor.setGain(ADS1015_CONFIG_PGA_TWOTHIRDS); // Gain of 2/3 to works well with flex glove board voltage swings (default is gain of 2)  
  
  //Set the calibration values for the hand.
  for (int channel; channel < 2; channel++)
  {
    for (int hiLo = 0; hiLo < 2; hiLo++)
    {
      indexSensor.setCalibration(channel, hiLo, handCalibration[channel][hiLo]);
      pinkySensor.setCalibration(channel, hiLo, handCalibration[channel + 2][hiLo]);
    }
    Serial.println();
  }
}

void loop() {
  for (int channel = 0; channel < 2; channel++)
  {
    //Keep in mind that getScaledAnalogData returns a float
    hand[channel] = indexSensor.getScaledAnalogData(channel);
    hand[channel + 2] = pinkySensor.getScaledAnalogData(channel);
  }
  for (int finger = 0; finger < 4; finger++)
  {
    Serial.print(finger);
    Serial.print(": ");
    Serial.print(hand[finger]);
    Serial.print(" ");
  }
  Serial.println();
}
