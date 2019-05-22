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

ADS1015 fingerSensor;

void setup() {
  
  Wire.begin();
  Serial.begin(115200);
  
  if (fingerSensor.begin(ADS1015_ADDRESS_GND) == false) {
     Serial.println("Device not found. Check wiring.");
     while (1);
  }
  fingerSensor.setGain(ADS1015_CONFIG_PGA_TWOTHIRDS); // Gain of 2/3 to works well with flex glove board voltage swings (default is gain of 2)
  
  Serial.println("Calibrating, flex all sensors through full range of motion multiple times, send 'e' when finished");
}

void loop() {
  uint8_t incoming;
  //Check for incoming serial data while we calibrate the sensors.
  do
  {
    fingerSensor.calibrate();

    if(Serial.available())
    {
       incoming = Serial.read();
    }
  } while (incoming != 'e');
  Serial.println("Calibrated");

  //Print calibration data
  for (int channel; channel < 2; channel++)
  {
    Serial.print("Channel ");
    Serial.print(channel);
    Serial.print(": ");
    for (int hiLo = 0; hiLo < 2; hiLo++)
    {
      switch (hiLo)
      {
        case 0:
        Serial.print("Low: ");
        Serial.print(fingerSensor.getCalibration(channel, hiLo));
        break;
        case 1:
        Serial.print(" High: ");
        Serial.print(fingerSensor.getCalibration(channel, hiLo));
        break;
      }
    }
    Serial.println();
  }
}
