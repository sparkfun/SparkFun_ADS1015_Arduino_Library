#include <SparkFun_ADS1015_Arduino_Library.h>
#include <Wire.h>

ADS1015 fingerSensor;

void setup() {
  
  Wire.begin();
  Serial.begin(115200);
  if (fingerSensor.begin(Wire, 100000, ADS1015_ADDRESS_GND) == false) {
     Serial.println("Device not found. Check wiring.");
     while (1);
  }
}

void loop() {
  uint8_t incoming;
  do
  {
    fingerSensor.calibrate();

    if(Serial.available())
    {
       incoming = Serial.read();
    }
  } while (incoming != 'e');
  Serial.println("calibrated");
}
