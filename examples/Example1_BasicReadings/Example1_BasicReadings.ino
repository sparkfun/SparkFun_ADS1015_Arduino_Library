#include <SparkFun_ADS1015_Arduino_Library.h>

ADS1015 fingerSensor;

void setup() {
  
  Wire.begin();
  
  if (fingerSensor.begin(Wire, 100000, ADS1015_ADDRESS_VDD, 9600) == false) {
     Serial.println("Device not found. Check wiring.");
     while (1);
  }
  
}

void loop() {  
  uint16_t data;
  for (int finger = 0; finger < 2; finger++) {
    data = fingerSensor.getAnalogData(finger);
    Serial.print(finger);
    Serial.print(": ");
    Serial.print(data);
    Serial.print(",");
  }
  Serial.println();
}
