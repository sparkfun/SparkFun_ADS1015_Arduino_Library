/*
  This is a library written for the ADS1015 Human Presence Sensor.

  Written by Nathan Seidle @ SparkFun Electronics, March 10th, 2017
  Revised by Andy England @ SparkFun Electronics, October 17th, 2017

  The sensor uses I2C to communicate, as well as a single (optional)
  interrupt line that is not currently supported in this driver.

  https://github.com/sparkfun/SparkFun_ADS1015_Arduino_Library

  Do you like this library? Help support SparkFun. Buy a board!

  Development environment specifics:
  Arduino IDE 1.8.1

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "SparkFun_ADS1015_Arduino_Library.h"

//Sets up the sensor for constant read
//Returns false if sensor does not respond
boolean ADS1015::begin(TwoWire &wirePort, uint32_t i2cSpeed, uint8_t i2caddr, long baud)
{
  //Bring in the user's choices
  _i2cPort = &wirePort; //Grab which port the user wants us to use

  _i2cPort->begin();
  _i2cPort->setClock(i2cSpeed);

  _i2caddr = i2caddr;

  Serial.begin(baud);

  //setMode(ADS1015_CONFIG_MODE_CONT); //Set to continuous read

  return (true); //Success!
}

//Returns the decimal value of sensor channel
int16_t ADS1015::getAnalogData(uint8_t channel)
{
	if (channel > 3) {
		return 0;
	}
	
	uint16_t config = ADS1015_CONFIG_OS_SINGLE   |
					  ADS1015_CONFIG_MODE_SINGLE |
					  ADS1015_CONFIG_RATE_2400HZ;
					  
	
	switch (channel)
    {
    case (0):
        config |= ADS1015_CONFIG_MUX_SINGLE_0;
        break;
    case (1):
        config |= ADS1015_CONFIG_MUX_SINGLE_1;
        break;
    case (2):
        config |= ADS1015_CONFIG_MUX_SINGLE_2;
        break;
    case (3):
        config |= ADS1015_CONFIG_MUX_SINGLE_3;
        break;
    }
	
	writeRegister(ADS1015_POINTER_CONFIG, config);
	delay(ADS1015_DELAY);
	
    return readRegister(ADS1015_POINTER_CONVERT) >> 4;
}

//Set the mode. Continuous mode 0 is favored
/*void ADS1015::setMode(uint8_t mode)
{
  if (mode > ADS1015_MODE_3) mode = ADS1015_MODE_0; //Default to mode 0
  if (mode == 0b011) mode = ADS1015_MODE_0; //0x03 is prohibited

  //Read, mask set, write
  byte currentSettings = readRegister(ADS1015_ECNTL1);

  currentSettings &= 0b11111000; //Clear Mode bits
  currentSettings |= mode;
  writeRegister(ADS1015_ECNTL1, currentSettings);
}*/

/*//Checks to see if DRDY flag is set in the status register
boolean ADS1015::available()
{
  uint8_t value = readRegister(ADS1015_ST1);
  return (value & (1 << 0)); //Bit 0 is DRDY
}


//Does a soft reset
void ADS1015::softReset()
{
  writeRegister(ADS1015_CNTL2, 0xFF);
}*/
/*
//Turn on/off Serial.print statements for debugging
void ADS1015::enableDebugging(Stream &debugPort)
{
  _debugSerial = &debugPort; //Grab which port the user wants us to use for debugging

  _printDebug = true; //Should we print the commands we send? Good for debugging
}
void ADS1015::disableDebugging()
{
  _printDebug = false; //Turn off extra print statements
}*/

//Reads from a give location
uint16_t ADS1015::readRegister(uint8_t location)
{
  _i2cPort->beginTransmission(_i2caddr);
  _i2cPort->write(ADS1015_POINTER_CONVERT);
  _i2cPort->endTransmission();
  _i2cPort->requestFrom(_i2caddr, 2); //Ask for one byte
  return (_i2cPort->read() << 8 | _i2cPort->read());
}

//Write a value to a spot
void ADS1015::writeRegister(uint8_t location, uint16_t val)
{
  _i2cPort->beginTransmission(_i2caddr);
  _i2cPort->write(location);
  _i2cPort->write((uint8_t)(val >> 8));
  _i2cPort->write((uint8_t)(val & 0xFF));
  _i2cPort->endTransmission();
}

//Reads a two byte value from a consecutive registers
uint16_t ADS1015::readRegister16(byte location)
{
  uint8_t result;
  _i2cPort->beginTransmission(_i2caddr);
  _i2cPort->write(ADS1015_POINTER_CONVERT);
  Serial.println("AYYo");
  result = _i2cPort->endTransmission();
  Serial.println("AYYo");
  _i2cPort->requestFrom((int)_i2caddr, 2);

  uint16_t data = _i2cPort->read();
  data |= (_i2cPort->read() << 8);

  return (data);
}

/** Private functions ***********************/

//If I2C communication fails this function will tell us which error occured
//Originally from Robotic Materials: https://github.com/RoboticMaterials/FA-I-sensor/blob/master/force_proximity_eval/force_proximity_eval.ino
/*uint8_t ADS1015::printI2CError(uint8_t errorCode)
{
  if (_printDebug == true)
  {
    switch (errorCode)
    {
      //From: https://www.arduino.cc/en/Reference/WireEndTransmission
      case 0:
        _debugSerial->println(F("Success"));
        break;
      case 1:
        _debugSerial->println(F("Data too long to fit in transmit buffer"));
        break;
      case 2:
        _debugSerial->println(F("Received NACK on transmit of address"));
        break;
      case 3:
        _debugSerial->println(F("Received NACK on transmit of data"));
        break;
      case 4:
        _debugSerial->println(F("Unknown error"));
        break;
    }
  }
  return (errorCode); //No matter what pass the code back out
}*/

