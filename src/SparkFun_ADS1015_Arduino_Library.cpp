/*
  This is a library written for the ADS1015 ADC->I2C.

  Written by Andy England @ SparkFun Electronics, October 17th, 2017

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

#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
	//Teensy 3.6
boolean ADS1015::begin(i2c_t3 &wirePort, uint32_t i2cSpeed, uint8_t i2caddr)
{
  //Bring in the user's choices
  _i2cPort = &wirePort; //Grab which port the user wants us to use

  _i2cPort->setClock(i2cSpeed);

  _i2caddr = i2caddr;

  return (true); //Success!
}
#else

boolean ADS1015::begin(TwoWire &wirePort, uint32_t i2cSpeed, uint8_t i2caddr)
{
  //Bring in the user's choices
  _i2cPort = &wirePort; //Grab which port the user wants us to use

  _i2cPort->setClock(i2cSpeed);

  _i2caddr = i2caddr;

  return (true); //Success!
}
#endif

//Returns the decimal value of sensor channel
uint16_t ADS1015::getAnalogData(uint8_t channel)
{
	if (channel > 3) {
		return 0;
	}
	
	uint16_t config = ADS1015_CONFIG_OS_SINGLE   |
					  _mode |
					  _sampleRate;
			
	config |= _gain;		  
	
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

//Returns a value between 0 and 1 based on how bent the finger is. This function will not work with an uncalibrated sensor
float ADS1015::getScaledAnalogData (uint8_t channel)
{
	float data = mapf(getAnalogData(channel), calibrationValues[channel][0], calibrationValues[channel][1], 0, 1);
	if (data > 1)
	{
		return 1;
	}
	else if (data < 0)
	{
		return 0;
	}
	else
	{
		return data;
	}
}

void ADS1015::calibrate ()
{
	for (int finger = 0; finger < 2; finger++)
	{
		uint16_t value = getAnalogData(finger);
		if ((value > calibrationValues[finger][1] || calibrationValues[finger][1] == 0) && value < 1085)
		{
			calibrationValues[finger][1] = value;
		}
		else if (value < calibrationValues[finger][0] || calibrationValues[finger][0] == 0)
		{
			calibrationValues[finger][0] = value;
		}
	}
}

uint16_t ADS1015::getCalibration(uint8_t channel, bool hiLo)
{
	return calibrationValues[channel][hiLo];
}

void ADS1015::setCalibration(uint8_t channel, bool hiLo, uint16_t value)
{
	calibrationValues[channel][hiLo] = value;
}

void ADS1015::resetCalibration()
{
	for (int channel = 0; channel < 2; channel++)
	{
		for (int hiLo = 0; hiLo < 2; hiLo++)
		{
			calibrationValues[channel][hiLo] = 0;			
		}
	}
}

float ADS1015::mapf(float val, float in_min, float in_max, float out_min, float out_max) {
	return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//Set the mode. Continuous mode 0 is favored
void ADS1015::setMode(uint16_t mode)
{
  _mode = mode;
}

//getMode will return 0 for continuous and 1 for single shot
uint16_t ADS1015::getMode ()
{
  return _mode;
}

void ADS1015::setGain (uint16_t gain)
{
	_gain = gain;
}

//Will return a different hex value for each gain
//0x0E00: +/- 0.256V
//0X0000: +/- 6.144V
//0X0200: +/- 4.096V
//0X0400: +/- 2.048V
//0X0600: +/- 1.024V
//0X0800: +/- 0.512V
//0X0A00: +/- 0.256V
uint16_t ADS1015::getGain ()
{
	return _gain;
}

void ADS1015::setSampleRate (uint16_t sampleRate)
{
	_sampleRate = sampleRate;
}

//Will return a different hex value for each sample rate
//0x0000: 128 Hz
//0X0020: 250 Hz
//0X0040: 490 Hz
//0X0060: 920 Hz
//0X0080: 1600 Hz
//0X00A0: 2400 Hz
//0X00C0: 3300 Hz
uint16_t ADS1015::getSampleRate ()
{
	return _sampleRate;
}

//Checks to see if DRDY flag is set in the status register
boolean ADS1015::available()
{
  uint16_t value = readRegister(ADS1015_POINTER_CONFIG);
  return (value & (1 << 0)); //Bit 0 is DRDY
}

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
  result = _i2cPort->endTransmission();
  _i2cPort->requestFrom((int)_i2caddr, 2);

  uint16_t data = _i2cPort->read();
  data |= (_i2cPort->read() << 8);

  return (data);
}

