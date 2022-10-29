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

// Sets up the sensor for constant read
// Returns false if sensor does not respond

#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
// Teensy 3.6
bool ADS1015::begin(uint8_t i2caddr, i2c_t3 &wirePort)
{
  // Bring in the user's choices
  _i2cPort = &wirePort; // Grab which port the user wants us to use

  _i2caddr = i2caddr;

  if (isConnected() == false)
    return (false); // Check for sensor presence

  return (true); // We're all setup!
}
#else

bool ADS1015::begin(uint8_t i2caddr, TwoWire &wirePort)
{
  // Bring in the user's choices
  _i2cPort = &wirePort; // Grab which port the user wants us to use

  _i2caddr = i2caddr;

  if (isConnected() == false)
    return (false); // Check for sensor presence

  return (true); // We're all setup!
}
#endif

// Returns true if I2C device ack's
bool ADS1015::isConnected()
{
  _i2cPort->beginTransmission((uint8_t)_i2caddr);
  if (_i2cPort->endTransmission() != 0)
    return (false); // Sensor did not ACK
  return (true);
}

// Returns the decimal value of sensor channel single-ended input
uint16_t ADS1015::getSingleEnded(uint8_t channel)
{
  if (channel > 3)
  {
    return 0;
  }

  uint16_t config = ADS1015_CONFIG_OS_SINGLE |
                    ADS1015_CONFIG_CQUE_NONE |
                    _sampleRate;

  config |= _gain;

  if (_useConversionReady)
    config |= ADS1015_CONFIG_MODE_SINGLE;
  else
    config |= _mode;

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

  if (_useConversionReady)
  {
    while (!available())
      yield();
  }
  else
    conversionDelay();

  return readRegister(ADS1015_POINTER_CONVERT) >> 4;
}

// Returns the sensor channel single-ended input as int16_t (two's complement)
int16_t ADS1015::getSingleEndedSigned(uint8_t channel)
{
  uint16_t result = getSingleEnded(channel);
  if (result > 0x07FF)
  {
    // negative number - extend the sign to 16th bit
    result |= 0xF000;
  }
  return (convertUnsignedToSigned(result)); // Convert without ambiguity
}

// Returns the sensor channel single-ended input as float in millivolts
float ADS1015::getSingleEndedMillivolts(uint8_t channel)
{
  return (((float)getSingleEndedSigned(channel)) * getMultiplier());
}

// Returns the *signed* decimal value of sensor differential input
// Note, there are 4 possible differential pin setups:
// ADS1015_CONFIG_MUX_DIFF_P0_N1
// ADS1015_CONFIG_MUX_DIFF_P0_N3
// ADS1015_CONFIG_MUX_DIFF_P1_N3
// ADS1015_CONFIG_MUX_DIFF_P2_N3
int16_t ADS1015::getDifferential(uint16_t CONFIG_MUX_DIFF)
{
  // check for valid argument input
  if (
      (CONFIG_MUX_DIFF == ADS1015_CONFIG_MUX_DIFF_P0_N1) ||
      (CONFIG_MUX_DIFF == ADS1015_CONFIG_MUX_DIFF_P0_N3) ||
      (CONFIG_MUX_DIFF == ADS1015_CONFIG_MUX_DIFF_P1_N3) ||
      (CONFIG_MUX_DIFF == ADS1015_CONFIG_MUX_DIFF_P2_N3))
  {
    // valid argument; do nothing and then carry on below
  }
  else
  {
    return 0; // received invalid argument
  }

  uint16_t config = ADS1015_CONFIG_OS_SINGLE |
                    ADS1015_CONFIG_CQUE_NONE |
                    _sampleRate;

  config |= _gain;

  if (_useConversionReady)
    config |= ADS1015_CONFIG_MODE_SINGLE;
  else
    config |= _mode;

  config |= CONFIG_MUX_DIFF; // default is ADS1015_CONFIG_MUX_DIFF_P0_N1

  writeRegister(ADS1015_POINTER_CONFIG, config);

  if (_useConversionReady)
  {
    while (!available())
      yield();
  }
  else
    conversionDelay();

  uint16_t result = readRegister(ADS1015_POINTER_CONVERT) >> 4;

  // making sure we keep the sign bit intact
  if (result > 0x07FF)
  {
    // negative number - extend the sign to 16th bit
    result |= 0xF000;
  }

  return (convertUnsignedToSigned(result)); // Convert to signed without ambiguity
}

// Returns the sensor channel single-ended input as float in millivolts
float ADS1015::getDifferentialMillivolts(uint16_t CONFIG_MUX_DIFF)
{
  return (((float)getDifferential(CONFIG_MUX_DIFF)) * getMultiplier());
}

// antiquated function from older library, here for backwards compatibility
uint16_t ADS1015::getAnalogData(uint8_t channel)
{
  return getSingleEnded(channel);
}

// Returns a value between 0 and 1 based on how bent the finger is. This function will not work with an uncalibrated sensor
float ADS1015::getScaledAnalogData(uint8_t channel)
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

void ADS1015::calibrate()
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

float ADS1015::mapf(float val, float in_min, float in_max, float out_min, float out_max)
{
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Set the mode. Continuous mode 0 is favored
void ADS1015::setMode(uint16_t mode)
{
  mode &= ADS1015_CONFIG_MODE_SINGLE;
  _mode = mode;
}

// getMode will return 0 for continuous and ADS1015_CONFIG_MODE_SINGLE for single shot
uint16_t ADS1015::getMode()
{
  return _mode;
}

void ADS1015::setGain(uint16_t gain)
{
  gain &= ADS1015_CONFIG_PGA_MASK;
  _gain = gain;
  updateMultiplierToVolts(); // each new gain setting changes how we convert to volts
}

// Will return a different hex value for each gain
// 0x0E00: +/- 0.256V
// 0X0000: +/- 6.144V
// 0X0200: +/- 4.096V
// 0X0400: +/- 2.048V
// 0X0600: +/- 1.024V
// 0X0800: +/- 0.512V
// 0X0A00: +/- 0.256V
uint16_t ADS1015::getGain()
{
  return _gain;
}

void ADS1015::updateMultiplierToVolts()
{
  switch (_gain)
  {
  case (ADS1015_CONFIG_PGA_TWOTHIRDS):
    _multiplierToVolts = 3.0F;
    break;
  case (ADS1015_CONFIG_PGA_1):
    _multiplierToVolts = 2.0F;
    break;
  case (ADS1015_CONFIG_PGA_2):
    _multiplierToVolts = 1.0F;
    break;
  case (ADS1015_CONFIG_PGA_4):
    _multiplierToVolts = 0.5F;
    break;
  case (ADS1015_CONFIG_PGA_8):
    _multiplierToVolts = 0.25F;
    break;
  case (ADS1015_CONFIG_PGA_16):
    _multiplierToVolts = 0.125F;
    break;
  default:
    _multiplierToVolts = 1.0F;
  }
}

float ADS1015::getMultiplier()
{
  return _multiplierToVolts;
}

void ADS1015::setSampleRate(uint16_t sampleRate)
{
  sampleRate &= ADS1015_CONFIG_RATE_MASK;
  _sampleRate = sampleRate;
}

// Will return a different hex value for each sample rate
// 0x0000: 128 Hz
// 0X0020: 250 Hz
// 0X0040: 490 Hz
// 0X0060: 920 Hz
// 0X0080: 1600 Hz
// 0X00A0: 2400 Hz
// 0X00C0: 3300 Hz
uint16_t ADS1015::getSampleRate()
{
  return _sampleRate;
}

// Checks to see if the Operational Status (OS) flag is set in the status register
bool ADS1015::available()
{
  uint16_t value = readRegister(ADS1015_POINTER_CONFIG);
  return ((value & ADS1015_CONFIG_OS_READY) > 0); // If the OS bit is 1 : the device is not currently performing a conversion (i.e. data is available)
}

// Reads from a give location
uint16_t ADS1015::readRegister(uint8_t location)
{
  _i2cPort->beginTransmission(_i2caddr);
  _i2cPort->write(location);
  bool success = _i2cPort->endTransmission() == 0;
  if (success)
    success &= _i2cPort->requestFrom(_i2caddr, (uint8_t)2) == 2; // Ask for two bytes
  if (success)
    return ((((uint16_t)_i2cPort->read()) << 8) | _i2cPort->read()); // MSB first

  return 0; // Return zero to indicate an error. Not great, but something...
}

// Write a value to a spot
bool ADS1015::writeRegister(uint8_t location, uint16_t val)
{
  _i2cPort->beginTransmission(_i2caddr);
  _i2cPort->write(location);
  _i2cPort->write((uint8_t)(val >> 8)); // MSB first
  _i2cPort->write((uint8_t)(val & 0xFF));
  return (_i2cPort->endTransmission() == 0);
}

// Reads a two byte value from a consecutive registers
uint16_t ADS1015::readRegister16(uint8_t location)
{
  return (readRegister(location));
}

/**************************************************************************/
/*!
    @brief  Sets up the comparator to operate in basic mode, causing the
            ALERT/RDY pin to assert (go from high to low) when the ADC
            value exceeds the specified threshold.

            This will also set the ADC in continuous conversion mode.

      Note, this function was adapted from the Adafruit Industries
      located here:
      https://github.com/adafruit/Adafruit_ADS1X15
*/
/**************************************************************************/
void ADS1015::setComparatorSingleEnded(uint8_t channel, int16_t threshold)
{
  if (channel > 3)
  {
    return;
  }

  uint16_t config =
      ADS1015_CONFIG_MODE_CONT |
      _sampleRate |
      ADS1015_CONFIG_CQUE_1CONV |   // Comparator enabled and asserts on 1 match
      ADS1015_CONFIG_CLAT_LATCH |   // Latching mode
      ADS1015_CONFIG_CPOL_ACTVLOW | // Alert/Rdy active low   (default val)
      ADS1015_CONFIG_CMODE_TRAD;    // Traditional comparator (default val)

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

  // Set the high threshold register
  // Shift 12-bit results left 4 bits for the ADS1015
  writeRegister(ADS1015_POINTER_HITHRESH, threshold << 4);

  // Write config register to the ADC
  writeRegister(ADS1015_POINTER_CONFIG, config);
}

/**************************************************************************/
/*!
    @brief  In order to clear the comparator, we need to read the
            conversion results.  This function reads the last conversion
            results without changing the config value.

      Note, this function was adapted from the Adafruit Industries
      located here:
      https://github.com/adafruit/Adafruit_ADS1X15
*/
/**************************************************************************/
int16_t ADS1015::getLastConversionResults()
{
  // Wait for the conversion to complete
  // conversionDelay(); // Why wait??

  // Read the conversion results
  // Shift 12-bit results right 4 bits for the ADS1015,
  // making sure we keep the sign bit intact
  uint16_t result = readRegister(ADS1015_POINTER_CONVERT) >> 4;
  if (result > 0x07FF)
  {
    // negative number - extend the sign to 16th bit
    result |= 0xF000;
  }
  return (convertUnsignedToSigned(result));
}

// Convert uint16_t to int16_t without cast ambiguity
int16_t ADS1015::convertUnsignedToSigned(uint16_t unsigned16)
{
  union
  {
    int16_t signed16;
    uint16_t unsigned16;
  } signedUnsigned16;
  signedUnsigned16.unsigned16 = unsigned16;
  return signedUnsigned16.signed16;
}

// These values were determined experimentally using Example7
void ADS1015::conversionDelay()
{
  if (_sampleRate >= ADS1015_CONFIG_RATE_3300HZ)
    delayMicroseconds(400); // > (303us + 10% + 25us power-up)
  else if (_sampleRate >= ADS1015_CONFIG_RATE_2400HZ)
    delayMicroseconds(500); // > (417us + 10% + 25us power-up)
  else if (_sampleRate >= ADS1015_CONFIG_RATE_1600HZ)
    delay(1); // > (625us + 10% + 25us power-up)
  else if (_sampleRate >= ADS1015_CONFIG_RATE_920HZ)
    delay(2);
  else if (_sampleRate >= ADS1015_CONFIG_RATE_490HZ)
    delay(4);
  else if (_sampleRate >= ADS1015_CONFIG_RATE_250HZ)
    delay(8);
  else
    delay(16); // 128Hz
}