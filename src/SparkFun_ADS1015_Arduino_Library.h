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

#pragma once

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
// Teensy
#include "i2c_t3.h"

#else
#include "Wire.h"

// The catch-all default is 32
#ifndef I2C_BUFFER_LENGTH
#define I2C_BUFFER_LENGTH 32
#endif

#endif

#define ADS1015_ADDRESS_GND 0x48 // 7-bit unshifted default I2C Address
#define ADS1015_ADDRESS_VDD 0x49
#define ADS1015_ADDRESS_SDA 0x4A
#define ADS1015_ADDRESS_SCL 0x4B

// Register addresses
#define ADS1015_DELAY (1)

// Pointer Register
#define ADS1015_POINTER_CONVERT (0x00)
#define ADS1015_POINTER_CONFIG (0x01)
#define ADS1015_POINTER_LOWTHRESH (0x02)
#define ADS1015_POINTER_HITHRESH (0x03)

// Config Register

// Operational status or single-shot conversion start
// This bit determines the operational status of the device. OS can only be written
// when in power-down state and has no effect when a conversion is ongoing.
#define ADS1015_CONFIG_OS_NO (0x0000)
#define ADS1015_CONFIG_OS_SINGLE (0x8000)	// 1 : Start a single conversion (when in power-down state)
#define ADS1015_CONFIG_OS_NOTREADY (0x0000) // 0 : Device is currently performing a conversion
#define ADS1015_CONFIG_OS_READY (0x8000)	// 1 : Device is not currently performing a conversion

#define ADS1015_CONFIG_MODE_CONT (0x0000)
#define ADS1015_CONFIG_MODE_SINGLE (0x0100)

#define ADS1015_CONFIG_MUX_SINGLE_0 (0x4000)
#define ADS1015_CONFIG_MUX_SINGLE_1 (0x5000)
#define ADS1015_CONFIG_MUX_SINGLE_2 (0x6000)
#define ADS1015_CONFIG_MUX_SINGLE_3 (0x7000)
#define ADS1015_CONFIG_MUX_DIFF_P0_N1 (0x0000)
#define ADS1015_CONFIG_MUX_DIFF_P0_N3 (0x1000)
#define ADS1015_CONFIG_MUX_DIFF_P1_N3 (0x2000)
#define ADS1015_CONFIG_MUX_DIFF_P2_N3 (0x3000)

#define ADS1015_CONFIG_RATE_MASK (0x00E0)
#define ADS1015_CONFIG_RATE_128HZ (0x0000)
#define ADS1015_CONFIG_RATE_250HZ (0x0020)
#define ADS1015_CONFIG_RATE_490HZ (0x0040)
#define ADS1015_CONFIG_RATE_920HZ (0x0060)
#define ADS1015_CONFIG_RATE_1600HZ (0x0080)
#define ADS1015_CONFIG_RATE_2400HZ (0x00A0)
#define ADS1015_CONFIG_RATE_3300HZ (0x00C0)

#define ADS1015_CONFIG_PGA_MASK (0X0E00)
#define ADS1015_CONFIG_PGA_TWOTHIRDS (0X0000) // +/- 6.144v
#define ADS1015_CONFIG_PGA_1 (0X0200)		  // +/- 4.096v
#define ADS1015_CONFIG_PGA_2 (0X0400)		  // +/- 2.048v
#define ADS1015_CONFIG_PGA_4 (0X0600)		  // +/- 1.024v
#define ADS1015_CONFIG_PGA_8 (0X0800)		  // +/- 0.512v
#define ADS1015_CONFIG_PGA_16 (0X0A00)		  // +/- 0.256v

#define ADS1015_CONFIG_CMODE_TRAD (0x0000)	 // Traditional comparator with hysteresis (default)
#define ADS1015_CONFIG_CMODE_WINDOW (0x0010) // Window comparator
#define ADS1015_CONFIG_CPOL_ACTVLOW (0x0000) // ALERT/RDY pin is low when active (default)
#define ADS1015_CONFIG_CPOL_ACTVHI (0x0008)	 // ALERT/RDY pin is high when active
#define ADS1015_CONFIG_CLAT_NONLAT (0x0000)	 // Non-latching comparator (default)
#define ADS1015_CONFIG_CLAT_LATCH (0x0004)	 // Latching comparator
#define ADS1015_CONFIG_CQUE_1CONV (0x0000)	 // Assert ALERT/RDY after one conversions
#define ADS1015_CONFIG_CQUE_2CONV (0x0001)	 // Assert ALERT/RDY after two conversions
#define ADS1015_CONFIG_CQUE_4CONV (0x0002)	 // Assert ALERT/RDY after four conversions
#define ADS1015_CONFIG_CQUE_NONE (0x0003)	 // Disable the comparator and put ALERT/RDY in high state (default)

class ADS1015
{
public:
// By default use Wire, standard I2C speed, and the default ADS1015 address
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
	// Teensy
	bool begin(uint8_t i2caddr = ADS1015_ADDRESS_GND, i2c_t3 &wirePort = Wire);

#else

	bool begin(uint8_t i2caddr = ADS1015_ADDRESS_GND, TwoWire &wirePort = Wire);

#endif
	bool isConnected(); // Checks if sensor ack's the I2C request

	uint16_t getSingleEnded(uint8_t channel);
	int16_t getSingleEndedSigned(uint8_t channel);
	float getSingleEndedMillivolts(uint8_t channel);
	int16_t getDifferential(uint16_t CONFIG_MUX_DIFF = ADS1015_CONFIG_MUX_DIFF_P0_N1);
	float getDifferentialMillivolts(uint16_t CONFIG_MUX_DIFF = ADS1015_CONFIG_MUX_DIFF_P0_N1);
	uint16_t getAnalogData(uint8_t channel); // antiquated function; here for backward compatibility
	float getScaledAnalogData(uint8_t channel);
	void calibrate();
	uint16_t getCalibration(uint8_t channel, bool hiLo);
	void setCalibration(uint8_t channel, bool hiLo, uint16_t value);
	void resetCalibration();

	float mapf(float val, float in_min, float in_max, float out_min, float out_max);

	bool available(); // True if OS bit is set

	void setMode(uint16_t mode); // Set mode of the sensor. Mode 0 is continuous read mode
	uint16_t getMode();

	void setGain(uint16_t gain);
	uint16_t getGain();

	void setSampleRate(uint16_t sampleRate);
	uint16_t getSampleRate();

	float getMultiplier();

	uint16_t readRegister(uint8_t location);			// Basic read of a register
	bool writeRegister(uint8_t location, uint16_t val); // Writes to a location
	uint16_t readRegister16(uint8_t location);			// Reads a 16bit value

	void setComparatorSingleEnded(uint8_t channel, int16_t threshold);
	int16_t getLastConversionResults();

	int16_t convertUnsignedToSigned(uint16_t unsigned16); // Convert uint16_t to int16_t without cast ambiguity

	void conversionDelay(); // Delay for the conversion time (as defined by _sampleRate)

	// When useConversionReady is enabled:
	//   The Config Register OS bit is read to determine when the conversion is complete - instead of using conversionDelay.
	//   Single-shot mode is always selected. _mode is ignored.
	// Defaults to false for backward-compatibility.
	void useConversionReady(bool enable) { _useConversionReady = enable; }

private:
#if defined(__MK64FX512__) || defined(__MK66FX1M0__)
	// Teensy
	i2c_t3 *_i2cPort;

#else

	TwoWire *_i2cPort;

#endif

	uint16_t _mode = ADS1015_CONFIG_MODE_CONT; // Default to continuous mode
	bool _useConversionReady = false; // Default to disabled, allowing continuous mode to be used.
	uint16_t _gain = ADS1015_CONFIG_PGA_2;
	uint16_t _sampleRate = ADS1015_CONFIG_RATE_1600HZ;
	float _multiplierToVolts = 1.0F; // at a default gain of 2, the multiplier is 1, also updated in setGain()
	void updateMultiplierToVolts();

	uint8_t _i2caddr;

	// Array is structured as calibrationValues[finger][lo/hi]
	uint16_t calibrationValues[2][2] = {{0, 0}, {0, 0}};

	bool _printDebug = false; // Flag to print the serial commands we are sending to the Serial port for debug

	Stream *_debugSerial; // The stream to send debug messages to if enabled
};
