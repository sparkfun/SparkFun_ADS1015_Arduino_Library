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

#pragma once

#if (ARDUINO >= 100)
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

#define ADS1015_DEFAULT_ADDRESS 0x48 //7-bit unshifted default I2C Address

#define I2C_SPEED_STANDARD        100000
#define I2C_SPEED_FAST            400000

//Register addresses
#define ADS1015_DELAY                (1)

//Pointer Register
#define ADS1015_POINTER_CONVERT      (0x00)
#define ADS1015_POINTER_CONFIG       (0x01)

#define ADS1015_CONFIG_OS_NO         (0x8000)
#define ADS1015_CONFIG_OS_SINGLE     (0x8000)
#define ADS1015_CONFIG_OS_READY      (0x0000)
#define ADS1015_CONFIG_OS_NOTREADY   (0x8000)

#define ADS1015_CONFIG_MODE_CONT     (0x0000)
#define ADS1015_CONFIG_MODE_SINGLE   (0x0100)

#define ADS1015_CONFIG_MUX_SINGLE_0  (0x4000)
#define ADS1015_CONFIG_MUX_SINGLE_1  (0x5000)
#define ADS1015_CONFIG_MUX_SINGLE_2  (0x6000)
#define ADS1015_CONFIG_MUX_SINGLE_3  (0x7000)

#define ADS1015_CONFIG_RATE_128HZ    (0x0000)
#define ADS1015_CONFIG_RATE_250HZ    (0x0020)
#define ADS1015_CONFIG_RATE_490HZ    (0x0040)
#define ADS1015_CONFIG_RATE_920HZ    (0x0060)
#define ADS1015_CONFIG_RATE_1600HZ   (0x0080)
#define ADS1015_CONFIG_RATE_2400HZ   (0x00A0)
#define ADS1015_CONFIG_RATE_3300HZ   (0x00C0)

#define ADS1015_CONFIG_PGA_MASK      (0X0E00)
#define ADS1015_CONFIG_PGA_2/3       (0X0000)  // +/- 6.144v
#define ADS1015_CONFIG_PGA_1         (0X0200)  // +/- 4.096v
#define ADS1015_CONFIG_PGA_2         (0X0400)  // +/- 2.048v
#define ADS1015_CONFIG_PGA_4         (0X0600)  // +/- 1.024v
#define ADS1015_CONFIG_PGA_8         (0X0800)  // +/- 0.512v
#define ADS1015_CONFIG_PGA_16        (0X0a00)  // +/- 0.256v

class ADS1015 {
  public:
    //By default use Wire, standard I2C speed, and the default ADS1015 address
    boolean begin(TwoWire &wirePort = Wire, uint32_t i2cSpeed = I2C_SPEED_STANDARD, uint8_t i2caddr = ADS1015_DEFAULT_ADDRESS, long baud = 9600);

    int16_t getAnalogData(uint8_t channel);

    //boolean available(); //True if OS bit is set
    //void softReset(); //Resets the IC via software

    //void setMode(uint8_t mode = ADS1015_CONFIG_MODE_CONT); //Set mode of the sensor. Mode 0 is continuous read mode

    //void enableDebugging(Stream &debugPort = Serial); //Output various extra messages to help with debug
    //void disableDebugging();

    uint16_t readRegister(uint8_t location); //Basic read of a register
    void writeRegister(uint8_t location, uint16_t val); //Writes to a location
    uint16_t readRegister16(byte location); //Reads a 16bit value

    //Variables

  private:
    //uint8_t printI2CError(uint8_t errorCode); //Prints endTransmission statuses

    TwoWire *_i2cPort; //The generic connection to user's chosen I2C hardware
    uint8_t _i2caddr;

    boolean _printDebug = false; //Flag to print the serial commands we are sending to the Serial port for debug

    Stream *_debugSerial; //The stream to send debug messages to if enabled
};


