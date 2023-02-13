//
//    FILE: main.ino
//  AUTHOR: Janaka
// VERSION: 0.01
// PURPOSE: read multiple analog inputs continuously interrupt 
//          driven to catch all conversions and send the value 
//          via the communication protocol.
//
// LIBRARY: This script uses a predifine arduino library for
//          extracting the the external ADC value. Following are 
//          the library details.
//             AUTHOR: Rob Tillaart
//            VERSION: 0.3.9
//               DATE: 2013-03-24
//            PUPROSE: Arduino library for ADS1015 and ADS1115
//                URL: https://github.com/RobTillaart/ADS1X15
//          

//
#include <stdio.h>
#include <string.h>
//#include <EEPROM.h>

///////////...........LIBRARY FILE...........................////////////////

//////// header file////////////////////////////////////////////////////////
#include "Arduino.h"
#include "Wire.h"

#define ADS1X15_LIB_VERSION               (F("0.3.9"))

//  allow compile time default address
//  address in { 0x48, 0x49, 0x4A, 0x4B }, no test...
#ifndef ADS1015_ADDRESS
#define ADS1015_ADDRESS                   0x48
#endif

#ifndef ADS1115_ADDRESS
#define ADS1115_ADDRESS                   0x48
#endif


#define ADS1X15_OK                        0
#define ADS1X15_INVALID_VOLTAGE           -100
#define ADS1X15_INVALID_GAIN              0xFF
#define ADS1X15_INVALID_MODE              0xFE


class ADS1X15
{
public:
  void     reset();

#if defined (ESP8266) || defined(ESP32)
  bool     begin(int sda, int scl);
#endif

#if defined (ARDUINO_ARCH_RP2040)
  bool    begin(int sda, int scl);
#endif

  bool     begin();
  bool     isConnected();

  //           GAIN
  //  0  =  +- 6.144V  default
  //  1  =  +- 4.096V
  //  2  =  +- 2.048V
  //  4  =  +- 1.024V
  //  8  =  +- 0.512V
  //  16 =  +- 0.256V
  void     setGain(uint8_t gain = 0);    //  invalid values are mapped to 0 (default).
  uint8_t  getGain();                    //  0xFF == invalid gain error.


  //  both may return ADS1X15_INVALID_VOLTAGE if the gain is invalid.
  float    toVoltage(int16_t value = 1); //   converts raw to voltage
  float    getMaxVoltage();              //   -100 == invalid voltage error


  //  0  =  CONTINUOUS
  //  1  =  SINGLE      default
  void     setMode(uint8_t mode = 1);    //  invalid values are mapped to 1 (default)
  uint8_t  getMode();                    //  0xFE == invalid mode error.


  //  0  =  slowest
  //  7  =  fastest
  //  4  =  default
  void     setDataRate(uint8_t dataRate = 4); // invalid values are mapped on 4 (default)
  uint8_t  getDataRate();                     // actual speed depends on device


  int16_t  readADC(uint8_t pin = 0);
  int16_t  readADC_Differential_0_1();

  //  used by continuous mode and async mode.
  int16_t  getLastValue() { return getValue(); };  // will be obsolete in the future 0.4.0
  int16_t  getValue();


  //  ASYNC INTERFACE
  //  requestADC(pin) -> isBusy() or isReady() -> getValue();
  //  see examples
  void     requestADC(uint8_t pin = 0);
  void     requestADC_Differential_0_1();
  bool     isBusy();
  bool     isReady();


  //  COMPARATOR
  //  0    = TRADITIONAL   > high          => on      < low   => off
  //  else = WINDOW        > high or < low => on      between => off
  void     setComparatorMode(uint8_t mode) { _compMode = mode == 0 ? 0 : 1; };
  uint8_t  getComparatorMode()             { return _compMode; };

  //  0    = LOW (default)
  //  else = HIGH
  void     setComparatorPolarity(uint8_t pol) { _compPol = pol ? 0 : 1; };
  uint8_t  getComparatorPolarity()            { return _compPol; };

  //  0    = NON LATCH
  //  else = LATCH
  void     setComparatorLatch(uint8_t latch) { _compLatch = latch ? 0 : 1; };
  uint8_t  getComparatorLatch()              { return _compLatch; };

  //  0   = trigger alert after 1 conversion
  //  1   = trigger alert after 2 conversions
  //  2   = trigger alert after 4 conversions
  //  3   = Disable comparator =  default, also for all other values.
  void     setComparatorQueConvert(uint8_t mode) { _compQueConvert = (mode < 3) ? mode : 3; };
  uint8_t  getComparatorQueConvert()             { return _compQueConvert; };

  void     setComparatorThresholdLow(int16_t lo);
  int16_t  getComparatorThresholdLow();
  void     setComparatorThresholdHigh(int16_t hi);
  int16_t  getComparatorThresholdHigh();


  int8_t   getError();

  //  EXPERIMENTAL
  //  see https://github.com/RobTillaart/ADS1X15/issues/22
  void     setWireClock(uint32_t clockSpeed = 100000);
  //  proto - getWireClock returns the value set by setWireClock
  //  not necessary the actual value
  uint32_t getWireClock();

protected:
  ADS1X15();

  //  CONFIGURATION
  //  BIT   DESCRIPTION
  //  0     # channels        0 == 1    1 == 4;
  //  1     0
  //  2     # resolution      0 == 12   1 == 16
  //  3     0
  //  4     has gain          0 = NO    1 = YES
  //  5     has comparator    0 = NO    1 = YES
  //  6     0
  //  7     0
  uint8_t  _config;
  uint8_t  _maxPorts;
  uint8_t  _address;
  uint8_t  _conversionDelay;
  uint8_t  _bitShift;
  uint16_t _gain;
  uint16_t _mode;
  uint16_t _datarate;

  //  COMPARATOR variables
  //  TODO merge these into one COMPARATOR MASK?  (low priority)
  //       would speed up code in _requestADC() and save 3 bytes RAM.
  //  TODO boolean flags for first three, or make it mask value that
  //       can be or-ed.   (low priority)
  uint8_t  _compMode;
  uint8_t  _compPol;
  uint8_t  _compLatch;
  uint8_t  _compQueConvert;

  int16_t  _readADC(uint16_t readmode);
  void     _requestADC(uint16_t readmode);
  bool     _writeRegister(uint8_t address, uint8_t reg, uint16_t value);
  uint16_t _readRegister(uint8_t address, uint8_t reg);
  int8_t   _err = ADS1X15_OK;

  TwoWire*  _wire;
  uint32_t  _clockSpeed = 0;
};


///////////////////////////////////////////////////////////////////////////
//
//  DERIVED CLASSES from ADS1X15
//
class ADS1013 : public ADS1X15
{
public:
  ADS1013(uint8_t Address = ADS1015_ADDRESS, TwoWire *wire = &Wire);
};


class ADS1014 : public ADS1X15
{
public:
  ADS1014(uint8_t Address = ADS1015_ADDRESS, TwoWire *wire = &Wire);
};


class ADS1015 : public ADS1X15
{
public:
  ADS1015(uint8_t Address = ADS1015_ADDRESS, TwoWire *wire = &Wire);
  int16_t  readADC_Differential_0_3();
  int16_t  readADC_Differential_1_3();
  int16_t  readADC_Differential_2_3();
  int16_t  readADC_Differential_0_2();   //  not possible in async
  int16_t  readADC_Differential_1_2();   //  not possible in async
  void     requestADC_Differential_0_3();
  void     requestADC_Differential_1_3();
  void     requestADC_Differential_2_3();
};


class ADS1113 : public ADS1X15
{
public:
  ADS1113(uint8_t address = ADS1115_ADDRESS, TwoWire *wire = &Wire);
};


class ADS1114 : public ADS1X15
{
public:
  ADS1114(uint8_t address = ADS1115_ADDRESS, TwoWire *wire = &Wire);
};


class ADS1115 : public ADS1X15
{
public:
  ADS1115(uint8_t address = ADS1115_ADDRESS, TwoWire *wire = &Wire);
  int16_t  readADC_Differential_0_3();
  int16_t  readADC_Differential_1_3();
  int16_t  readADC_Differential_2_3();
  int16_t  readADC_Differential_0_2();   //  not possible in async
  int16_t  readADC_Differential_1_2();   //  not possible in async
  void     requestADC_Differential_0_3();
  void     requestADC_Differential_1_3();
  void     requestADC_Differential_2_3();
};


//  -- END OF FILE --

////////.cpp file///////////////////////////////////////////////////////////
#define ADS1015_CONVERSION_DELAY    1
#define ADS1115_CONVERSION_DELAY    8


//  Kept #defines a bit in line with Adafruit library.

//  REGISTERS
#define ADS1X15_REG_CONVERT         0x00
#define ADS1X15_REG_CONFIG          0x01
#define ADS1X15_REG_LOW_THRESHOLD   0x02
#define ADS1X15_REG_HIGH_THRESHOLD  0x03


//  CONFIG REGISTER

//  BIT 15      Operational Status           // 1 << 15
#define ADS1X15_OS_BUSY             0x0000
#define ADS1X15_OS_NOT_BUSY         0x8000
#define ADS1X15_OS_START_SINGLE     0x8000

//  BIT 12-14   read differential
#define ADS1X15_MUX_DIFF_0_1        0x0000
#define ADS1X15_MUX_DIFF_0_3        0x1000
#define ADS1X15_MUX_DIFF_1_3        0x2000
#define ADS1X15_MUX_DIFF_2_3        0x3000
//              read single
#define ADS1X15_READ_0              0x4000   //  pin << 12
#define ADS1X15_READ_1              0x5000   //  pin = 0..3
#define ADS1X15_READ_2              0x6000
#define ADS1X15_READ_3              0x7000


//  BIT 9-11    gain                         //  (0..5) << 9
#define ADS1X15_PGA_6_144V          0x0000   //  voltage
#define ADS1X15_PGA_4_096V          0x0200   //
#define ADS1X15_PGA_2_048V          0x0400   //  default
#define ADS1X15_PGA_1_024V          0x0600
#define ADS1X15_PGA_0_512V          0x0800
#define ADS1X15_PGA_0_256V          0x0A00

//  BIT 8       mode                         //  1 << 8
#define ADS1X15_MODE_CONTINUE       0x0000
#define ADS1X15_MODE_SINGLE         0x0100

//  BIT 5-7     data rate sample per second  // (0..7) << 5
/*
differs for different devices, check datasheet or readme.md

|  data rate  |  ADS101x  |  ADS111x  |   Notes   |
|:-----------:|----------:|----------:|:---------:|
|     0       |   128     |    8      |  slowest  |
|     1       |   250     |    16     |           |
|     2       |   490     |    32     |           |
|     3       |   920     |    64     |           |
|     4       |   1600    |    128    |  default  |
|     5       |   2400    |    250    |           |
|     6       |   3300    |    475    |           |
|     7       |   3300    |    860    |  fastest  |
*/

//  BIT 4 comparator modi                    // 1 << 4
#define ADS1X15_COMP_MODE_TRADITIONAL   0x0000
#define ADS1X15_COMP_MODE_WINDOW        0x0010

//  BIT 3 ALERT active value                 // 1 << 3
#define ADS1X15_COMP_POL_ACTIV_LOW      0x0000
#define ADS1X15_COMP_POL_ACTIV_HIGH     0x0008

//  BIT 2 ALERT latching                     // 1 << 2
#define ADS1X15_COMP_NON_LATCH          0x0000
#define ADS1X15_COMP_LATCH              0x0004

//  BIT 0-1 ALERT mode                       // (0..3)
#define ADS1X15_COMP_QUE_1_CONV         0x0000  //  trigger alert after 1 convert
#define ADS1X15_COMP_QUE_2_CONV         0x0001  //  trigger alert after 2 converts
#define ADS1X15_COMP_QUE_4_CONV         0x0002  //  trigger alert after 4 converts
#define ADS1X15_COMP_QUE_NONE           0x0003  //  disable comparator


// _CONFIG masks
//
//  |  bit  |  description           |
//  |:-----:|:-----------------------|
//  |   0   |  # channels            |
//  |   1   |  -                     |
//  |   2   |  resolution            |
//  |   3   |  -                     |
//  |   4   |  GAIN supported        |
//  |   5   |  COMPARATOR supported  |
//  |   6   |  -                     |
//  |   7   |  -                     |
//
#define ADS_CONF_CHAN_1  0x00
#define ADS_CONF_CHAN_4  0x01
#define ADS_CONF_RES_12  0x00
#define ADS_CONF_RES_16  0x04
#define ADS_CONF_NOGAIN  0x00
#define ADS_CONF_GAIN    0x10
#define ADS_CONF_NOCOMP  0x00
#define ADS_CONF_COMP    0x20


//////////////////////////////////////////////////////
//
//  BASE CONSTRUCTOR
//
ADS1X15::ADS1X15()
{
  reset();
}


//////////////////////////////////////////////////////
//
//  PUBLIC
//
void ADS1X15::reset()
{
  setGain(0);      //  _gain = ADS1X15_PGA_6_144V;
  setMode(1);      //  _mode = ADS1X15_MODE_SINGLE;
  setDataRate(4);  //  middle speed, depends on device.

  //  COMPARATOR variables   # see notes .h
  _compMode       = 0;
  _compPol        = 1;
  _compLatch      = 0;
  _compQueConvert = 3;
}


#if defined (ESP8266) || defined(ESP32)
bool ADS1X15::begin(int sda, int scl)
{
  _wire = &Wire;
  _wire->begin(sda, scl);
  if ((_address < 0x48) || (_address > 0x4B)) return false;
  if (! isConnected()) return false;
  return true;
}
#endif

#if defined (ARDUINO_ARCH_RP2040)

bool ADS1X15::begin(int sda, int scl)
{
  _wire->setSDA(sda);
  _wire->setSCL(scl);
  _wire->begin();
  if ((_address < 0x48) || (_address > 0x4B)) return false;
  if (! isConnected()) return false;
  return true;
}

#endif

bool ADS1X15::begin()
{
  _wire->begin();
  if ((_address < 0x48) || (_address > 0x4B)) return false;
  if (! isConnected()) return false;
  return true;
}


bool ADS1X15::isBusy()
{
  return isReady() == false;
}


bool ADS1X15::isReady()
{
  uint16_t val = _readRegister(_address, ADS1X15_REG_CONFIG);
  return ((val & ADS1X15_OS_NOT_BUSY) > 0);
}


bool ADS1X15::isConnected()
{
  _wire->beginTransmission(_address);
  return (_wire->endTransmission() == 0);
}


void ADS1X15::setGain(uint8_t gain)
{
  if (!(_config & ADS_CONF_GAIN)) gain = 0;
  switch (gain)
  {
    default:  //  catch invalid values and go for the safest gain.
    case 0:  _gain = ADS1X15_PGA_6_144V;  break;
    case 1:  _gain = ADS1X15_PGA_4_096V;  break;
    case 2:  _gain = ADS1X15_PGA_2_048V;  break;
    case 4:  _gain = ADS1X15_PGA_1_024V;  break;
    case 8:  _gain = ADS1X15_PGA_0_512V;  break;
    case 16: _gain = ADS1X15_PGA_0_256V;  break;
  }
}


uint8_t ADS1X15::getGain()
{
  if (!(_config & ADS_CONF_GAIN)) return 0;
  switch (_gain)
  {
    case ADS1X15_PGA_6_144V: return 0;
    case ADS1X15_PGA_4_096V: return 1;
    case ADS1X15_PGA_2_048V: return 2;
    case ADS1X15_PGA_1_024V: return 4;
    case ADS1X15_PGA_0_512V: return 8;
    case ADS1X15_PGA_0_256V: return 16;
  }
  _err = ADS1X15_INVALID_GAIN;
  return _err;
}


float ADS1X15::toVoltage(int16_t value)
{
  if (value == 0) return 0;

  float volts = getMaxVoltage();
  if (volts < 0) return volts;

  volts *= value;
  if (_config & ADS_CONF_RES_16)
  {
    volts /= 32767;  //  value = 16 bits - sign bit = 15 bits mantissa
  }
  else
  {
    volts /= 2047;   //  value = 12 bits - sign bit = 11 bit mantissa
  }
  return volts;
}


float ADS1X15::getMaxVoltage()
{
  switch (_gain)
  {
    case ADS1X15_PGA_6_144V: return 6.144;
    case ADS1X15_PGA_4_096V: return 4.096;
    case ADS1X15_PGA_2_048V: return 2.048;
    case ADS1X15_PGA_1_024V: return 1.024;
    case ADS1X15_PGA_0_512V: return 0.512;
    case ADS1X15_PGA_0_256V: return 0.256;
  }
  _err = ADS1X15_INVALID_VOLTAGE;
  return _err;
}


void ADS1X15::setMode(uint8_t mode)
{
  switch (mode)
  {
    case 0: _mode = ADS1X15_MODE_CONTINUE; break;
    default:
    case 1: _mode = ADS1X15_MODE_SINGLE;   break;
  }
}


uint8_t ADS1X15::getMode(void)
{
  switch (_mode)
  {
    case ADS1X15_MODE_CONTINUE: return 0;
    case ADS1X15_MODE_SINGLE:   return 1;
  }
  _err = ADS1X15_INVALID_MODE;
  return _err;
}


void ADS1X15::setDataRate(uint8_t dataRate)
{
  _datarate = dataRate;
  if (_datarate > 7) _datarate = 4;  //  default
  _datarate <<= 5;      //  convert 0..7 to mask needed.
}


uint8_t ADS1X15::getDataRate(void)
{
  return (_datarate >> 5) & 0x07;  //  convert mask back to 0..7
}


int16_t ADS1X15::readADC(uint8_t pin)
{
  if (pin >= _maxPorts) return 0;
  uint16_t mode = ((4 + pin) << 12);  //  pin to mask
  return _readADC(mode);
}


void  ADS1X15::requestADC_Differential_0_1()
{
  _requestADC(ADS1X15_MUX_DIFF_0_1);
}


int16_t ADS1X15::readADC_Differential_0_1()
{
  return _readADC(ADS1X15_MUX_DIFF_0_1);
}


void ADS1X15::requestADC(uint8_t pin)
{
  if (pin >= _maxPorts) return;
  uint16_t mode = ((4 + pin) << 12);   //  pin to mask
  _requestADC(mode);
}


int16_t ADS1X15::getValue()
{
  int16_t raw = _readRegister(_address, ADS1X15_REG_CONVERT);
  if (_bitShift) raw >>= _bitShift;  //  Shift 12-bit results
  return raw;
}


void ADS1X15::setComparatorThresholdLow(int16_t lo)
{
  _writeRegister(_address, ADS1X15_REG_LOW_THRESHOLD, lo);
};


int16_t ADS1X15::getComparatorThresholdLow()
{
  return _readRegister(_address, ADS1X15_REG_LOW_THRESHOLD);
};


void ADS1X15::setComparatorThresholdHigh(int16_t hi)
{
  _writeRegister(_address, ADS1X15_REG_HIGH_THRESHOLD, hi);
};


int16_t ADS1X15::getComparatorThresholdHigh()
{
  return _readRegister(_address, ADS1X15_REG_HIGH_THRESHOLD);
};


int8_t ADS1X15::getError()
{
  int8_t rv = _err;
  _err = ADS1X15_OK;
  return rv;
}


void ADS1X15::setWireClock(uint32_t clockSpeed)
{
  _clockSpeed = clockSpeed;
  _wire->setClock(_clockSpeed);
}


//////////////////////////////////////////////////////
//
//  EXPERIMENTAL
//
//  see https://github.com/RobTillaart/ADS1X15/issues/22
//      https://github.com/arduino/Arduino/issues/11457
//  TODO: get the real clock speed from the I2C interface if possible.
uint32_t ADS1X15::getWireClock()
{
// UNO 328 and
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
  uint32_t speed = F_CPU / ((TWBR * 2) + 16);
  return speed;

#elif defined(ESP32)
  return (uint32_t) _wire->getClock();

//  #elif defined(ESP8266)
//  core_esp8266_si2c.cpp holds the data see => void Twi::setClock(
//  not supported.
//  return -1;

#else  // best effort is remembering it
  return _clockSpeed;
#endif
}


//////////////////////////////////////////////////////
//
//  PROTECTED
//
int16_t ADS1X15::_readADC(uint16_t readmode)
{
  _requestADC(readmode);
  if (_mode == ADS1X15_MODE_SINGLE)
  {
    while ( isBusy() ) yield();   //  wait for conversion; yield for ESP.
  }
  else
  {
    delay(_conversionDelay);      //  TODO needed in continuous mode?
  }
  return getValue();
}


void ADS1X15::_requestADC(uint16_t readmode)
{
  //  write to register is needed in continuous mode as other flags can be changed
  uint16_t config = ADS1X15_OS_START_SINGLE;  //  bit 15     force wake up if needed
  config |= readmode;                         //  bit 12-14
  config |= _gain;                            //  bit 9-11
  config |= _mode;                            //  bit 8
  config |= _datarate;                        //  bit 5-7
  if (_compMode)  config |= ADS1X15_COMP_MODE_WINDOW;         //  bit 4      comparator modi
  else            config |= ADS1X15_COMP_MODE_TRADITIONAL;
  if (_compPol)   config |= ADS1X15_COMP_POL_ACTIV_HIGH;      //  bit 3      ALERT active value
  else            config |= ADS1X15_COMP_POL_ACTIV_LOW;
  if (_compLatch) config |= ADS1X15_COMP_LATCH;
  else            config |= ADS1X15_COMP_NON_LATCH;           //  bit 2      ALERT latching
  config |= _compQueConvert;                                  //  bit 0..1   ALERT mode
  _writeRegister(_address, ADS1X15_REG_CONFIG, config);
}


bool ADS1X15::_writeRegister(uint8_t address, uint8_t reg, uint16_t value)
{
  _wire->beginTransmission(address);
  _wire->write((uint8_t)reg);
  _wire->write((uint8_t)(value >> 8));
  _wire->write((uint8_t)(value & 0xFF));
  return (_wire->endTransmission() == 0);
}


uint16_t ADS1X15::_readRegister(uint8_t address, uint8_t reg)
{
  _wire->beginTransmission(address);
  _wire->write(reg);
  _wire->endTransmission();

  int rv = _wire->requestFrom((int) address, (int) 2);
  if (rv == 2)
  {
    uint16_t value = _wire->read() << 8;
    value += _wire->read();
    return value;
  }
  return 0x0000;
}



///////////////////////////////////////////////////////////////////////////
//
//  DERIVED CLASSES
//


///////////////////////////////////////////////////////////////////////////
//
//  ADS1013
//
ADS1013::ADS1013(uint8_t address, TwoWire *wire)
{
  _address = address;
  _wire = wire;
  _config = ADS_CONF_NOCOMP | ADS_CONF_NOGAIN | ADS_CONF_RES_12 | ADS_CONF_CHAN_1;
  _conversionDelay = ADS1015_CONVERSION_DELAY;
  _bitShift = 4;
  _maxPorts = 1;
}


///////////////////////////////////////////////////////////////////////////
//
//  ADS1014
//
ADS1014::ADS1014(uint8_t address, TwoWire *wire)
{
  _address = address;
  _wire = wire;
  _config = ADS_CONF_COMP | ADS_CONF_GAIN | ADS_CONF_RES_12 | ADS_CONF_CHAN_1;
  _conversionDelay = ADS1015_CONVERSION_DELAY;
  _bitShift = 4;
  _maxPorts = 1;
}


///////////////////////////////////////////////////////////////////////////
//
//  ADS1015
//
ADS1015::ADS1015(uint8_t address, TwoWire *wire)
{
  _address = address;
  _wire = wire;
  _config = ADS_CONF_COMP | ADS_CONF_GAIN | ADS_CONF_RES_12 | ADS_CONF_CHAN_4;
  _conversionDelay = ADS1015_CONVERSION_DELAY;
  _bitShift = 4;
  _maxPorts = 4;
}


int16_t ADS1015::readADC_Differential_0_3()
{
  return _readADC(ADS1X15_MUX_DIFF_0_3);
}


int16_t ADS1015::readADC_Differential_1_3()
{
  return _readADC(ADS1X15_MUX_DIFF_1_3);
}


int16_t ADS1015::readADC_Differential_2_3()
{
  return _readADC(ADS1X15_MUX_DIFF_2_3);
}


int16_t ADS1015::readADC_Differential_0_2()
{
  return readADC(2) - readADC(0);
}


int16_t ADS1015::readADC_Differential_1_2()
{
  return readADC(2) - readADC(1);;
}


void ADS1015::requestADC_Differential_0_3()
{
  _requestADC(ADS1X15_MUX_DIFF_0_3);
}


void ADS1015::requestADC_Differential_1_3()
{
  _requestADC(ADS1X15_MUX_DIFF_1_3);
}


void ADS1015::requestADC_Differential_2_3()
{
  _requestADC(ADS1X15_MUX_DIFF_2_3);
}


///////////////////////////////////////////////////////////////////////////
//
//  ADS1113
//
ADS1113::ADS1113(uint8_t address, TwoWire *wire)
{
  _address = address;
  _wire = wire;
  _config = ADS_CONF_NOCOMP | ADS_CONF_NOGAIN | ADS_CONF_RES_16 | ADS_CONF_CHAN_1;
  _conversionDelay = ADS1115_CONVERSION_DELAY;
  _bitShift = 0;
  _maxPorts = 1;
}


///////////////////////////////////////////////////////////////////////////
//
//  ADS1114
//
ADS1114::ADS1114(uint8_t address, TwoWire *wire)
{
  _address = address;
  _wire = wire;
  _config = ADS_CONF_COMP | ADS_CONF_GAIN | ADS_CONF_RES_16 | ADS_CONF_CHAN_1;
  _conversionDelay = ADS1115_CONVERSION_DELAY;
  _bitShift = 0;
  _maxPorts = 1;
}


///////////////////////////////////////////////////////////////////////////
//
//  ADS1115
//
ADS1115::ADS1115(uint8_t address, TwoWire *wire)
{
  _address = address;
  _wire = wire;
  _config = ADS_CONF_COMP | ADS_CONF_GAIN | ADS_CONF_RES_16 | ADS_CONF_CHAN_4;
  _conversionDelay = ADS1115_CONVERSION_DELAY;
  _bitShift = 0;
  _maxPorts = 4;
}


int16_t ADS1115::readADC_Differential_0_3()
{
  return _readADC(ADS1X15_MUX_DIFF_0_3);
}
//////////.................END OF LIBRARY...................////////////////








typedef struct
{
  uint8_t mode;
  uint16_t reg;
  char data[30];
  bool dataAvailable;
}cmdDet_t;

typedef struct
{
  uint16_t reg;
  uint8_t mode;
  char (*cmd_fn)(uint8_t mode,char *str);
}cmd_t;

#define REG_SOFTWARE_VER            0x0001
#define REG_HARDWARE_VER            0x0002
#define REG_SERIAL_NUMBER           0x0003

#define REG_INPUT_VOLTAGE           0x0020
#define REG_OUTPUT_VOLTAGE          0x0021

#define REG_ALL_ADC_VALUE           0x0030
#define REG_CH1_ADC_VALUE           0x0031
#define REG_CH2_ADC_VALUE           0x0032
#define REG_CH3_ADC_VALUE           0x0033
#define REG_CH4_ADC_VALUE           0x0034
#define REG_CH5_ADC_VALUE           0x0035
#define REG_CH6_ADC_VALUE           0x0036
#define REG_CH7_ADC_VALUE           0x0037
#define REG_CH8_ADC_VALUE           0x0038
#define REG_CH1_TO_CH6_ADC_VALUE    0x003A

#define REG_END                     0xFFFF

#define arrLen(arr) ((int)(sizeof(arr)/sizeof(arr)[0]))
#define lowerCaseToUpperCase(charactor) ((char)(((int)(charactor)) - 20))
#define upperCaseToLowerCase(charactor) ((char)(((int)(charactor)) + 20))

#define REG_EXECUTABLE      0x10
#define REG_READABLE        0x11
#define REG_WRITABLE        0x12

///////.............combined register mode...................//////
/* combined register modes created by adding individual register modes
 * ex: if 0x00FF reg is capable of read and wire the combined reg mode value (RRW_) = (REG_READABLE + REG_WRITABLE)
 *
 * (RXXX) is the short form of reg combined mode and XXX denote Read capability, write capability, and execute capability
 * "_" denote null capability
 */
#define RRWE  (REG_EXECUTABLE + REG_READABLE + REG_WRITABLE)
#define RRW_  (REG_READABLE + REG_WRITABLE)
#define RR_E  (REG_READABLE + REG_EXECUTABLE)
#define R_WE  (REG_WRITABLE + REG_EXECUTABLE)
#define RR__  (REG_READABLE)
#define R_W_  (REG_WRITABLE)
#define R__E  (REG_EXECUTABLE)

#define BUFFER_SIZE       256
#define MILI_TO_SECOND    1000
#define SECOND_TO_MINUTE  60
#define SAMPLE_RATE       10
#define MIN_CMD_LENGTH    7
#define MAX_ADC_VALUE     1024
#define MIN_ADC_VALUE     0
#define BASE_DECIMAL      10
#define GRADF_TO_GRAD     100000
#define MAX_SAMPLES       144
#define SOFT_VER_MAIN     1
#define SOFT_VER_SUB      0
#define HARD_VER_MAIN     0
#define HARD_VER_SUB      2
#define SERIAL_NUMBER     1
#define EXT_ADC_1_ADD     0x49
#define EXT_ADC_2_ADD     0x48
#define CHNL_PER_EXT_ADC  4
#define EXT_BAUD_RATE     115200
#define MAX_INPUT_VOLT    36
#define MAX_OUTPUT_VOLT   5
#define MAX_INT_ADC_VOLT  5
#define RES_INP_MAP_L     18000
#define RES_INP_MAP_S     2000
#define RES_OUT_MAP_L     10000
#define RES_OUT_MAP_S     2500


#define PROTOCOL_SEPERATOR ':'

typedef enum {REG_MODE_EXECUTE = 0, REG_MODE_READ, REG_MODE_WRITE, REG_MODE_MAX}regMode_t;
typedef enum {EXT_ADC_1 = 0, EXT_ADC_2, MAX_ADC_COUNT}extAdc_t;
typedef enum {EXT_ADC_CH1 = 0, EXT_ADC_CH2, EXT_ADC_CH3, EXT_ADC_CH4, EXT_ADC_CH5, EXT_ADC_CH6, EXT_ADC_CH7, EXT_ADC_CH8, MAX_ADC_CHANNELS}extAdcChannels_t;
typedef enum {INPUT_VOLTAGE = 0, OUTPUT_VOLTAGE, MAX_VOLTAGE}measuringVolt_t;

cmdDet_t cmdDet;

char message[BUFFER_SIZE];
char tempRegRes[35];
uint8_t buffer_count = 0;
uint16_t time_count = 0;
uint16_t second_count = 0;
uint16_t minute_count = 0;
uint8_t testLEDPin = 13;
uint8_t ExtAdcRedyPin[MAX_ADC_COUNT] = { 2, 3 };  // interrupt pins for device 1 and 2
uint8_t measuringPin[MAX_VOLTAGE] = {A0, A1}; // analog pins fro measure input and output voltges
bool extAdcStatus[MAX_ADC_COUNT] = { false, false };
int16_t extAdcVal[MAX_ADC_COUNT*CHNL_PER_EXT_ADC] = { 0, 0, 0, 0, 0, 0, 0, 0 };

///..............PROGRAM.............///
void initTimer(void);
void initSerial(void);
void initExtAdc(void);
void initVoltageRead(void);
void initCmdDet(void);
void idle(void);
bool updateExtAdcVal(void);
void setExtAdcReady(uint8_t adc,bool status_adc);
int16_t getExtAdcValue(uint8_t channel);
void setExtAdcValue(uint8_t channel,int16_t value);
uint8_t getExtAdcIntPin(uint8_t pin);
float convertVoltage(uint8_t channel);
float getVoltage(uint8_t channel);
bool isExtAdcReady(uint8_t adc);
bool handleConversion(void);

void ext_adc_1_set(void);
void ext_adc_2_set(void);

///...............COMMUNICATION...........///
void commsHandle(void);
bool checkForEnd(char inByte);
void commandHandle(char *msg);
bool isMsgValid(char *msg);
char findCommand( char *header, char *str);
void showResults(char res, char *str);
void updateHeader(char *hdr);
bool LookupRegMode(uint8_t regMode);
static char LookupAndRunCommand(uint16_t reg, char *param_ptr);
void ProccessCmd(void);
void ExicuteRinCmd(void);
void WriteRinRegister(void);
void ReplyRinCmd(void);
void MakeHeader(char *message);
long hex_to_long(char *string, unsigned short width);
void substr(char *str, uint8_t str_val, uint8_t end_val, char *ans);


char SoftwareVersion(uint8_t mode, char *str);
char HardwareVersion(uint8_t mode, char *str);
char SerialNumber(uint8_t mode, char *str);

char SampleFunction(uint8_t mode, char *str);
char InpuVoltage(uint8_t mode, char *str);
char OutputVoltage(uint8_t mode, char *str);
char AllExtAdcValue(uint8_t mode, char *str);
char Ch1ExtAdcValue(uint8_t mode, char *str);
char Ch2ExtAdcValue(uint8_t mode, char *str);
char Ch3ExtAdcValue(uint8_t mode, char *str);
char Ch4ExtAdcValue(uint8_t mode, char *str);
char Ch5ExtAdcValue(uint8_t mode, char *str);
char Ch6ExtAdcValue(uint8_t mode, char *str);
char Ch7ExtAdcValue(uint8_t mode, char *str);
char Ch8ExtAdcValue(uint8_t mode, char *str);
char Ch1ToCh6ExtAdcValue(uint8_t mode, char *str);

char NullTest(uint8_t mode, char *str);

static const cmd_t cmdSet[]=
{
  {REG_SOFTWARE_VER,                    RR__,   SoftwareVersion},
  {REG_HARDWARE_VER,                    RR__,   HardwareVersion},
  {REG_SERIAL_NUMBER,                   RR__,   SerialNumber},

  {REG_INPUT_VOLTAGE,                   RR__,   InpuVoltage},
  {REG_OUTPUT_VOLTAGE,                  RR__,   OutputVoltage},
  
  {REG_ALL_ADC_VALUE,                   RR__,   AllExtAdcValue},
  {REG_CH1_ADC_VALUE,                   RR__,   Ch1ExtAdcValue},
  {REG_CH2_ADC_VALUE,                   RR__,   Ch2ExtAdcValue},
  {REG_CH3_ADC_VALUE,                   RR__,   Ch3ExtAdcValue},
  {REG_CH4_ADC_VALUE,                   RR__,   Ch4ExtAdcValue},
  {REG_CH5_ADC_VALUE,                   RR__,   Ch5ExtAdcValue},
  {REG_CH6_ADC_VALUE,                   RR__,   Ch6ExtAdcValue},
  {REG_CH7_ADC_VALUE,                   RR__,   Ch7ExtAdcValue},
  {REG_CH8_ADC_VALUE,                   RR__,   Ch8ExtAdcValue},
  {REG_CH1_TO_CH6_ADC_VALUE,            RR__,   Ch1ToCh6ExtAdcValue},
  

  {REG_END,                             RRWE,   NullTest}
};


// adjust addresses if needed
ADS1115 extADC_1(EXT_ADC_1_ADD);
ADS1115 extADC_2(EXT_ADC_2_ADD);

void setup()
{
  pinMode(testLEDPin,OUTPUT);
  initTimer();
  initSerial();
  initExtAdc();
  initVoltageRead();
}


void loop()
{
  idle();
  commsHandle();

/*
  for (int i = 0; i < 8; i++)
  {
    Serial.print(extAdcVal[i]);
    Serial.print('\t');
    updateExtAdcVal();
  }
  Serial.println();
  delay(100);
*/
}

void ext_adc_1_set(void)
{
  setExtAdcReady(EXT_ADC_1,true);
}

void ext_adc_2_set(void)
{
  setExtAdcReady(EXT_ADC_2,true);
}

bool isExtAdcReady(uint8_t adc)
{
  return extAdcStatus[adc];
}

uint8_t getExtAdcIntPin(uint8_t pin)
{
  return ExtAdcRedyPin[pin]; 
}

void setExtAdcReady(uint8_t adc,bool status_adc)
{
  extAdcStatus[adc] = status_adc;
}

void setExtAdcValue(uint8_t channel,int16_t value)
{
  extAdcVal[MAX_ADC_CHANNELS - channel - 1] = value;
}

int16_t getExtAdcValue(uint8_t channel)
{
  return extAdcVal[channel];
}

bool updateExtAdcVal(void)
{
  bool update_st = false;
  static uint8_t ch_1 = 0, ch_2 = 0;
  if(isExtAdcReady(EXT_ADC_1))
  {
    setExtAdcValue(ch_1,extADC_1.getValue());
    //extAdcVal[ch_1] = extADC_1.getValue();
    ch_1++;
    if (ch_1 >= CHNL_PER_EXT_ADC) 
    {
      ch_1 = 0;
    }
    extADC_1.readADC(ch_1);
    setExtAdcReady(EXT_ADC_1,false);
    update_st = true;
  }

  if(isExtAdcReady(EXT_ADC_2))
  {
    setExtAdcValue(CHNL_PER_EXT_ADC + ch_2,extADC_2.getValue());
    //extAdcVal[CHNL_PER_EXT_ADC + ch_2] = extADC_2.getValue();
    ch_2++;
    if (ch_2 >= CHNL_PER_EXT_ADC) 
    {
      ch_2 = 0;
    }
    extADC_2.readADC(ch_2);
    setExtAdcReady(EXT_ADC_2,false);
    update_st = true;
  }
  return update_st;
}

void initTimer(void)
{
  //timer 0 interrupt
  /*
  cli();//stop interrupts
  TCCR0A=(1<<WGM01);    //Set the CTC mode   
  OCR0A=0xF9; //Value for ORC0A for 1ms  
  
  TIMSK0|=(1<<OCIE1A);   //Set the interrupt request
  
  TCCR0B|=(1<<CS01);    //Set the prescale 1/64 clock
  TCCR0B|=(1<<CS00);

  sei(); //Enable interrupt
  */
  //timer 1 interrupt
  cli();//stop interrupts
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  OCR1A = 0xF9;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS01 and CS00 bits for 64 prescaler
  TCCR1B |= (1 << CS01) | (1 << CS00);  
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei(); //Enable interrupt
}

void initSerial(void)
{
  Serial.begin(EXT_BAUD_RATE);
}

void initExtAdc(void)
{
  uint8_t adc; 
  // SETUP FIRST ADS1115
  adc = EXT_ADC_1;
  extADC_1.begin();
  extADC_1.setGain(0);        // 6.144 volt
  extADC_1.setDataRate(7);

  // SET ALERT RDY PIN
  extADC_1.setComparatorThresholdHigh(0x8000);
  extADC_1.setComparatorThresholdLow(0x0000);
  extADC_1.setComparatorQueConvert(0);

  // SET INTERRUPT HANDLER TO CATCH CONVERSION READY
  setExtAdcReady(adc,false);
  pinMode(getExtAdcIntPin(adc), INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(getExtAdcIntPin(adc)), ext_adc_1_set, RISING);

  extADC_1.setMode(0);          // continuous mode
  extADC_1.readADC(0);  // trigger first read


  // SETUP SECOND ADS1115
  adc = EXT_ADC_2;
  extADC_2.begin();
  extADC_2.setGain(0);        // 6.144 volt
  extADC_2.setDataRate(7);

  // SET ALERT RDY PIN
  extADC_2.setComparatorThresholdHigh(0x8000);
  extADC_2.setComparatorThresholdLow(0x0000);
  extADC_2.setComparatorQueConvert(0);

  // SET INTERRUPT HANDLER TO CATCH CONVERSION READY
  setExtAdcReady(adc,false);
  pinMode(getExtAdcIntPin(adc), INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(getExtAdcIntPin(adc)), ext_adc_2_set, RISING);

  extADC_2.setMode(0);          // continuous mode
  extADC_2.readADC(0);  // trigger first read  
}

float convertVoltage(uint8_t channel)
{
  uint16_t digitalVal; 
  float analogVal;
  switch (channel)
  {
    case INPUT_VOLTAGE:
      digitalVal = (uint16_t)analogRead(measuringPin[INPUT_VOLTAGE]);
      analogVal = ((float)digitalVal/MAX_ADC_VALUE)*((float)(RES_INP_MAP_S + RES_INP_MAP_L)/RES_INP_MAP_S)*MAX_INT_ADC_VOLT;
      break;
    case OUTPUT_VOLTAGE:
      digitalVal = (uint16_t)analogRead(measuringPin[OUTPUT_VOLTAGE]);
      analogVal = ((float)digitalVal/MAX_ADC_VALUE)*((float)(RES_OUT_MAP_S + RES_OUT_MAP_L)/RES_OUT_MAP_L)*MAX_INT_ADC_VOLT;
      break;
  }
  return analogVal; 
}

float getVoltage(uint8_t channel)
{
  return convertVoltage(channel);
}

void initVoltageRead(void)
{
  pinMode(measuringPin[INPUT_VOLTAGE],INPUT);
  pinMode(measuringPin[OUTPUT_VOLTAGE],INPUT);
}

void initCmdDet(void)
{
  cmdDet.mode = 0;
  cmdDet.reg = 0;
  memset(cmdDet.data,'\0', arrLen(cmdDet.data));
  cmdDet.dataAvailable = false;
}

void idle(void)
{
  static bool status_pin = true;
  updateExtAdcVal();
  if(time_count > (MILI_TO_SECOND - 1))
  {
    second_count++;
    time_count = 0;
    /*
    for (int i = 0; i < 8; i++)
    {
      Serial.print(extAdcVal[i]);
      Serial.print('\t');
      updateExtAdcVal();
    }
    Serial.println();
  //delay(100);
  */
    if(status_pin)
      digitalWrite(testLEDPin,HIGH);
     else
      digitalWrite(testLEDPin,LOW);
    status_pin = !status_pin;
  }
  
  if(second_count > (SECOND_TO_MINUTE - 1))
  {
    //updateReadings(minute_count);
    //printMinuteData(minute_count);
    minute_count++;
    second_count = 0;
  }
  
  if (minute_count > (SAMPLE_RATE - 1))
  {
    //updateSample();
    minute_count = 0;
  }
}

///...............COMMUNICATION...........///
void commsHandle(void)
{
  char incomingByte;
  bool msgAvailable = false;
  
  while (Serial.available() > 0)
  {
    incomingByte = Serial.read();
    message[buffer_count++] = incomingByte;

    if(checkForEnd(incomingByte))
    {
      msgAvailable = true;
      break;
      //Serial.flush();
    }
        
    if (buffer_count > (BUFFER_SIZE - 1))
      break;
  }
  
  if (msgAvailable)
  {
   commandHandle(message);
   memset(message, '\0', BUFFER_SIZE);
   buffer_count = 0;
  }
}

void commandHandle(char *msg)
{
 
  char *cp, *str;
  uint8_t heade_array_lenght = 20;
  char header[heade_array_lenght], res;
  uint8_t headerLength = 0 , count = 0 ;

  if (isMsgValid(msg))
  {
    cp = strchr(msg,PROTOCOL_SEPERATOR);
    /*
    if((uint8_t)(cp) > MIN_CMD_LENGTH)
    {
      memset(header, '\0', heade_array_lenght);
      substr(msg,(uint8_t)cp - MIN_CMD_LENGTH + 1,(uint8_t)cp,header);
      //strncpy( header , msg , headerLength );
    }
    else
    {
      headerLength = (uint8_t)(cp - msg);
      memset(header, '\0', heade_array_lenght);
      strncpy( header , msg , headerLength );
    }
    */
    headerLength = (uint8_t)(cp - msg);
    memset(header, '\0', heade_array_lenght);
    strncpy( header , msg , headerLength );
    initCmdDet();
    updateHeader(header);
    //strcpy(cmdDet.header,header);
    *cp++;
    if(checkForEnd(*cp))
      cmdDet.data[0] = '0';
       
    while(!checkForEnd(*cp))
    {
      cmdDet.data[count++] = *cp++;
    }
    
    ProccessCmd();
    //showResults(res, str); 
  }
}

void substr(char *str, uint8_t str_val, uint8_t end_val, char *ans) 
{
    strncpy(ans, str+str_val, end_val);
}

void updateHeader(char *hdr)
{
  uint8_t mode_length = 3,reg_length = 5;
  char mode[mode_length];
  char reg[reg_length];
  
  memset(reg, '\0', reg_length);
  memset(mode, '\0', mode_length);
  mode[0] = *hdr++;
  mode[1] = *hdr++;
  reg[0] = *hdr++;
  reg[1] = *hdr++;
  reg[2] = *hdr++;
  reg[3] = *hdr++;
  //strncpy( mode , hdr , 2);
  //*hdr++;
  //*hdr++;
  //strncpy( reg , hdr , 4);
  //hex_to_long
  cmdDet.mode = (uint8_t)strtol(mode,NULL,16);
  cmdDet.reg = (uint16_t)strtol(reg,NULL,16);
  //cmdDet.mode = (uint8_t)hex_to_long(mode,2);
  //cmdDet.reg = (uint16_t)hex_to_long(reg,4);
        //Serial.print(cmdDet.mode);
      //Serial.print(",");
      //Serial.println(cmdDet.reg);
  
}

/*...............................End of Register Related functions.................................... */

/* see response is a magic number
 *
 * magic numbers (defined by programmer)
 *
 *   20: Invalid register
 *  21: Authentication fails
 *  22: Write successful
 *  23: Write fail
 *
 * */

void GetRegValue(uint16_t addres ,char *data)
{
  uint8_t res;
  
  res = LookupAndRunCommand(addres,data);
  switch (res)    // see magic 20 or 21 (20 is responsible for no reg found, 21 is responsible for authentication fails)
  {
    case (char)23:
      memset(tempRegRes, '\0', arrLen(tempRegRes));
      sprintf(tempRegRes, "%s;", "Fail");
      break;
    case (char)22:
      memset(tempRegRes, '\0', arrLen(tempRegRes));
      sprintf(tempRegRes, "%s;", "Ok");
      break;
    case (char)21:
      memset(tempRegRes, '\0', arrLen(tempRegRes));
      sprintf(tempRegRes, "%s;", "Auth_Fail");
      break;
    case (char)20:
      memset(tempRegRes, '\0', arrLen(tempRegRes));
      sprintf(tempRegRes, "%s;", "Inv_Reg");
      break;
  }
}

/** Lookup Reg Mode and return true if the incoming mode from rinCommand allow.*/
bool LookupRegMode(uint8_t regMode)
{
  bool status = false;
  uint8_t incomingRegMode;

  incomingRegMode = cmdDet.mode;
  switch (incomingRegMode)
  {
  case REG_READABLE:
    status = ((regMode == RRWE) ||
              (regMode == RRW_) ||
              (regMode == RR_E) ||
              (regMode == RR__)   )? true: false;
    break;
  case REG_WRITABLE:
    status = ((regMode == RRWE) ||
              (regMode == R_WE) ||
              (regMode == RRW_) ||
              (regMode == R_W_)   )? true: false;
    break;
  case REG_EXECUTABLE:
    status = ((regMode == RRWE) ||
              (regMode == R_WE) ||
              (regMode == RR_E) ||
              (regMode == R__E)   )? true: false;
    break;
  }

  return status;
}

/** Lookup cmd in the list of tests, and run the command.
 * Returns magic 20 if not found.
 * Returns magic 21 if register is not accessible with the current mode
 * */
static char LookupAndRunCommand(uint16_t reg, char *param_ptr)
{
  int i = 0;
  char result = 0;
  uint8_t regMode = 0;

  /* Locate command */
  while(!(cmdSet[i].reg == reg))
  {
    if((cmdSet[i].reg == REG_END))
      break;
    i += 1;
  }

  if(cmdSet[i].reg == REG_END)
    result = cmdSet[i].cmd_fn(regMode, param_ptr);
  else
  {
    if(LookupRegMode(cmdSet[i].mode))
    {
      switch(cmdDet.mode)
      {
        case REG_EXECUTABLE:
          regMode = REG_MODE_EXECUTE;
          break;
        case REG_READABLE:
          regMode = REG_MODE_READ;
          break;
        case REG_WRITABLE:
          regMode = REG_MODE_WRITE;
          break;
      }
      result = cmdSet[i].cmd_fn(regMode, param_ptr);
    }
    else
      result = 21;
  }

  return result;
}

char findCommand( char *header, char *str)
{
//  uint8_t count = 0;
//  char res;

//  for (count = 0 ; (strcmp("END" ,cmdSet[count].command) != 0) ; count++)
//  {
//    if (strcmp(header,cmdSet[count].command) == 0)
//      break;
//  }
//  res = cmdSet[count].cmd_fn(str); 
//  return res; 
}

long hex_to_long(char *string, unsigned short width)
{
   char szTemp[sizeof(long)*2 + 1]="";

   if (width > sizeof(long)*2)
      width = sizeof(long)*2;
   strncat(szTemp, string, (size_t)width);

   return strtol(szTemp, NULL, 0x10);
}

void MakeHeader(char *message)
{
  uint8_t cmdMode = cmdDet.mode;
  uint16_t cmdReg = cmdDet.reg;
//  sprintf(message, "%02X%0*X:", (uint8_t)pHeader->mode,
 //          (uint8_t)(sizeof(pHeader->reg) * 2), (uint16_t)pHeader->reg);
   sprintf(message, "%02X%04X:", cmdMode, cmdReg);
}

void ReplyRinCmd(void)
{
  char szTemp[15];
  char szReply[35];

  memset(szReply, '\0', arrLen(szReply));
  memset(szTemp, '\0', arrLen(szTemp));
  MakeHeader(szTemp);
  GetRegValue(cmdDet.reg ,szReply);
  Serial.print(szTemp);
  Serial.println(tempRegRes);
}

void WriteRinRegister(void)
{
  char szReply[35];
  char szHead[15];
  
  memset(szReply, '\0', arrLen(szReply));
  memset(szHead, '\0', arrLen(szHead));
  GetRegValue(cmdDet.reg ,szReply);
  MakeHeader(szHead);
  Serial.print(szHead);
  Serial.println(tempRegRes);
}

void ExicuteRinCmd(void)
{
  char szReply[35];
  char szHead[15];

  memset(szReply, '\0', arrLen(szReply));
  memset(szHead, '\0', arrLen(szHead));
  GetRegValue(cmdDet.reg ,szReply);
  MakeHeader(szHead);
  Serial.print(szHead);
  Serial.println(tempRegRes);
}

void ProccessCmd(void)
{
  switch(cmdDet.mode)
  {
    case (uint8_t)REG_EXECUTABLE:
      ExicuteRinCmd();
      break;
    case (uint8_t)REG_READABLE:
      ReplyRinCmd();
      break;
    case (uint8_t)REG_WRITABLE:
      WriteRinRegister();
      break;
  }

}

void showResults(char res, char *str)
{
  switch(res)
  {
    case 19:
      break;
    case 20:
      Serial.print(cmdDet.mode);
      Serial.print(cmdDet.reg);
      Serial.print(":");
      Serial.println("FAIL");
      break;
    case 21:
      Serial.print(cmdDet.mode);
      Serial.print(cmdDet.reg);
      Serial.print(":");
      Serial.println("SUCCESS");
      break;
    case 22:
      Serial.print(cmdDet.mode);
      Serial.print(cmdDet.reg);
      Serial.print(":");
      Serial.println(str);
      break;
  }
}

bool isMsgValid(char *msg)
{
  char *cp;
  bool msgValid = false;
  
  cp = strchr(msg,PROTOCOL_SEPERATOR);
  msgValid = ((cp != NULL) && ((cp - message) > (MIN_CMD_LENGTH - 2)))? true : false;
    
  return msgValid;
}

bool checkForEnd(char inByte)
{
  bool msgEnd = false;
  switch(inByte)
  {
    case ';':
    case '\r':
    case '\n':
      msgEnd = true;
    break;
  }
  return msgEnd;
}

char SampleFunction(uint8_t mode, char *str)
{
  char status = 0; 

  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    // nothing to Exicute
    //status = 23;
    break;
  case REG_MODE_READ:
    sprintf(tempRegRes, "%d,%d,%d,%d;", 1, 2, 3, 4);
    status = 19;
    break;
  case REG_MODE_WRITE:
    // noting to write
    break;
  }
  return status;
}

char InpuVoltage(uint8_t mode, char *str)
{
  char status = 0; 
  char str_temp[6];
  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    break;
  case REG_MODE_READ:
    dtostrf(getVoltage(INPUT_VOLTAGE), 4, 2, str_temp);
    sprintf(tempRegRes, "%sV;", str_temp);
    status = 19;
    break;
  case REG_MODE_WRITE:
    break;
  }
  return status;
}

char OutputVoltage(uint8_t mode, char *str)
{
  char status = 0;
  char str_temp[6];
  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    break;
  case REG_MODE_READ:
    dtostrf(getVoltage(OUTPUT_VOLTAGE), 4, 2, str_temp);
    sprintf(tempRegRes, "%sV;", str_temp);
    status = 19;
    break;
  case REG_MODE_WRITE:
    break;
  }
  return status;
}

char Ch1ExtAdcValue(uint8_t mode, char *str)
{
  char status = 0; 
  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    break;
  case REG_MODE_READ:
    sprintf(tempRegRes, "%04X;", getExtAdcValue(EXT_ADC_CH1));
    status = 19;
    break;
  case REG_MODE_WRITE:
    break;
  }
  return status;
}

char Ch2ExtAdcValue(uint8_t mode, char *str)
{
  char status = 0; 
  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    break;
  case REG_MODE_READ:
    sprintf(tempRegRes, "%04X;", getExtAdcValue(EXT_ADC_CH2));
    status = 19;
    break;
  case REG_MODE_WRITE:
    break;
  }
  return status;
}

char Ch3ExtAdcValue(uint8_t mode, char *str)
{
  char status = 0; 
  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    break;
  case REG_MODE_READ:
    sprintf(tempRegRes, "%04X;", getExtAdcValue(EXT_ADC_CH3));
    status = 19;
    break;
  case REG_MODE_WRITE:
    break;
  }
  return status;
}

char Ch4ExtAdcValue(uint8_t mode, char *str)
{
  char status = 0; 
  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    break;
  case REG_MODE_READ:
    sprintf(tempRegRes, "%04X;", getExtAdcValue(EXT_ADC_CH4));
    status = 19;
    break;
  case REG_MODE_WRITE:
    break;
  }
  return status;
}

char Ch5ExtAdcValue(uint8_t mode, char *str)
{
  char status = 0; 
  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    break;
  case REG_MODE_READ:
    sprintf(tempRegRes, "%04X;", getExtAdcValue(EXT_ADC_CH5));
    status = 19;
    break;
  case REG_MODE_WRITE:
    break;
  }
  return status;
}

char Ch6ExtAdcValue(uint8_t mode, char *str)
{
  char status = 0; 
  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    break;
  case REG_MODE_READ:
    sprintf(tempRegRes, "%04X;", getExtAdcValue(EXT_ADC_CH6));
    status = 19;
    break;
  case REG_MODE_WRITE:
    break;
  }
  return status;
}

char Ch7ExtAdcValue(uint8_t mode, char *str)
{
  char status = 0; 
  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    break;
  case REG_MODE_READ:
    sprintf(tempRegRes, "%04X;", getExtAdcValue(EXT_ADC_CH7));
    status = 19;
    break;
  case REG_MODE_WRITE:
    break;
  }
  return status;
}

char Ch8ExtAdcValue(uint8_t mode, char *str)
{
  char status = 0; 
  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    break;
  case REG_MODE_READ:
    sprintf(tempRegRes, "%04X;", getExtAdcValue(EXT_ADC_CH8));
    status = 19;
    break;
  case REG_MODE_WRITE:
    break;
  }
  return status;
}

char Ch1ToCh6ExtAdcValue(uint8_t mode, char *str)
{
  char status = 0; 

  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    // nothing to Exicute
    //status = 23;
    break;
  case REG_MODE_READ:
    sprintf(tempRegRes, "%04X,%04X,%04X,%04X,%04X,%04X;", getExtAdcValue(EXT_ADC_CH1),getExtAdcValue(EXT_ADC_CH2),
                                                          getExtAdcValue(EXT_ADC_CH3),getExtAdcValue(EXT_ADC_CH4),
                                                          getExtAdcValue(EXT_ADC_CH5),getExtAdcValue(EXT_ADC_CH6));
    status = 19;
    break;
  case REG_MODE_WRITE:
    // noting to write
    break;
  }
  return status;
}

char AllExtAdcValue(uint8_t mode, char *str)
{
  char status = 0; 

  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    // nothing to Exicute
    //status = 23;
    break;
  case REG_MODE_READ:
    sprintf(tempRegRes, "%04X,%04X,%04X,%04X,%04X,%04X,%04X,%04X;", getExtAdcValue(EXT_ADC_CH1),getExtAdcValue(EXT_ADC_CH2),
                                                                    getExtAdcValue(EXT_ADC_CH3),getExtAdcValue(EXT_ADC_CH4),
                                                                    getExtAdcValue(EXT_ADC_CH5),getExtAdcValue(EXT_ADC_CH6),
                                                                    getExtAdcValue(EXT_ADC_CH7),getExtAdcValue(EXT_ADC_CH8));
    status = 19;
    break;
  case REG_MODE_WRITE:
    // noting to write
    break;
  }
  return status;
}

char SoftwareVersion(uint8_t mode, char *str)
{
  char status = 0; 

  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    // nothing to Exicute
    //status = 23;
    break;
  case REG_MODE_READ:
    sprintf(tempRegRes, "V%d.%02d;", SOFT_VER_MAIN, SOFT_VER_SUB);
    status = 19;
    break;
  case REG_MODE_WRITE:
    // noting to write
    break;
  }
  return status;
}

char HardwareVersion(uint8_t mode, char *str)
{
  char status = 0; 

  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    // nothing to Exicute
    //status = 23;
    break;
  case REG_MODE_READ:
    sprintf(tempRegRes, "V%d.%02d;", HARD_VER_MAIN, HARD_VER_SUB);
    status = 19;
    break;
  case REG_MODE_WRITE:
    // noting to write
    break;
  }
  return status;
}

char SerialNumber(uint8_t mode, char *str)
{
  char status = 0; 

  memset(tempRegRes, '\0', arrLen(tempRegRes));
  switch (mode)
  {
  case REG_MODE_EXECUTE:
    // nothing to Exicute
    //status = 23;
    break;
  case REG_MODE_READ:
    sprintf(tempRegRes, "%07d;", SERIAL_NUMBER);
    status = 19;
    break;
  case REG_MODE_WRITE:
    // noting to write
    break;
  }
  return status;
}

char NullTest(uint8_t mode, char *str)
{
  return (char)20;
}

/*
ISR(TIMER0_COMPA_vect)
{
  //initTimer();
  time_count++; 
}*/

ISR(TIMER1_COMPA_vect)
{
  //initTimer();
  time_count++; 
}
