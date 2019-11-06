/*
 * @file state_machine.h
 * @brief Project 4
 *
 * @details Contains both a table and state based state machine.
 *
 * @tools  PC Compiler: GNU gcc 8.3.0
 *         PC Linker: GNU ld 2.32
 *         PC Debugger: GNU gdb 8.2.91.20190405-git
 *         ARM Compiler: GNU gcc version 8.2.1 20181213
 *         ARM Linker: GNU ld 2.31.51.20181213
 *         ARM Debugger: GNU gdb 8.2.50.20181213-git
 *
 *  LEVERAGED CODE: Much of the implementation/interface for this code
 *  comes from the Arduino TMP102 library at
 *  https://learn.sparkfun.com/tutorials/tmp102-digital-temperature-sensor-hookup-guide
 */

#include "i2c.h"
#include "tmp102.h"

#define USE_TMP102

/**
 * @brief Base address of the TMP102
 */
#define TMP102_PERIPHERAL_ADDR (0x48<<1)

/**
 * @brief TMP102 temperature register
 */
#define TEMPERATURE_REGISTER 0x00

/**
 * @brief TMP102 config register
 */
#define CONFIG_REGISTER 0x01

/**
 * @brief TMP102 T LOW register
 */
#define T_LOW_REGISTER 0x02

/**
 * @brief TMP102 T HIGH register
 */
#define T_HIGH_REGISTER 0x03

#ifdef USE_TMP102

//  - Returns the current temperature in Celsius.
float readTempC()
{
	uint8_t registerByte[2]= {0,0};	// Store the data from the register here
	int16_t digitalTemp;  // Temperature stored in TMP102 register

	// Read Temperature
	// Change pointer address to temperature register (0)
	i2c_read_bytes(TMP102_PERIPHERAL_ADDR, TEMPERATURE_REGISTER, registerByte, 2);

	// Combine bytes to create a signed int
	digitalTemp = ((registerByte[0]) << 4) | (registerByte[1] >> 4);
	// Temperature data can be + or -, if it should be negative,
	// convert 12 bit to 16 bit and use the 2s compliment.
	if(digitalTemp > 0x7FF)
	{
	  digitalTemp |= 0xF000;
	}
	// Convert digital reading to analog temperature (1-bit is equal to 0.0625 C)
	return digitalTemp*0.0625;
}

// - Reads T_LOW register in Celsius.
float readLowTempC()
{
	uint8_t registerByte[2];	// Store the data from the register here
	int16_t digitalTemp;		// Store the digital temperature value here

	i2c_read_bytes(TMP102_PERIPHERAL_ADDR, T_LOW_REGISTER, registerByte, 2);

	// Combine bytes to create a signed int
	digitalTemp = ((registerByte[0]) << 4) | (registerByte[1] >> 4);
	// Temperature data can be + or -, if it should be negative,
	// convert 12 bit to 16 bit and use the 2s compliment.
	if(digitalTemp > 0x7FF)
	{
	  digitalTemp |= 0xF000;
	}

	// Convert digital reading to analog temperature (1-bit is equal to 0.0625 C)
	return digitalTemp*0.0625;
}

//  - Reads T_HIGH register in Celsius.
float readHighTempC()
{
	uint8_t registerByte[2];	// Store the data from the register here
	int16_t digitalTemp;		// Store the digital temperature value here

	i2c_read_bytes(TMP102_PERIPHERAL_ADDR, T_HIGH_REGISTER, registerByte, 2);

	// Combine bytes to create a signed int
	digitalTemp = ((registerByte[0]) << 4) | (registerByte[1] >> 4);
	// Temperature data can be + or -, if it should be negative,
	// convert 12 bit to 16 bit and use the 2s compliment.
	if(digitalTemp > 0x7FF)
	{
	  digitalTemp |= 0xF000;
	}

	// Convert digital reading to analog temperature (1-bit is equal to 0.0625 C)
	return digitalTemp*0.0625;
}

// - Returns the state of the Alert register. The state of the register is the same as the ALT pin.
bool alert()
{
	  // Change pointer address to configuration register (1)
	  uint8_t registerByte = i2c_read_byte(TMP102_PERIPHERAL_ADDR, CONFIG_REGISTER);

	  registerByte &= 0x20;	// Clear everything but the alert bit (bit 5)
	  return registerByte>>5;
}

// - Sets T_LOW (in Celsius) alert threshold.
void setLowTempC(float temperature)
{
	uint8_t registerByte[2];	// Store the data from the register here
	// Prevent temperature from exceeding 150C or -55C
	if(temperature > 150.0f)
	{
		temperature = 150.0f;
	}
	if(temperature < -55.0)
	{
		temperature = -55.0f;
	}

	// Split temperature into separate bytes
	registerByte[0] = (int)(temperature)>>4;
	registerByte[1] = (int)(temperature)<<4;


	i2c_write_bytes(TMP102_PERIPHERAL_ADDR, T_LOW_REGISTER, registerByte, 2);
}

// - Sets T_HIGH (in Celsius) alert threshold.
void setHighTempC(float temperature)
{
	uint8_t registerByte[2];	// Store the data from the register here
	// Prevent temperature from exceeding 150C or -55C
	if(temperature > 150.0f)
	{
		temperature = 150.0f;
	}
	if(temperature < -55.0)
	{
		temperature = -55.0f;
	}

	// Split temperature into separate bytes
	registerByte[0] = (int)(temperature)>>4;
	registerByte[1] = (int)(temperature)<<4;


	i2c_write_bytes(TMP102_PERIPHERAL_ADDR, T_HIGH_REGISTER, registerByte, 2);
}

// - Sets the type of alert.
//   0: Comparator Mode (Active from when temperature > T_HIGH until temperature < T_LOW),
//   1: Thermostat mode (Active from when temperature > T_HIGH until any read operation occurs.
void setAlertMode(bool mode)
{
	  // Change pointer address to configuration register (1)
	  uint8_t registerByte = i2c_read_byte(TMP102_PERIPHERAL_ADDR, CONFIG_REGISTER);

	  registerByte &= 0xFD;	// Clear old TM bit (bit 1 of first byte)
	  registerByte |= mode<<1;	// Shift in new TM bit

	  delay(300); // wait before you read...

	  i2c_write_byte(TMP102_PERIPHERAL_ADDR, CONFIG_REGISTER, registerByte);
}

bool tmp102_connected()
{
	return i2c_connected(TMP102_PERIPHERAL_ADDR);
}

#else

float readTempC()
{
	static uint64_t sentinel;
	if((sentinel++ % 200)==0) return -5.273;
	else if((sentinel++ % 40)==0) return 3.68547;
	else if((sentinel++ % 5)==0) return 20.0;
	else return -23.342;
}

// - Reads T_LOW register in Celsius.
float readLowTempC()
{
	return -40.0;
}

//  - Reads T_HIGH register in Celsius.
float readHighTempC()
{
	return 40.0;
}

// - Returns the state of the Alert register. The state of the register is the same as the ALT pin.
bool alert()
{
	static uint64_t sentinel;
	if((sentinel++ % 200)==0) return true;

	return false;
}

void setLowTempC(float temperature)
{
}

void setHighTempC(float temperature)
{
}

void setAlertMode(bool mode)
{
}

// stub
bool tmp102_connected()
{
	return true;
}

#endif
