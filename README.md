# PES Project 4 Readme
Jack Campbell

## Description
This repo contains custom sources and makefiles for Project 4 as well as adapted and generated code 
from MCUXpresso and the KL25Z SDK.

This project contains three configurations: Test, Debug, and Normal.

Test runs a suite of unit tests covering the state machine code.

Debug and Normal both run the state machines controlling the TMP102, but contain different logging
statement levels.

The state machine uses a strategy pattern to ping-pong its event handler between a table based and
a state-based state machine.

## Observations

I spent the bulk of my time on the front end of this project working on designing the state machine and
logging infrastructure, which ended up being potentially a bad call -- I saved the I2C and TMP102 work 
for the weekend before it was due, and had a lot of trouble debugging that communication.

I also spent most of the weekend learning how to use an oscilloscope and how to pick a header pin configuration
to solder to my board, which was a good experience, but pushed my I2C troubles even closer to the deadline.

In the future I think I will consider my "unbounded work" anything hardware related and try to get it handled first.
Writing and designing pure software is more bounded and can be crammed into the end of a schedule easier than
something I have much less experience with.


I've still attached my logic analyzer diagrams, since I believe the I2C transactions were 
*not* malformed... I point out in the diagrams everywhere where there *should* be a TMP102 ACK, but 
none arrives. However, these graphs show that the master  is doing what it is supposed to be doing. 
According to several students and Shreya, my init code and I2C code is doing the same thing as others who have
a working solution...

I spent a fair amount of time debugging with Shreya (five hours) and tried three different TMP102's and 4 different KL25Z's.
I even tried running Shreya's working I2C code and it wasn't working when run under my setup. So something is wrong in my
setup, probably something simple and dumb, but in the end I decided it was beyond my dedication to get it actually working.


To show off the state machine logic, I added a define in tmp102.c called USE_TMP102. If this is not defined, then I 
have stub functions that return "meaningful" values so I can demonstrate the sequence of the state machine.


## Installation/Execution Notes

These are the steps to build the project in MCUXpresso.

1) Clone the repo
2) In MCUXpresso, click `New > Project`.
3) Select `Makefile project with existing code...`
4) Unselect C++, enter a project name, browse to the directory of the repo, and select `NXP MCU Tools`, then hit next.
5) Now set up the configurations. Right click the project,
6) Hit Properties
7) Uncheck "Generate makefiles"
8) Add "Debug" to the build directory path in the same dialog.
9) Do the same for Normal and Test configurations.

### Running the FB builds

1) Right click on the project name in the file hierarchy, select `Debug as > Debug configurations...`
2) Select `GDB PEMicro Interface Debugging`
3) Hit `New launch configuration`
4) Select a name for the output configuration (you need one for both Release and Debug)
5) Set the `C/C++ Application` field to the binary you want to run, either `Debug/output/kl25z_debug.axf` for Debug or `Release/output/kl25z_run.axf` for Release
6) Hit Apply
7) Hit Debug
8) The program should run in the console below, provided the board is connected successfully.


# CODE

```
/*
* @file delay.h
* @brief Project 4
*
* @details This file contains prototypes for calculating a spin-wait
*          on various platforms, used for delaying LED state changes.
*
* @author Jack Campbell
* @tools  PC Compiler: GNU gcc 8.3.0
*         PC Linker: GNU ld 2.32
*         PC Debugger: GNU gdb 8.2.91.20190405-git
*         ARM Compiler: GNU gcc version 8.2.1 20181213
*         ARM Linker: GNU ld 2.31.51.20181213
*         ARM Debugger: GNU gdb 8.2.50.20181213-git
*/
#ifndef PES_PROJECT_4_DELAY_H
#define PES_PROJECT_4_DELAY_H

#include <stdint.h>

/**
* delay
*
* @brief Blocks execution for the specified time.
* @param inDelayMs Then time in milliseconds to block.
*/
void delay(uint64_t inDelayMs);

#endif //PES_PROJECT_4_DELAY_H
/*
* @file handle_led.h
* @brief Project 4
*
* @details Contains the prototype for handling LEDs on various platforms.
*          This may be actually turning an LED on and off or just printing
*          what the LED state would be, in the absence of LEDs.
*
* @author Jack Campbell
* @tools  PC Compiler: GNU gcc 8.3.0
*         PC Linker: GNU ld 2.32
*         PC Debugger: GNU gdb 8.2.91.20190405-git
*         ARM Compiler: GNU gcc version 8.2.1 20181213
*         ARM Linker: GNU ld 2.31.51.20181213
*         ARM Debugger: GNU gdb 8.2.50.20181213-git
*/
#ifndef PES_PROJECT_4_HANDLE_LED_H
#define PES_PROJECT_4_HANDLE_LED_H

#include <stdint.h>
#include "led_types.h"

/**
* set_led
*
* @brief Sets the LED state.
* @details This function, depending on platform, may or may not
*          control a physical LED. On PC, it will simply print the
*          state of what the LED would be.
* @param inValue The on/off state of the LED to set.
* @param inColor The color of the LED to set.
*/
void set_led(uint8_t inValue, enum COLOR inColor);

#endif //PES_PROJECT_2_HANDLE_LED_H
/*
* @file i2c.h
* @brief Project 4
*
* @details Handles I2C transactions.
*
* @author Jack Campbell
* @tools  PC Compiler: GNU gcc 8.3.0
*         PC Linker: GNU ld 2.32
*         PC Debugger: GNU gdb 8.2.91.20190405-git
*         ARM Compiler: GNU gcc version 8.2.1 20181213
*         ARM Linker: GNU ld 2.31.51.20181213
*         ARM Debugger: GNU gdb 8.2.50.20181213-git
*
*  Attribution: Much of this code taken from from Embedded Systems Fundamentals
*  with Arm Cortex-M based Microcontrollers, Dean, 2017, ARM Education Media
*  (The word doc supplied by Prof. Montgomery)
*/

#ifndef PESI2C_H
#define PESI2C_H

#include <stdint.h>
#include <stdbool.h>
/**
* @brief Initialize I2C registers.
*/
void i2c_init(void);

/**
* @brief Write a single byte over I2C.
* @param dev The address of the peripheral to write to.
* @param reg The address of the register to write to.
* @param data The byte to write to that register.
*/
void i2c_write_byte(uint8_t dev, uint8_t reg, uint8_t data);

/**
* @brief Write multiple bytes over I2C.
* @param inSlaveAddr The address of the peripheral to write to.
* @param inRegAddr The address of the register to write to.
* @param inData The data to write.
* @param inDataSize The size of the data to write.
*/
void i2c_write_bytes(uint8_t inSlaveAddr, uint8_t inRegAddr, uint8_t* inData, int8_t inDataSize);

/**
* @brief Read a single byte over I2C.
* @param dev The address of the peripheral to read from.
* @param reg The register to read from.
* @return The byte read.
*/
uint8_t i2c_read_byte(uint8_t dev, uint8_t reg);

/**
* @brief Read multiple bytes over I2C.
* @param dev_adx The address of the peripheral to read from.
* @param reg_adx The address of the register to read from.
* @param outData The data read.
* @param data_count The size of the output array for data read.
*/
void i2c_read_bytes(uint8_t dev_adx, uint8_t reg_adx, uint8_t* outData, int8_t data_count);

/**
* Return whether the requested device gave back an ACK when requested.
*/
bool i2c_connected(uint8_t dev_adx);

#endif
/*
* @file led_types.h
* @brief Project 4
*
* @details Defines enumerations and constants used to describe colors and
*          on/off states for LEDs.
*
* @author Jack Campbell
* @tools  PC Compiler: GNU gcc 8.3.0
*         PC Linker: GNU ld 2.32
*         PC Debugger: GNU gdb 8.2.91.20190405-git
*         ARM Compiler: GNU gcc version 8.2.1 20181213
*         ARM Linker: GNU ld 2.31.51.20181213
*         ARM Debugger: GNU gdb 8.2.50.20181213-git
*/

#ifndef PES_PROJECT_2_LED_TYPES_H
#define PES_PROJECT_2_LED_TYPES_H

/**
* COLOR
*
* @brief The possible color values of the LED.
*/
enum COLOR
{
RED = 0,
GREEN,
BLUE,
NUM_COLORS
};

/**
* COLOR_STRINGS
*
* @brief String representations of the COLOR enum, used for printing.
*/
static const char * const COLOR_STRINGS[3] = {
"RED",
"GREEN",
"BLUE"
};

#endif //PES_PROJECT_2_LED_TYPES_H
/*
* @file logger.h
* @brief Project 4
*
* Interface to use for logging on either PC or KL25Z.
*
* @author Jack Campbell
* @tools  PC Compiler: GNU gcc 8.3.0
*         PC Linker: GNU ld 2.32
*         PC Debugger: GNU gdb 8.2.91.20190405-git
*         ARM Compiler: GNU gcc version 8.2.1 20181213
*         ARM Linker: GNU ld 2.31.51.20181213
*         ARM Debugger: GNU gdb 8.2.50.20181213-git
*/

#ifndef PES_PROJECT_3_LOGGER_H
#define PES_PROJECT_3_LOGGER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/**
* @brief The category in which log messages should appear.
*/
typedef enum LogSeverity
{
LOG_SEVERITY_TEST,
LOG_SEVERITY_DEBUG,
LOG_SEVERITY_STATUS,
NUM_LOG_SEVERITIES
} LogSeverity_t;

/**
* @brief The module associated with a log message.
*/
typedef enum LogModule
{
LOG_MODULE_MAIN,
LOG_MODULE_LED,
LOG_MODULE_UNIT_TEST,
LOG_MODULE_SETUP_TEARDOWN,
LOG_MODULE_STATE_MACHINE_STATE,
LOG_MODULE_STATE_MACHINE_TABLE,
LOG_MODULE_POST,
LOG_MODULE_TMP102,
LOG_MODULE_I2C,
NUM_LOG_MODULES
} LogModule_t;

/**
* @brief Log_enable – begin printing log messages when called
*/
void log_enable(LogSeverity_t inSeverity);

/**
* @brief Log_disable – ignore any log messages until re-enabled
*/
void log_disable();


void log_set_severity(LogSeverity_t inSeverity);

/**
* @brief Log_status – returns a flag to indicate whether the logger is enabled or disabled
* @return Whether the log is currently enabled.
*/
bool log_enabled();

/**
* @brief Log_data – display in hexadecimal an address and contents of a memory location,
* @param inModule The module associated with this log statement.
* @param inFuncName The function name from which we are logging.
* @param inSeverity The severity of this log statement.
* @param inBytes a pointer to a sequence of bytes to log
* @param inSize Number of bytes to log
*/
void log_data(LogModule_t inModule, const char* inFuncName, LogSeverity_t inSeverity, const uint8_t* inBytes, size_t inSize);

/**
* @brief A macro used to wrap a log data. Used to write the function name automatically.
*/
#define LOG_DATA(category, severity, data, length) \
{ \
log_data(category, __FUNCTION__, severity, data, length); \
}


/**
* @brief Log a string.
* @param inModule The module associated with this log statement.
* @param inFuncName The function name from which we are logging.
* @param inSeverity The severity of this log statement.
* @param inString
* @param ... Printf style args.
*/
void log_string(LogModule_t inModule, const char* inFuncName, LogSeverity_t inSeverity, const char* inString, ...);

/**
* @brief A macro used to wrap a log_string. Includes function name automatically and accepts printf-style args.
*/
#define LOG_STRING_ARGS(category, severity, fmt, ...) \
{ \
log_string(category, __func__, severity, fmt, __VA_ARGS__); \
}

/**
* @brief A macro used to wrap a log_string. Includes function name automatically.
*/
#define LOG_STRING(category, severity, fmt) \
{ \
log_string(category, __func__, severity, fmt); \
}

/**
* @brief Logs an integer.
* @param inModule The module associated with this log statement.
* @param inFuncName The function name from which we are logging.
* @param inSeverity The severity of this log statement.
* @param inNum Integer to log.
*/
void log_integer(LogModule_t inModule, const char* inFuncName, LogSeverity_t inSeverity, uint64_t inNum);

/**
* @brief A wrapper for log integer that include the function name by default.
*/
#define LOG_INTEGER(category, severity, num) \
{ \
log_integer(category, __func__, severity, num); \
}

#endif
/*
* @file post.h
* @brief Project 4
*
* A power on self test.
*
* @author Jack Campbell
* @tools  PC Compiler: GNU gcc 8.3.0
*         PC Linker: GNU ld 2.32
*         PC Debugger: GNU gdb 8.2.91.20190405-git
*         ARM Compiler: GNU gcc version 8.2.1 20181213
*         ARM Linker: GNU ld 2.31.51.20181213
*         ARM Debugger: GNU gdb 8.2.50.20181213-git
*/

#ifndef POSTH
#define POSTH

#include <stdbool.h>

/**
* @brief Power on self test that checks for connection with TMP102.
* @return Whether the test succeeded.
*/
bool power_on_self_test();

#endif
/*
* @file setup_teardown.h
* @brief Project 4
*
* @details Contains the setup and cleanup prototypes to be implemented
*          both for the FB and PC variants of the build.
*
* @tools  PC Compiler: GNU gcc 8.3.0
*         PC Linker: GNU ld 2.32
*         PC Debugger: GNU gdb 8.2.91.20190405-git
*         ARM Compiler: GNU gcc version 8.2.1 20181213
*         ARM Linker: GNU ld 2.31.51.20181213
*         ARM Debugger: GNU gdb 8.2.50.20181213-git
*/

#ifndef PES_PROJECT_4_SETUP_TEARDOWN_H
#define PES_PROJECT_4_SETUP_TEARDOWN_H

/**
* initialize
*
* @details Initializes components needed by a particular platform,
*          such as LEDs.
*
*/
void initialize(void);


/**
* terminate
*
* @details Cleans up any required components on a particular platform.
*
*/
void terminate(void);

#endif //PES_PROJECT_2_SETUP_TEARDOWN_H
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
*/

#ifndef STATEMACHINE
#define STATEMACHINE

/**
* @brief Set of states available in the state machine.
*/
typedef enum  {
STATE_TEMP_READING,
STATE_AVERAGE_WAIT,
STATE_TEMP_ALERT,
STATE_DISCONNECTED,
STATE_NUM_STATES
} State_t;

/**
* @brief Set of events that must be handled by the state machine.
*/
typedef enum {
EVENT_TIMEOUT,
EVENT_COMPLETE,
EVENT_ALERT,
EVENT_DISCONNECT,
EVENT_NUM_EVENTS
} Event_t;

/**
* @brief The state object for the state machine.
*/
struct StateMachine_s
{
State_t state;
int8_t timeout;
void (*eventHandler)(struct StateMachine_s*, Event_t);
};

/**
* @brief Typedef for easier usage.
*/
typedef struct StateMachine_s StateMachine;

/**
* @brief Handles event using a state-based approach.
* @param inState Current state machine object.
* @param inEvent Event to handle.
*/
void handle_event_state(StateMachine* inState, Event_t inEvent);

/**
* @brief Handles event using a table-based approach.
* @param inState Current state machine object.
* @param inEvent Event to handle.
*/
void handle_event_table(StateMachine* inState, Event_t inEvent);

#endif
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

#ifndef TMP102_H
#define TMP102_H

#include <stdbool.h>

/**
* @brief Returns the current temperature in Celsius.
*/
float readTempC();

/**
* @return T_LOW register in Celsius.
*/
float readLowTempC();

/**
* @return T_HIGH register in Celsius.
*/
float readHighTempC();

/**
* @brief
* @return Returns the state of the Alert register. The state of the register is the same as the ALT pin.
*/
bool alert();

/**
* @brief Sets T_LOW (in Celsius) alert threshold.
* @param temperature Temp to set.
*/
void setLowTempC(float temperature);

/**
* @brief Sets T_HIGH (in Celsius) alert threshold.
* @param temperature Temp to set.
*/
void setHighTempC(float temperature);

/**
* @brief Sets the type of alert.
* @param mode 0: Comparator Mode (Active from when temperature > T_HIGH until temperature < T_LOW),
*             1: Thermostat mode (Active from when temperature > T_HIGH until any read operation occurs.
*/
void setAlertMode(bool mode);

/**
* @brief
* @return Returns whether the tmp102 is connected.
*/
bool tmp102_connected();

#endif
/*
* @file delay.c
* @brief Project 4
*
* @details This file contains prototypes for calculating a spin-wait
*          on the FB, used for delaying LED state changes.
*
* @author Jack Campbell
* @tools  PC Compiler: GNU gcc 8.3.0
*         PC Linker: GNU ld 2.32
*         PC Debugger: GNU gdb 8.2.91.20190405-git
*         ARM Compiler: GNU gcc version 8.2.1 20181213
*         ARM Linker: GNU ld 2.31.51.20181213
*         ARM Debugger: GNU gdb 8.2.50.20181213-git
*/
#include "delay.h"
#include "fsl_debug_console.h"

/* GLOBALS */
const uint64_t CLOCKS_PER_MILLISECOND = 2600UL;

/**
* delay
*
* @brief Blocks execution for the specified time.
* @param inDelayMs Then time in milliseconds to block.
*/
void delay(uint64_t inDelayMs)
{
volatile uint64_t number = inDelayMs * CLOCKS_PER_MILLISECOND;

while(number--)
{
__asm volatile ("nop");
}
}
/*
* @file handle_led.c
* @brief Project 4
*
* Functions for handling the state of an LED.
*
* @tools  PC Compiler: GNU gcc 8.3.0
*         PC Linker: GNU ld 2.32
*         PC Debugger: GNU gdb 8.2.91.20190405-git
*         ARM Compiler: GNU gcc version 8.2.1 20181213
*         ARM Linker: GNU ld 2.31.51.20181213
*         ARM Debugger: GNU gdb 8.2.50.20181213-git
*/

#include <stdint.h>
#include "handle_led.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "fsl_gpio.h"
#include "MKL25Z4.h"
#include "logger.h"

/**
* set_led
*
* @brief Sets the LED state.
* @details This function controls a physical LED and prints
*          debug info over UART on debug builds.
* @param inValue The on/off state of the LED to set.
* @param inColor The color of the LED to set.
*/
void set_led(uint8_t inValue, enum COLOR inColor)
{
LOG_STRING_ARGS(LOG_MODULE_LED,
LOG_SEVERITY_TEST,
"LED %s %s", COLOR_STRINGS[inColor],
inValue ? "ON" : "OFF");

switch(inColor)
{
case RED:
{
LED_BLUE_OFF();
LED_GREEN_OFF();
if(inValue)
{
LED_RED_ON();
}
else
{
LED_RED_OFF();
}

break;
}
case GREEN:
{
LED_BLUE_OFF();
LED_RED_OFF();

if(inValue)
{
LED_GREEN_ON();
}
else
{
LED_GREEN_OFF();
}
break;
}
case BLUE:
{
LED_GREEN_OFF();
LED_RED_OFF();

if(inValue)
{
LED_BLUE_ON();
}
else
{
LED_BLUE_OFF();
}
break;
}
default:
break;
}
}
/*
* @file i2c.h
* @brief Project 4
*
* @details Handles I2C transactions.
*
* @author Jack Campbell
* @tools  PC Compiler: GNU gcc 8.3.0
*         PC Linker: GNU ld 2.32
*         PC Debugger: GNU gdb 8.2.91.20190405-git
*         ARM Compiler: GNU gcc version 8.2.1 20181213
*         ARM Linker: GNU ld 2.31.51.20181213
*         ARM Debugger: GNU gdb 8.2.50.20181213-git
*
*  Attribution: Much of this code taken from from Embedded Systems Fundamentals
*  with Arm Cortex-M based Microcontrollers, Dean, 2017, ARM Education Media
*  (The word doc supplied by Prof. Montgomery)
*/

#include "i2c.h"
#include "MKL25Z4.h"
#include "logger.h"
#include "delay.h"

/**
* @brief Send a start bit using the control register.
*/
#define I2C_M_START I2C1->C1 |= I2C_C1_MST_MASK

/**
* @brief Send a stop bit using the control register.
*/
#define I2C_M_STOP I2C1->C1 &= ~I2C_C1_MST_MASK

/**
* @brief Send a repeated start bit using the control register.
*/
#define I2C_M_RSTART I2C1->C1 |= I2C_C1_RSTA_MASK

/**
* @brief Use control register to change to transmit mode.
*/
#define I2C_TX I2C1->C1 |= I2C_C1_TX_MASK

/**
* @brief Use the control register to change to receiver mode.
*/
#define I2C_RX I2C1->C1 &= ~I2C_C1_TX_MASK

/**
* @brief Spin until the interrupt flag has been set.
*/
#define I2C_WAIT while((I2C1->S & I2C_S_IICIF_MASK)==0) {} \
I2C1->S |= I2C_S_IICIF_MASK;

/**
* @brief Send a no-acknowledge bit after this operation.
*/
#define NACK I2C1->C1 |= I2C_C1_TXAK_MASK

/**
* @brief Send an acknowledge bit after this operation.
*/
#define ACK I2C1->C1 &= ~I2C_C1_TXAK_MASK

/**
* Checks for arbitration lost or no ACK received from device.
*/
static bool communicationErrorOccured()
{
// check if arbitration was lost
if((I2C1->S & I2C_S_ARBL_MASK))
{
LOG_STRING(LOG_MODULE_I2C,
LOG_SEVERITY_STATUS,
"Arbitration lost. Aborting read.");
I2C1->S |= I2C_S_ARBL_MASK;
I2C_M_STOP; //send stop
return true;
}

if(!(I2C1->S & I2C_S_RXAK_MASK))
{
LOG_STRING(LOG_MODULE_I2C,
LOG_SEVERITY_STATUS,
"No ACK received. Aborting read.");
I2C_M_STOP; //send stop
return true;
}
return false;
}

void i2c_init(void)
{
/* Enable clock for I2C1 module */
SIM->SCGC4 |= SIM_SCGC4_I2C1_MASK;

/* Enable clock for Port E */
SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK;

/* Port E MUX configuration */
PORTE->PCR[24] |=  PORT_PCR_MUX(5);
PORTE->PCR[25] |=  PORT_PCR_MUX(5);

/* Configure Divider Register */
I2C1->F |= I2C_F_ICR(0x11) | I2C_F_MULT(0);

/* Enable I2C module and interrupt */
I2C1->C1 |= I2C_C1_IICEN_MASK;
}

void i2c_write_byte(uint8_t dev, uint8_t reg, uint8_t data)
{
// poll until not busy
while((I2C1->S & I2C_S_BUSY_MASK)!=0) {}

I2C_TX; // set to transmit mode
I2C_M_START; //send start
I2C1->D = dev; // send dev address (write)
I2C_WAIT // wait for ack

if(communicationErrorOccured()){ return; }

I2C1->D = reg; // send register address
I2C_WAIT

if(communicationErrorOccured()){ return; }

I2C1->D = data; //send data
I2C_WAIT
if(communicationErrorOccured()){ return; }

I2C_M_STOP;
}

void i2c_write_bytes(uint8_t inSlaveAddr, uint8_t inRegAddr, uint8_t* inData, int8_t inDataSize)
{
int8_t num_bytes_written = 0;
I2C_TX; //set to transmit mode
I2C_M_START; //send start
I2C1->D = inSlaveAddr;
I2C_WAIT

I2C1->D = inRegAddr; //send register address
I2C_WAIT

do {
ACK;
I2C1->D = inData[num_bytes_written++];
I2C_WAIT
} while (num_bytes_written < inDataSize-2);

NACK; // to HW to send NACK after read
inData[num_bytes_written++] = I2C1->D; // read data
I2C_WAIT
I2C_M_STOP; // send stop
}

uint8_t i2c_read_byte(uint8_t dev, uint8_t reg)
{
uint8_t data;

// poll until not busy
while((I2C1->S & I2C_S_BUSY_MASK)!=0) {}

I2C_TX; // set to transmit mode
I2C_M_START; //send start
I2C1->D = dev; // send dev address (write)
I2C_WAIT
if(communicationErrorOccured()){ return 0x0; }

I2C1->D = reg; // send register address
I2C_WAIT
if(communicationErrorOccured()){ return 0x0; }

I2C_M_RSTART; // repeated start
I2C1->D = (dev|0x1); // send dev address (read)
I2C_WAIT
if(communicationErrorOccured()){ return 0x0; }

I2C_RX; // set to receive mode

NACK; // set NACK after read
data = I2C1->D; // dummy read
I2C_WAIT

I2C_M_STOP; //send stop
data = I2C1->D; // read data

return data;
}

void i2c_read_bytes(uint8_t inSlaveAddr, uint8_t inRegAddr, uint8_t* outData, int8_t outDataSize)
{
// poll until not busy
while((I2C1->S & I2C_S_BUSY_MASK)!=0) {}

I2C_TX; //set to transmit mode
I2C_M_START; //send start
I2C1->D = inSlaveAddr;
I2C_WAIT

if(communicationErrorOccured()){ return; }

I2C1->D = inRegAddr; //send register address
I2C_WAIT

if(communicationErrorOccured()){ return; }

I2C_M_RSTART; //repeated start

I2C1->D = inSlaveAddr|0x01; //send dev address (read)
I2C_WAIT

if(!(I2C1->S & I2C_S_RXAK_MASK))
{
LOG_STRING(LOG_MODULE_I2C,
LOG_SEVERITY_STATUS,
"No ACK received. Aborting read.");
return;
}

I2C_RX; //set to receive mode
ACK; // tell hardware to send ack after read
outData[0] = I2C1->D; // dummy read to start I2C read
I2C_WAIT

ACK;
outData[0] = I2C1->D;
I2C_WAIT

NACK; // to HW to send nack after read
outData[1] = I2C1->D; // read data
I2C_WAIT

if(communicationErrorOccured()){ return; }

I2C_M_STOP; // send stop
}

bool i2c_connected(uint8_t inSlaveAddr)
{
I2C_TX; // set to transmit mode
I2C_M_START; //send start
I2C1->D = inSlaveAddr; // send dev address (write)
I2C_WAIT

bool disconnected = communicationErrorOccured();

I2C_M_STOP; // send stop

return !disconnected;
}

/*
* @file logger.h
* @brief Project 5
*
* Tools for logging.
*
* @author Jack Campbell
* @tools  PC Compiler: GNU gcc 8.3.0
*         PC Linker: GNU ld 2.32
*         PC Debugger: GNU gdb 8.2.91.20190405-git
*         ARM Compiler: GNU gcc version 8.2.1 20181213
*         ARM Linker: GNU ld 2.31.51.20181213
*         ARM Debugger: GNU gdb 8.2.50.20181213-git
*/

#include "logger.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdint.h>

#include <stdio.h>
#include <stdarg.h>

#include "fsl_debug_console.h"

/**
* Used as a size for static char arrays.
*/
#define ARRLEN 2048

/**
* @brief Used to standardize the prefix before log messages.
*/
#define PRINT_LOG_PREFIX(module, func, severity)\
PRINTF("\n\n%s -> %s::[%s] : ",  sLogSeverityStrings[severity] , sLogModuleStrings[module], func );

/**
* @brief Strings associated with severities.
*/
static const char* sLogSeverityStrings[NUM_LOG_SEVERITIES] =
{
"TEST",
"DEBUG",
"STATUS"
};

/**
* @brief Strings associated with modules.
*/
static const char* sLogModuleStrings[NUM_LOG_MODULES] =
{
"MAIN",
"LED",
"UNIT_TEST",
"SETUP_TEARDOWN",
"STATE_MACHINE_STATE",
"STATE_MACHINE_TABLE",
"POST",
"TMP102",
"I2C"
};

/**
* @brief Static variable maintains the logging state.
*/
static bool sLoggingEnabled = false;

/**
* @brief Static severity maintains the severity for the module.
*/
static LogSeverity_t sLogSeverity = LOG_SEVERITY_STATUS;

void log_enable(LogSeverity_t inSeverity)
{
sLoggingEnabled = true;
sLogSeverity = inSeverity;
}

void log_disable()
{
sLoggingEnabled = false;
}

bool log_enabled()
{
return sLoggingEnabled;
}

void log_data(LogModule_t inModule, const char* inFuncName, LogSeverity_t inSeverity, const uint8_t* inBytes, size_t inSize)
{
if (sLoggingEnabled && inSeverity >= sLogSeverity)
{
PRINT_LOG_PREFIX(inModule, inFuncName, inSeverity)
PRINTF("\n\rBytes at address %p:\n\r==========================\n\r", inBytes);
for(int i = 0; i < inSize; i++)
{
PRINTF("%2x ", inBytes[i]);
if((i+1)%4 == 0)
{
PRINTF("\r\n");
}
}
printf("\n\r==========================\n\r");
}
}


void log_string(LogModule_t inModule, const char* inFuncName, LogSeverity_t inSeverity, const char* inString, ...)
{
static char format_buf[ARRLEN] = {0};
for(int i = 0; i < 2048; i++) format_buf[i] = '\0';

if (sLoggingEnabled && inSeverity >= sLogSeverity) {

va_list argp;
va_start(argp, inString);
vsprintf(format_buf, inString, argp);
va_end(argp);
PRINT_LOG_PREFIX(inModule, inFuncName, inSeverity)
PRINTF("%s\n\r", format_buf);
}
}


void log_integer(LogModule_t inModule, const char* inFuncName, LogSeverity_t inSeverity, uint64_t inNum)
{
if (sLoggingEnabled && inSeverity >= sLogSeverity)
{
PRINT_LOG_PREFIX(inModule, inFuncName, inSeverity)
PRINTF("%lld\n\r", inNum);
}
}
/*
* @file main.c
* @brief Project 4
*
* @details Main controller for the state machine.
*
* @author Jack Campbell
* @tools  PC Compiler: GNU gcc 8.3.0
*         PC Linker: GNU ld 2.32
*         PC Debugger: GNU gdb 8.2.91.20190405-git
*         ARM Compiler: GNU gcc version 8.2.1 20181213
*         ARM Linker: GNU ld 2.31.51.20181213
*         ARM Debugger: GNU gdb 8.2.50.20181213-git
*/
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL25Z4.h"
#include "fsl_debug_console.h"

#include "handle_led.h"
#include "setup_teardown.h"
#include "logger.h"
#include "state_machine.h"
#include "post.h"
#include "tmp102.h"
#include "delay.h"

int main(void) {

initialize();
StateMachine stateMachine = {STATE_TEMP_READING, 0, &handle_event_state};
stateMachine.eventHandler(&stateMachine, EVENT_COMPLETE);

while(1)
{
if(stateMachine.state == STATE_DISCONNECTED)
{
break;
}
else if(!tmp102_connected())
{
stateMachine.eventHandler(&stateMachine, EVENT_DISCONNECT);
}
else if(alert())
{
stateMachine.eventHandler(&stateMachine, EVENT_ALERT);
}
else if(stateMachine.state == STATE_TEMP_READING)
{
stateMachine.eventHandler(&stateMachine, EVENT_COMPLETE);
}
else
{
delay(2000);
stateMachine.eventHandler(&stateMachine, EVENT_TIMEOUT);
}
}

return 0 ;
}

#include "handle_led.h"
#include "post.h"
#include "logger.h"
#include "MKL25Z4.h"
#include "tmp102.h"

#include "delay.h"

bool power_on_self_test()
{
LOG_STRING( LOG_MODULE_POST, LOG_SEVERITY_DEBUG, "Starting power-on self test." );
set_led(1, RED);
set_led(1, GREEN);
set_led(1, BLUE);

return tmp102_connected();
}

/*
* @file setup_teardown.h
* @brief Project 4
*
* @details Contains the setup and cleanup prototypes.
*
* @tools  PC Compiler: GNU gcc 8.3.0
*         PC Linker: GNU ld 2.32
*         PC Debugger: GNU gdb 8.2.91.20190405-git
*         ARM Compiler: GNU gcc version 8.2.1 20181213
*         ARM Linker: GNU ld 2.31.51.20181213
*         ARM Debugger: GNU gdb 8.2.50.20181213-git
*/

#include "board.h"
#include "peripherals.h"
#include "clock_config.h"
#include "pin_mux.h"
#include "post.h"
#include "logger.h"
#include "stdlib.h"
#include "i2c.h"

void initialize()
{
/* Init board hardware. */
BOARD_InitBootPins();
BOARD_InitBootClocks();
BOARD_InitBootPeripherals();

#ifdef DEBUG
BOARD_InitDebugConsole();
log_enable(LOG_SEVERITY_TEST);
LOG_STRING(LOG_MODULE_SETUP_TEARDOWN, LOG_SEVERITY_DEBUG, "program start");
#else
log_enable(SEVERITY_STATUS);
#endif

/* led setup */
LED_RED_INIT(1);
LED_BLUE_INIT(1);
LED_GREEN_INIT(1);

LED_BLUE_OFF();
LED_GREEN_OFF();
LED_RED_OFF();

i2c_init();

if(!power_on_self_test())
exit(-1);
}

/**
* terminate
*
* @details Print "program end" in debug builds.
*          Shows that the program successfully completed.
*
*/
void terminate()
{
#ifdef DEBUG
LOG_STRING(LOG_MODULE_SETUP_TEARDOWN, LOG_SEVERITY_DEBUG, "program end");
#endif
}
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
*  Leveraged code: Table based state machine was inspired by
*  https://yakking.branchable.com/posts/state-machines-in-c/table.c
*/

#include "logger.h"
#include "state_machine.h"
#include "handle_led.h"
#include "tmp102.h"
#include <stdlib.h>
#include "delay.h"


/**
* @brief Strings for states.
*/
static const char* sStateStrings[STATE_NUM_STATES] =
{
"STATE_TEMP_READING",
"STATE_AVERAGE_WAIT",
"STATE_TEMP_ALERT",
"STATE_DISCONNECTED"
};

/**
* @brief Strings for events.
*/
static const char* sEventStrings[EVENT_NUM_EVENTS] =
{
"EVENT_TIMEOUT",
"EVENT_COMPLETE",
"EVENT_ALERT",
"EVENT_DISCONNECT"
};

#define MAX_TIMEOUTS 4

static float sCurrentTempReading = 0.0;
static float sCumulativeTempReading = 0.0;
static uint64_t sNumTempReadings = 0;

void read_temp()
{
set_led(1, GREEN);
LOG_STRING( LOG_MODULE_STATE_MACHINE_STATE, LOG_SEVERITY_STATUS, "Reading TMP102 over I2C" );
sNumTempReadings++;
sCurrentTempReading = readTempC();
delay(1000); // want to reduce the sample rate for temp reads
}

void average_wait()
{
set_led(1, GREEN);
LOG_STRING( LOG_MODULE_STATE_MACHINE_STATE, LOG_SEVERITY_DEBUG, "Using last read temp to calculate an average." );
sCumulativeTempReading += sCurrentTempReading;
LOG_STRING_ARGS( LOG_MODULE_STATE_MACHINE_STATE,
LOG_SEVERITY_STATUS,
"Current temp: { %f }, Average temp: { %f }",
sCurrentTempReading,
sCumulativeTempReading / sNumTempReadings );
}

void temp_alert()
{
set_led(1, BLUE);
LOG_STRING( LOG_MODULE_STATE_MACHINE_STATE, LOG_SEVERITY_STATUS, "ALERT Temp value was negative." );
LOG_STRING( LOG_MODULE_STATE_MACHINE_STATE, LOG_SEVERITY_STATUS, "Reading TMP102 over I2C" );
read_temp();
}

void handle_event_state(StateMachine* inState, Event_t inEvent)
{
switch(inState->state)
{
case STATE_TEMP_READING:
{
read_temp();
switch(inEvent)
{
case EVENT_COMPLETE:
{
inState->state = STATE_AVERAGE_WAIT;
break;
}
case EVENT_ALERT:
{
inState->state = STATE_TEMP_ALERT;
break;
}
case EVENT_DISCONNECT:
{
inState->state = STATE_DISCONNECTED;
goto disconnected_error;
break;
}
default:
break;
}
break;
}
case STATE_AVERAGE_WAIT:
{
average_wait();
switch(inEvent)
{
case EVENT_TIMEOUT:
{
if(++inState->timeout < MAX_TIMEOUTS)
{
inState->state = STATE_TEMP_READING;
LOG_STRING_ARGS( LOG_MODULE_STATE_MACHINE_STATE, LOG_SEVERITY_TEST, "Timeout val: %d", inState->timeout );
}
else
{
inState->timeout = 0;
LOG_STRING( LOG_MODULE_STATE_MACHINE_STATE, LOG_SEVERITY_STATUS, "Transition to table state machine." );
inState->state = STATE_TEMP_READING;
inState->eventHandler = handle_event_table;
}
break;
}
case EVENT_DISCONNECT:
{
inState->state = STATE_DISCONNECTED;
goto disconnected_error;
break;
}
default:
break;
}
break;
}
case STATE_TEMP_ALERT:
{
temp_alert();
switch(inEvent)
{
case EVENT_COMPLETE:
{
inState->state = STATE_AVERAGE_WAIT;
break;
}
case EVENT_DISCONNECT:
{
inState->state = STATE_DISCONNECTED;
goto disconnected_error;
break;
}
default:
break;
}
break;
}
case STATE_DISCONNECTED:
{
goto disconnected_error;
break;
}
default:
inState->state = STATE_TEMP_READING;
break;
}

LOG_STRING_ARGS(LOG_MODULE_STATE_MACHINE_STATE,
LOG_SEVERITY_DEBUG,
"Handling [%s] event. New state is [%s].",
sEventStrings[inEvent],
sStateStrings[inState->state]);

return;

disconnected_error:
LOG_STRING( LOG_MODULE_STATE_MACHINE_STATE, LOG_SEVERITY_STATUS, "Got disconnected event. Exiting." );
exit(-1);
}

void temp_reading_handle_complete(StateMachine* inState)
{
read_temp();
inState->state = STATE_AVERAGE_WAIT;
}

void temp_reading_handle_alert(StateMachine* inState)
{
read_temp();
inState->state = STATE_TEMP_ALERT;
}

void temp_reading_handle_disconnect(StateMachine* inState)
{
read_temp();
inState->state = STATE_DISCONNECTED;
}

void temp_alert_handle_complete(StateMachine* inState)
{
temp_alert();
inState->state = STATE_AVERAGE_WAIT;
}

void temp_alert_handle_disconnect(StateMachine* inState)
{
temp_alert();
inState->state = STATE_DISCONNECTED;
}

void avg_wait_handle_timeout(StateMachine* inState)
{
average_wait();
if(++inState->timeout < MAX_TIMEOUTS)
{
inState->state = STATE_TEMP_READING;
LOG_STRING_ARGS( LOG_MODULE_STATE_MACHINE_TABLE, LOG_SEVERITY_TEST, "Timeout val: %d", inState->timeout );
}
else
{
inState->state = STATE_TEMP_READING;
inState->timeout = 0;
LOG_STRING( LOG_MODULE_STATE_MACHINE_TABLE, LOG_SEVERITY_TEST, "Transition to state-based state machine." );
inState->eventHandler = handle_event_state;
}
}

void avg_wait_handle_disconnect(StateMachine* inState)
{
average_wait();
inState->state = STATE_DISCONNECTED;
}

// make a table of state handlers, mapped to events x states
typedef void (*transition_handler)(StateMachine*);

transition_handler transitions[STATE_NUM_STATES][EVENT_NUM_EVENTS] =
{    /* Timeout */                   /* Complete */                    /* Alert */                    /* Disconnect*/
{   NULL,                    temp_reading_handle_complete, temp_reading_handle_alert, temp_reading_handle_disconnect    }, /* Temp Reading */
{   avg_wait_handle_timeout, NULL,                         NULL,                      avg_wait_handle_disconnect       }, /* Average Wait */
{   NULL,                    temp_alert_handle_complete,   NULL,                      temp_alert_handle_disconnect      }, /* Temp Alert   */
{   NULL,                    NULL,                         NULL,                      NULL                              }  /* Disconnected */
};

void handle_event_table(StateMachine* inState, Event_t inEvent)
{
transition_handler handler = transitions[inState->state][inEvent];
if (handler)
{
handler(inState);
LOG_STRING_ARGS(LOG_MODULE_STATE_MACHINE_TABLE,
LOG_SEVERITY_DEBUG,
"Handling [%s] event. New state is [%s].",
sEventStrings[inEvent],
sStateStrings[inState->state]);
}
else
{
LOG_STRING( LOG_MODULE_STATE_MACHINE_TABLE, LOG_SEVERITY_STATUS, "State transition NULL." );
}
}
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

/**
* Comment this to use the mock TMP102 implementation.
*/
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
uint8_t registerByte[2]= {0,0};    // Store the data from the register here
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
uint8_t registerByte[2];    // Store the data from the register here
int16_t digitalTemp;        // Store the digital temperature value here

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
uint8_t registerByte[2];    // Store the data from the register here
int16_t digitalTemp;        // Store the digital temperature value here

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
//      // Change pointer address to configuration register (1)
//      uint8_t registerByte = i2c_read_byte(TMP102_PERIPHERAL_ADDR, CONFIG_REGISTER);
//
//      registerByte &= 0x20;    // Clear everything but the alert bit (bit 5)
//      return registerByte>>5;
}

// - Sets T_LOW (in Celsius) alert threshold.
void setLowTempC(float temperature)
{
uint8_t registerByte[2];    // Store the data from the register here
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
uint8_t registerByte[2];    // Store the data from the register here
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

registerByte &= 0xFD;    // Clear old TM bit (bit 1 of first byte)
registerByte |= mode<<1;    // Shift in new TM bit

delay(300); // wait before you read...

i2c_write_byte(TMP102_PERIPHERAL_ADDR, CONFIG_REGISTER, registerByte);
}

bool tmp102_connected()
{
//return i2c_connected(TMP102_PERIPHERAL_ADDR);
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

```

