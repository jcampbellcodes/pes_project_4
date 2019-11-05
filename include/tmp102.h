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
