/*
 * @file logger.h
 * @brief Project 3
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


/**
 * @brief Log_enable – begin printing log messages when called
 */
void log_enable();

/**
 * @brief Log_disable – ignore any log messages until re-enabled
 */
void log_disable();

/**
 * @brief Log_status – returns a flag to indicate whether the logger is enabled or disabled
 * @return Whether the log is currently enabled.
 */
bool log_enabled();


/**
 * @brief Log_data – display in hexadecimal an address and contents of a memory location,
 * @param inBytes a pointer to a sequence of bytes to log
 * @param inSize Number of bytes to log
 */
void log_data(const uint8_t* inBytes, size_t inSize);

/**
 * @brief Display a string.
 * @param inString String to display.
 */
void log_string(const char* inString);

/**
 * @brief Display an integer
 * @param inNum Integer to display.
 */
void log_integer(uint64_t inNum);

#endif
