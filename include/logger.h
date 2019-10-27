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
#include <stdint.h>

typedef enum LogSeverity
{
	LOG_SEVERITY_TEST,
	LOG_SEVERITY_DEBUG,
	LOG_SEVERITY_STATUS,
	NUM_LOG_SEVERITIES
} LogSeverity_t;

typedef enum LogModule
{
	LOG_MODULE_MAIN,
	LOG_MODULE_LED,
	LOG_MODULE_UNIT_TEST,
	LOG_MODULE_SETUP_TEARDOWN,
	LOG_MODULE_STATE_MACHINE_STATE,
	LOG_MODULE_STATE_MACHINE_TABLE,
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
 * @param inBytes a pointer to a sequence of bytes to log
 * @param inSize Number of bytes to log
 */
void log_data(LogModule_t inModule, const char* inFuncName, LogSeverity_t inSeverity, const uint8_t* inBytes, size_t inSize);

#define LOG_DATA(category, severity, data, length) \
{ \
    log_data(category, __FUNCTION__, severity, data, length); \
}


/**
 * @brief Display a string.
 * @param inString String to display.
 */
void log_string(LogModule_t inModule, const char* inFuncName, LogSeverity_t inSeverity, const char* inString, ...);

#define LOG_STRING_ARGS(category, severity, fmt, ...) \
{ \
	log_string(category, __func__, severity, fmt, __VA_ARGS__); \
}

#define LOG_STRING(category, severity, fmt) \
{ \
	log_string(category, __func__, severity, fmt); \
}

/**
 * @brief Display an integer
 * @param inNum Integer to display.
 */
void log_integer(LogModule_t inModule, const char* inFuncName, LogSeverity_t inSeverity, uint64_t inNum);

#define LOG_INTEGER(category, severity, num) \
{ \
	log_integer(category, __func__, severity, num); \
}

#endif
