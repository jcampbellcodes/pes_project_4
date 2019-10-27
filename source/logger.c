/*
 * @file logger.h
 * @brief Project 3
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

#define PRINT_LOG_PREFIX(module, func, severity)\
	PRINTF("\n%s -> %s::[%s] : ",  sLogSeverityStrings[severity] , sLogModuleStrings[module], func );

static const char* sLogSeverityStrings[NUM_LOG_SEVERITIES] =
{
	"LOG_SEVERITY_TEST",
	"LOG_SEVERITY_DEBUG",
	"LOG_SEVERITY_STATUS"
};

static const char* sLogModuleStrings[NUM_LOG_MODULES] =
{
	"LOG_MODULE_MAIN",
	"LOG_MODULE_LED",
	"LOG_MODULE_UNIT_TEST",
	"LOG_MODULE_SETUP_TEARDOWN",
	"LOG_MODULE_STATE_MACHINE_STATE",
	"LOG_MODULE_STATE_MACHINE_TABLE",
	"LOG_MODULE_TMP102",
	"LOG_MODULE_I2C"
};

/**
 * @brief Static variable maintains the logging state.
 */
static bool sLoggingEnabled = false;

static LogSeverity_t sLogSeverity = LOG_SEVERITY_STATUS;

/**
 * @brief Log_enable – begin printing log messages when called
 */
void log_enable(LogSeverity_t inSeverity)
{
	sLoggingEnabled = true;
	sLogSeverity = inSeverity;
}

/**
 * @brief Log_disable – ignore any log messages until re-enabled
 */
void log_disable()
{
	sLoggingEnabled = false;
}

/**
 * @brief Log_status – returns a flag to indicate whether the logger is enabled or disabled
 * @return Whether the log is currently enabled.
 */
bool log_enabled()
{
	return sLoggingEnabled;
}

/**
 * @brief Log_data – display in hexadecimal an address and contents of a memory location,
 * @param inBytes a pointer to a sequence of bytes to log
 * @param inSize Number of bytes to log
 */
void log_data(LogModule_t inModule, const char* inFuncName, LogSeverity_t inSeverity, const uint8_t* inBytes, size_t inSize)
{
	if (sLoggingEnabled && inSeverity <= sLogSeverity)
	{
		PRINT_LOG_PREFIX(inModule, inFuncName, inSeverity)
		PRINTF("\nBytes at address %p:\n==========================\n", inBytes);
		for(int i = 0; i < inSize; i++)
		{
			PRINTF("%2x ", inBytes[i]);
			if((i+1)%4 == 0)
			{
				PRINTF("\n");
			}
		}
		printf("\n==========================\n");
	}
}

/**
 * @brief Display a string.
 * @param inString String to display.
 */
void log_string(LogModule_t inModule, const char* inFuncName, LogSeverity_t inSeverity, const char* inString, ...)
{
	// TODO: no magic numbers
	static char format_buf[2048] = {0};

	if (sLoggingEnabled && inSeverity <= sLogSeverity) {

	    va_list argp;
	    va_start(argp, inString);
	    vsprintf(format_buf, inString, argp);
	    va_end(argp);
	    PRINT_LOG_PREFIX(inModule, inFuncName, inSeverity)
		PRINTF("%s\n", format_buf);
	}
}

/**
 * @brief Display an integer
 * @param inNum Integer to display.
 */
void log_integer(LogModule_t inModule, const char* inFuncName, LogSeverity_t inSeverity, uint64_t inNum)
{
	if (sLoggingEnabled && inSeverity <= sLogSeverity)
	{
		PRINT_LOG_PREFIX(inModule, inFuncName, inSeverity)
		PRINTF("%llu\n", inNum);
	}
}
