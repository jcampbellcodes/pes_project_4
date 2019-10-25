/*
 * @file setup_teardown.h
 * @brief Project 3
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

#include "logger.h"

void initialize()
{
#ifdef DEBUG
	/* serial debug console setup: use PRINTF("debug msg"); */
	BOARD_InitDebugConsole();
	log_enable();
	log_string("\nprogram start\n");
#endif

    /* Board pin, clock, debug console init */
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitBootPeripherals();

	/* led setup */
	LED_RED_INIT(1);
	LED_BLUE_INIT(1);
	LED_GREEN_INIT(1);

	LED_BLUE_OFF();
	LED_GREEN_OFF();
	LED_RED_OFF();
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
	log_string("\nprogram end\n");
#endif
}
