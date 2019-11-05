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
 
/**
 * @file    pesproject4.c
 * @brief   Application entry point.
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

int main(void) {

	initialize();
	StateMachine stateMachine = {STATE_TEMP_READING, 0, handle_event_state};
	stateMachine.eventHandler(&stateMachine, EVENT_COMPLETE);

	while(1)
	{
		// listen for events

		// check for timeout
		// checkout for disconnected
		// check for alert
	}

    return 0 ;
}
