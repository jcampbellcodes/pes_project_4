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
