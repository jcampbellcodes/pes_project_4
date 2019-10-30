#include "logger.h"
#include "state_machine.h"

#define MAX_TIMEOUTS 4

void read_temp()
{
	LOG_STRING( LOG_MODULE_STATE_MACHINE_STATE, LOG_SEVERITY_STATUS, "Reading TMP102 over I2C" );
}

void average_wait()
{
	/*
	 * use the last temperature reading to update an
	 * average temperature value.  Normal status
	 * messages should print the last reading and
	 * the current average.  You will then wait 15
	 * seconds for a timeout.
	 *
	 * disable all interrupts (alert) whilst in this state
	 */
	LOG_STRING( LOG_MODULE_STATE_MACHINE_STATE, LOG_SEVERITY_STATUS, "Using last read temp to calculate an average." );
	LOG_STRING( LOG_MODULE_STATE_MACHINE_STATE, LOG_SEVERITY_STATUS, "Wait 15 seconds." );
}

void temp_alert()
{
	LOG_STRING( LOG_MODULE_STATE_MACHINE_STATE, LOG_SEVERITY_STATUS, "ALERT Temp value was negative." );
	LOG_STRING( LOG_MODULE_STATE_MACHINE_STATE, LOG_SEVERITY_STATUS, "Reading TMP102 over I2C" );
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
					break;
				}
				default:
					break;
			}
			break;
		}
		case STATE_DISCONNECTED:
		{
			// TODO: Anything? Or can this case be removed?
			break;
		}
		default:
			inState->state = STATE_TEMP_READING;
			break;
	}
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

// https://yakking.branchable.com/posts/state-machines-in-c/table.c
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
	}
	else
	{
		LOG_STRING( LOG_MODULE_STATE_MACHINE_TABLE, LOG_SEVERITY_STATUS, "State transition NULL, exiting program." );
		// TODO: Exit?
	}
}
