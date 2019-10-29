#ifndef STATEMACHINE
#define STATEMACHINE

#define MAX_TIMEOUTS 4

// states, starting state
typedef enum  {
	STATE_TEMP_READING,
	STATE_AVERAGE_WAIT,
	STATE_TEMP_ALERT,
	STATE_DISCONNECTED,
	STATE_NUM_STATES
} State_t;

// events
typedef enum {
	EVENT_TIMEOUT,
	EVENT_COMPLETE,
	EVENT_ALERT,
	EVENT_DISCONNECT,
	EVENT_NUM_EVENTS
} Event_t;

// https://stackoverflow.com/questions/13716913/default-value-for-struct-member-in-c

struct StateMachine_s
{
	State_t state;
	int8_t timeout;
	void (*eventHandler)(struct StateMachine_s*, Event_t);
};

typedef struct StateMachine_s StateMachine;

void handle_event_table(StateMachine* inState, Event_t inEvent);

void handle_event_state(StateMachine* inState, Event_t inEvent)
{
	switch(inState->state)
	{
		case STATE_TEMP_READING:
		{
			//TODO: Access the TMP102 for a temperature reading via I2C.
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
			/*
			 * use the last temperature reading to update an
			 * average temperature value.  Normal status
			 * messages should print the last reading and
			 * the current average.  You will then wait 15
			 * seconds for a timeout.
			 *
			 * disable all interrupts (alert) whilst in this state
			 */


			switch(inEvent)
			{
				case EVENT_TIMEOUT:
				{
					if(inState->timeout++ < MAX_TIMEOUTS)
					{
						inState->state = STATE_TEMP_READING;
					}
					else
					{
						inState->timeout = 0;
						// TODO: Go to TEMP_READING on the table based state machine
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
		}
		default:
			inState->state = STATE_TEMP_READING;
			break;
	}
}

void handle_event_table(StateMachine* inState, Event_t inEvent)
{
	inState->state = STATE_DISCONNECTED;
}

#endif
