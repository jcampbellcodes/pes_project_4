#ifndef STATEMACHINE
#define STATEMACHINE

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

void handle_event_state(StateMachine* inState, Event_t inEvent);
void handle_event_table(StateMachine* inState, Event_t inEvent);

#endif
