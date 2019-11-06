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
 */

#ifndef STATEMACHINE
#define STATEMACHINE

/**
 * @brief Set of states available in the state machine.
 */
typedef enum  {
	STATE_TEMP_READING,
	STATE_AVERAGE_WAIT,
	STATE_TEMP_ALERT,
	STATE_DISCONNECTED,
	STATE_NUM_STATES
} State_t;

/**
 * @brief Set of events that must be handled by the state machine.
 */
typedef enum {
	EVENT_TIMEOUT,
	EVENT_COMPLETE,
	EVENT_ALERT,
	EVENT_DISCONNECT,
	EVENT_NUM_EVENTS
} Event_t;

/**
 * @brief The state object for the state machine.
 */
struct StateMachine_s
{
	State_t state;
	int8_t timeout;
	void (*eventHandler)(struct StateMachine_s*, Event_t);
};

/**
 * @brief Typedef for easier usage.
 */
typedef struct StateMachine_s StateMachine;

/**
 * @brief Handles event using a state-based approach.
 * @param inState Current state machine object.
 * @param inEvent Event to handle.
 */
void handle_event_state(StateMachine* inState, Event_t inEvent);

/**
 * @brief Handles event using a table-based approach.
 * @param inState Current state machine object.
 * @param inEvent Event to handle.
 */
void handle_event_table(StateMachine* inState, Event_t inEvent);

#endif
