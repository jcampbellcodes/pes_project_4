#include "uCUnit.h"
#include <stdint.h>
#include "logger.h"
#include "setup_teardown.h"
#include "state_machine.h"
#include "handle_led.h"

#define HANDLE_ERROR(num_errors) if(num_errors > 0) set_led(1, RED);

int main()
{
	initialize();
	set_led(1, BLUE);
	{
	    UCUNIT_TestcaseBegin("state_event_handler_state");
	    StateMachine stateMachine = {STATE_TEMP_READING, 0, handle_event_state};
	    UCUNIT_CheckIsEqual( stateMachine.eventHandler, handle_event_state );
	    UCUNIT_CheckIsEqual( stateMachine.state, STATE_TEMP_READING );
	    UCUNIT_CheckIsEqual( stateMachine.timeout, 0 );
	    UCUNIT_TestcaseEnd();
	}

	HANDLE_ERROR(ucunit_testcases_failed)

	{
	    UCUNIT_TestcaseBegin("state_temp_reading_complete");
	    StateMachine stateMachine = {STATE_TEMP_READING, 0, handle_event_state};
	    UCUNIT_CheckIsEqual( stateMachine.state, STATE_TEMP_READING );
	    stateMachine.eventHandler(&stateMachine, EVENT_COMPLETE);
	    UCUNIT_CheckIsEqual( stateMachine.state, STATE_AVERAGE_WAIT );
	    UCUNIT_TestcaseEnd();
	}

	HANDLE_ERROR(ucunit_testcases_failed)

	{
		UCUNIT_TestcaseBegin("state_temp_reading_alert");
		StateMachine stateMachine = {STATE_TEMP_READING, 0, handle_event_state};
		UCUNIT_CheckIsEqual( stateMachine.state, STATE_TEMP_READING );
		stateMachine.eventHandler(&stateMachine, EVENT_ALERT);
		UCUNIT_CheckIsEqual( stateMachine.state, STATE_TEMP_ALERT );
		UCUNIT_TestcaseEnd();
	}

	HANDLE_ERROR(ucunit_testcases_failed)

	{
		UCUNIT_TestcaseBegin("state_temp_reading_disconnect");
		StateMachine stateMachine = {STATE_TEMP_READING, 0, handle_event_state};
		UCUNIT_CheckIsEqual( stateMachine.state, STATE_TEMP_READING );
		stateMachine.eventHandler(&stateMachine, EVENT_DISCONNECT);
		UCUNIT_CheckIsEqual( stateMachine.state, STATE_DISCONNECTED );
		UCUNIT_TestcaseEnd();
	}

	HANDLE_ERROR(ucunit_testcases_failed)

	{
	    UCUNIT_TestcaseBegin("state_average_wait_timeouts");
	    StateMachine stateMachine = {STATE_TEMP_READING, 0, handle_event_state};

	    UCUNIT_CheckIsEqual( stateMachine.state, STATE_TEMP_READING );
	    stateMachine.eventHandler(&stateMachine, EVENT_COMPLETE);
	    UCUNIT_CheckIsEqual( stateMachine.state, STATE_AVERAGE_WAIT );
	    stateMachine.eventHandler(&stateMachine, EVENT_TIMEOUT);
	    UCUNIT_CheckIsEqual( stateMachine.state, STATE_TEMP_READING );
	    UCUNIT_CheckIsEqual( &stateMachine.eventHandler, &handle_event_state );

	    stateMachine.eventHandler(&stateMachine, EVENT_COMPLETE);
	    UCUNIT_CheckIsEqual( stateMachine.state, STATE_AVERAGE_WAIT );
	    stateMachine.eventHandler(&stateMachine, EVENT_TIMEOUT);
	    UCUNIT_CheckIsEqual( stateMachine.state, STATE_TEMP_READING );
	    UCUNIT_CheckIsEqual( &stateMachine.eventHandler, &handle_event_state );

	    stateMachine.eventHandler(&stateMachine, EVENT_COMPLETE);
	    UCUNIT_CheckIsEqual( stateMachine.state, STATE_AVERAGE_WAIT );
	    stateMachine.eventHandler(&stateMachine, EVENT_TIMEOUT);
	    UCUNIT_CheckIsEqual( stateMachine.state, STATE_TEMP_READING );
	    UCUNIT_CheckIsEqual( &stateMachine.eventHandler, &handle_event_state );

	    stateMachine.eventHandler(&stateMachine, EVENT_COMPLETE);
	    UCUNIT_CheckIsEqual( stateMachine.state, STATE_AVERAGE_WAIT );
	    stateMachine.eventHandler(&stateMachine, EVENT_TIMEOUT);
	    UCUNIT_CheckIsEqual( stateMachine.state, STATE_TEMP_READING );
	    UCUNIT_CheckIsEqual( &stateMachine.eventHandler, &handle_event_table );

	    UCUNIT_TestcaseEnd();
	}

	HANDLE_ERROR(ucunit_testcases_failed)

	{
		UCUNIT_TestcaseBegin("state_average_wait_disconnect");
		StateMachine stateMachine = {STATE_TEMP_READING, 0, handle_event_state};
		stateMachine.state = STATE_AVERAGE_WAIT;
		stateMachine.eventHandler(&stateMachine, EVENT_DISCONNECT);
		UCUNIT_CheckIsEqual( stateMachine.state, STATE_DISCONNECTED );
		UCUNIT_TestcaseEnd();
	}

	HANDLE_ERROR(ucunit_testcases_failed)

	{
		UCUNIT_TestcaseBegin("state_temp_alert_complete");
		StateMachine stateMachine = {STATE_TEMP_READING, 0, handle_event_state};
		stateMachine.state = STATE_TEMP_ALERT;
		stateMachine.eventHandler(&stateMachine, EVENT_COMPLETE);
		UCUNIT_CheckIsEqual( stateMachine.state, STATE_AVERAGE_WAIT );
		UCUNIT_TestcaseEnd();
	}

	HANDLE_ERROR(ucunit_testcases_failed)

	{
		UCUNIT_TestcaseBegin("state_temp_alert_disconnect");
		StateMachine stateMachine = {STATE_TEMP_READING, 0, handle_event_state};
		stateMachine.state = STATE_TEMP_ALERT;
		stateMachine.eventHandler(&stateMachine, EVENT_DISCONNECT);
		UCUNIT_CheckIsEqual( stateMachine.state, STATE_DISCONNECTED );
		UCUNIT_TestcaseEnd();
	}

	HANDLE_ERROR(ucunit_testcases_failed)

	{
		UCUNIT_TestcaseBegin("state_disconnected_any");
		StateMachine stateMachine = {STATE_TEMP_READING, 0, handle_event_state};
		stateMachine.state = STATE_DISCONNECTED;
		for(Event_t event = 0; event < EVENT_NUM_EVENTS; event++)
		{
			stateMachine.eventHandler(&stateMachine, event);
			UCUNIT_CheckIsEqual( stateMachine.state, STATE_DISCONNECTED );
		}
		UCUNIT_TestcaseEnd();
	}

	HANDLE_ERROR(ucunit_testcases_failed)


	{
	    UCUNIT_TestcaseBegin("table_temp_reading_complete");
	    UCUNIT_TestcaseEnd();
	}

	{
		UCUNIT_TestcaseBegin("table_temp_reading_alert");
		UCUNIT_TestcaseEnd();
	}

	{
		UCUNIT_TestcaseBegin("table_temp_reading_disconnect");
		UCUNIT_TestcaseEnd();
	}


	{
	    UCUNIT_TestcaseBegin("table_average_wait_timeouts");
	    UCUNIT_TestcaseEnd();
	}

	{
		UCUNIT_TestcaseBegin("table_average_wait_disconnect");
		UCUNIT_TestcaseEnd();
	}

	{
		UCUNIT_TestcaseBegin("table_temp_alert_complete");
		UCUNIT_TestcaseEnd();
	}

	{
		UCUNIT_TestcaseBegin("table_temp_alert_disconnect");
		UCUNIT_TestcaseEnd();
	}

	{
		UCUNIT_TestcaseBegin("table_disconnected_any");
		UCUNIT_TestcaseEnd();
	}

	terminate();

	return 0;
}
