#include "handle_led.h"
#include "post.h"
#include "logger.h"
#include "MKL25Z4.h"
#include "tmp102.h"


bool power_on_self_test()
{
	LOG_STRING( LOG_MODULE_POST, LOG_SEVERITY_DEBUG, "Starting power-on self test." );
    set_led(1, RED);
    set_led(1, GREEN);
    set_led(1, BLUE);

    while(1)
    {
//    	if(!tmp102_connected())
//    	{
//    		LOG_STRING( LOG_MODULE_POST, LOG_SEVERITY_DEBUG, "Not connected to TMP102." );
//    	}

    	//setAlertMode(false);
    	float temp = readTempC();
    	LOG_STRING_ARGS( LOG_MODULE_POST, LOG_SEVERITY_DEBUG, "Temp claims to be: { %f }", temp );
    }

    return true;
}
