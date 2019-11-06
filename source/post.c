#include "handle_led.h"
#include "post.h"
#include "logger.h"
#include "MKL25Z4.h"
#include "tmp102.h"

#include "delay.h"

bool power_on_self_test()
{
	LOG_STRING( LOG_MODULE_POST, LOG_SEVERITY_DEBUG, "Starting power-on self test." );
    set_led(1, RED);
    set_led(1, GREEN);
    set_led(1, BLUE);

//    if(tmp102_connected())
//    {
//    	while(1)
//    	{
//    		delay(300);
//    		float temp = readTempC();
//    		delay(300);
//    		setAlertMode(0);
//    	}
//    }

    return tmp102_connected();
}
