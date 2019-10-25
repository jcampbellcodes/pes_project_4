#include "uCUnit.h"
#include <stdint.h>
#include "logger.h"
#define ALLOC_SIZE 16

int main()
{
#ifdef DEBUG
	log_enable();
#endif

	{
	    UCUNIT_TestcaseBegin("we did it");
	    UCUNIT_TestcaseEnd();
	}

	return 0;
}
