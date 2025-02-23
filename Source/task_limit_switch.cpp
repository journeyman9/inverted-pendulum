#include <stdlib.h>                         // Prototype declarations for I/O functions
#include <avr/io.h>                         // Port I/O for SFR's
#include <avr/wdt.h>                        // Watchdog timer header
#include <avr/interrupt.h>					//
#include <string.h>                         // Functions for C string handling

#include "FreeRTOS.h"                       // Primary header for FreeRTOS
#include "task.h"                           // Header for FreeRTOS task functions
#include "queue.h"                          // FreeRTOS inter-task communication queues
#include "croutine.h"                       // Header for co-routines and such

#include "rs232int.h"                       // ME405/507 library for serial comm.
#include "time_stamp.h"                     // Class to implement a microsecond timer
#include "frt_task.h"                       // Header of wrapper for FreeRTOS tasks
#include "frt_text_queue.h"                 // Wrapper for FreeRTOS character queues
#include "frt_queue.h"                      // Header of wrapper for FreeRTOS queues
#include "frt_shared_data.h"                // Header for thread-safe shared data
#include "shares.h"                         // Global ('extern') queue declarations

#include "shared_data_sender.h"
#include "shared_data_receiver.h"

#include "task_limit_switch.h"               // Template

task_limit_switch::task_limit_switch (
	const char* a_name,
	unsigned portBASE_TYPE a_priority,
	size_t a_stack_size,
	emstream* p_ser_dev,
	uint8_t a_bit_mask
)
// Call the parent (task base) constructor
: frt_task (a_name, a_priority, a_stack_size, p_ser_dev) {
	// Nothing to do in this constructor other than call the parent constructor
	bit_mask =  a_bit_mask;
}


void task_limit_switch::run(void) {
	// Make a variable which will hold times to use for precise task scheduling
	portTickType previousTicks = xTaskGetTickCount ();

	PORTD.DIR |= bit_mask;
	PORTD.OUT |= bit_mask;
	
	if (bit_mask == PIN0_bm) {
		PORTD.PIN0CTRL = PORT_OPC_PULLUP_gc;
	}
	else if (bit_mask == PIN2_bm) {
		PORTD.PIN2CTRL = PORT_OPC_PULLUP_gc;
	}
	else {
	}

	bool rightLimit = false; 		// Init right limit bool
	bool leftLimit = false; 		// Init left limit bool

	while(1) {

		if(!(PORTD_IN & PIN0_bm)) {						// check whether limit is pressed (pin D0 is high)
				leftLimit = true;
				leftLimitSwitch->put(leftLimit);
		}
		else {
			leftLimit = false;
			leftLimitSwitch->put(leftLimit);
		}

		if (!(PORTD_IN & PIN2_bm)) {				// check whether limit is pressed (pin D1 is high)
			rightLimit = true;
			rightLimitSwitch->put(rightLimit);

		}
		else {
			rightLimit = false;
			rightLimitSwitch->put(rightLimit);
		}
		
		/*
		if(runs%100==0) {
			*p_serial << "Left" << leftLimitSwitch->get() << "\t";
			*p_serial << "Right" << rightLimitSwitch->get() << endl;
			*p_serial << "leftLimit: " << leftLimit << endl;
			*p_serial << "limits: " << leftLimit << rightLimit << endl;
			*p_serial << "rightLimit: " << rightLimit << endl;
		}
		*/
		
	
		// Increment counter for debugging
		runs++;


		// set dt
		//_delay_ms(1);
		// This is a method we use to cause a task to make one run through its task
		// loop every N milliseconds and let other tasks run at other times
		delay_from_to (previousTicks, configMS_TO_TICKS (1));
	}
}