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

#include "task_motor_command.h"                  // Template

task_motor_command::task_motor_command(
	const char* a_name,
	unsigned portBASE_TYPE a_priority,
	size_t a_stack_size,
	emstream* p_ser_dev
)
// Call the parent (task base) constructor
: frt_task (a_name, a_priority, a_stack_size, p_ser_dev) {
	// Nothing to do in this constructor other than call the parent constructor
}


void task_motor_command::run(void) {
	// Make a variable which will hold times to use for precise task scheduling
	portTickType previousTicks = xTaskGetTickCount ();
	
	PORTC.DIRSET = PIN0_bm | PIN1_bm | PIN2_bm; // Configure PC0 and PC1 as outputs
	PORTC.OUTSET = PIN2_bm; // Disable sleep mode
	TCC0.CTRLA = TC0_CLKSEL0_bm; // Configures Clock select bits prescaler to 1
	TCC0.CTRLB = TC0_WGMODE0_bm | TC0_WGMODE1_bm; // Configures waveform generation mode to single slope PWM

	TCC0.PER = 0x640; // Set period to 320 counts for a PWM frequency 20kHz with 20% duty cycle 32MHz / (1 * 1600) = 20kHz
	TCC0.CCA = 0; // Ensure channel A is off when enabled
	TCC0.CCB = 0; // Ensure channel B is off when enabled
	TCC0.CTRLB |= TC0_CCAEN_bm | TC0_CCBEN_bm; // Enable output compare on channels A and B

	output = 0;
	
	while(1) {

		output = motor_command->get();
		// PWM function to command motor
		if (output >= 0)
		{
			TCC0.CCA = output;
			TCC0.CCB = 0;
		}
		else if (output < 0)
		{
			TCC0.CCA = 0;
			TCC0.CCB = -output;
		}
		
		/*
		if (runs % 100 == 0) {
			*p_serial << "Scary, scary skeletons!" << endl;
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