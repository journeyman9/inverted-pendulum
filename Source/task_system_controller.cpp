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

#include "task_system_controller.h"                  // Template
#include "lqr.h"

task_system_controller::task_system_controller(
	const char* a_name,
	unsigned portBASE_TYPE a_priority,
	size_t a_stack_size,
	emstream* p_ser_dev
)
// Call the parent (task base) constructor
: frt_task (a_name, a_priority, a_stack_size, p_ser_dev) {
	// Nothing to do in this constructor other than call the parent constructor
}


void task_system_controller::run(void) {
	// Make a variable which will hold times to use for precise task scheduling
	portTickType previousTicks = xTaskGetTickCount ();
	
		
	while(1) {
		switch (state) {
			// Home right
			case(0): 
				linear_offset->put(0);
				if (begin->get()) {
					reset->put(0);
					stop->put(0);
					motor_command->put(220);

					if (rightLimitSwitch->get()) {
						linear_offset->put(linear_position->get());
						transition_to(1);
					}
				}
				break;

			// Home left
			case(1):
				begin->put(0);
				motor_command->put(-220);

				if (leftLimitSwitch->get()) {
					left_home = linear_position->get();
					transition_to(2);
				}

				if (reset->get() == 1) {
					transition_to(0);
				}
				break;
			
			// Move away from limit switch and delay
			case(2): 
				motor_command->put(0);
				delay_ms(200);
				if (leftLimitSwitch->get()){
					motor_command->put(220);
				}
				else {
				transition_to(3);
				}

				break;
			
			// Center cart
			case(3):
				// Aim for the middle of the carriage
				position_set = left_home / 2;
				position_error = position_set - linear_position->get();
				
				// P controller to get to middle position
				motor_command->put(Kp * position_error);
				
				// Pring out all of our linear control 
				/*
				if (runs%100 == 0){
					*p_serial << "position_set: " << position_set << endl;
					*p_serial << "linear Position: " << linear_position->get() << endl;
					*p_serial << "motor_command: " << motor_command->get() << endl;
				}
				*/
				
				// Transition to error if limit switches are hit
				if (leftLimitSwitch->get() || rightLimitSwitch->get()) {
					*p_serial << "LIMIT SWITCH HIT ERROR" << endl;
					transition_to(100);
				}

				if (reset->get() == 1) {
					reset->put(0);
					transition_to(0);
				}

				if (go->get() == 1) {
					transition_to(4);
				}
				break;
			
			// Balance
			case(4):
				angle_set = pendulum_encoder->get();
				go->put(0);

				Lqr controller;
				
				float e[4] = {
					linear_position->get() - position_set,	
					linear_velocity->get() - 0,
					pendulum_encoder_radians->get() - angle_set,
					pendulum_encoder_w_radians->get() - 0,
				}

				motor_command->put(controller.calculate_action(e));
				
				if (leftLimitSwitch->get() || rightLimitSwitch->get()) {
					*p_serial << "LIMIT SWITCH HIT ERROR" << endl;
					transition_to(100);
				}

				if (reset->get() == 1) {
					reset->put(0);
					transition_to(0);
				}

				break;
				
			case(100):
				motor_command->put(0);
				
				/*
				if (runs%100 == 0) {
					*p_serial << "Error State" << endl;
				}
				*/
				
				if (reset->get()){
					reset->put(0);
					transition_to(0);
				}
			
				break;
		}	
		
		/*
		if (runs % 100 == 0) {
			*p_serial << "b: " << begin->get() << endl;
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