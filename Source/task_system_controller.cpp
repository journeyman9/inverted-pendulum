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
#include "planner.h"
#include <vector>

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
	portTickType lastMeasuredTicks = previousTicks;   // For measurement
	
	uint32_t timing_samples[10] = {0};
	uint8_t sample_idx = 0;
	
	Lqr controller;
	Planner planner;
	bool set_already = false;
	float x[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	float x_r[4] = {0.0f, 0.0f, 0.0f, 0.0f};
	float u = 0.0f;
		
	while(1) {
		/*
		currentTicks = xTaskGetTickCount();
		uint32_t delta = currentTicks - lastMeasuredTicks;
		lastMeasuredTicks = currentTicks;
		
		timing_samples[sample_idx] = delta;
		sample_idx = (sample_idx + 1) % 10;
		
		if (runs % 100 == 0) {
			*p_serial  << "Timing: ";
			for (int i=0; i<10; i++) {
				*p_serial << timing_samples[i] << " ";
			}
			*p_serial << endl;
		}
		*/
		
		switch (state) {
			// Home right
			case(0): 
				linear_offset->put(0);
				set_already = false;
				integrated_error = 0.0f; // Reset integral
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
				position_set = left_home / 2.0;
				position_error = position_set - linear_position->get();
				
				integrated_error = integrated_error + position_error; 
				
				// P controller to get to middle position

				motor_command->put((int16_t)(Kp * position_error + ((Ki * integrated_error))));
				
				// Print out all of our linear control 
				/*
				if (runs%100 == 0){
					*p_serial << "position_set: " << position_set << endl;
					*p_serial << "linear Position: " << linear_position->get() << endl;
					*p_serial << "error : " << linear_position->get() - position_set  << endl;
					//*p_serial << "motor_command: " << motor_command->get() << endl;
				}
				*/
				
				// Transition to error if limit switches are hit
				if (leftLimitSwitch->get() || rightLimitSwitch->get()) {
					*p_serial << "LIMIT SWITCH HIT ERROR" << endl;
					transition_to(100);
				}

				if (reset->get() == 1) {
					reset->put(0);
					stop->put(0);
					transition_to(0);
				}

				if (go->get() == 1) {
					transition_to(4);
				}
				break;
			
			// Balance
			case(4):
				if (stop->get() == true) {
					transition_to(100);
				}
				if (set_already == false) {
					angle_set = pendulum_encoder_radians->get();
					set_already = true;
				}
				
				go->put(0);

				// Atomic read of state to prevent race conditions
				taskENTER_CRITICAL();
				x[0] = linear_position->get();
				x[1] = linear_velocity->get();
				x[2] = pendulum_encoder_radians->get();
				x[3] = pendulum_encoder_w_radians->get();
				taskEXIT_CRITICAL();

				planner.plan(x);
				
				// Error handling for too great of angle
				if ((x[2] - angle_set >= 0.2616) || (x[2] - angle_set < -0.2616)){
					*p_serial << "Outside Angle Recovery" << endl;
					transition_to(100);
					
				}
				u = controller.calculate_action(x, x_r, position_set, angle_set);				
				motor_command->put(u);
				
				/*
				if (runs%100 == 0) {
					char buf[6];
					char buf2[6];
					char buf3[6];
					*p_serial << "Angle radians: " << dtostrf(x[2], 0, 6, buf);
					*p_serial << " Angle error: " << dtostrf(x[2] - 3.14159f, 0, 6, buf2);
					*p_serial << " Motor u: " << dtostrf(u, 0, 6, buf3) << endl;
				}
				*/
				if (runs%100 == 0) {
					/*
					x: -0.159, xdot: 0.000, angle: -0.017, thetadot: 0.000, Motor u: 31.316
					x: -0.158, xdot: 0.000, angle: -0.024, thetadot: 0.000, Motor u: 39.541
					x: -0.152, xdot: 0.000, angle: -0.011, thetadot: 0.000, Motor u: 30.284
					x: -0.150, xdot: 0.000, angle: -0.027, thetadot: 0.000, Motor u: 49.569
					x: -0.147, xdot: 0.000, angle: -0.052, thetadot: 0.000, Motor u: 80.388
					x: -0.136, xdot: 0.240, angle: -0.058, thetadot: 3.141, Motor u: -289.933
					x: -0.103, xdot: 0.300, angle: -0.005, thetadot: 0.000, Motor u: 264.534
					x: -0.066, xdot: 0.300, angle: 0.068, thetadot: 0.000, Motor u: 211.067
					x: -0.047, xdot: 0.120, angle: 0.093, thetadot: 0.000, Motor u: 76.008
					x: -0.045, xdot: 0.000, angle: 0.080, thetadot: 0.000, Motor u: 10.583
					x: -0.046, xdot: 0.000, angle: 0.090, thetadot: 0.000, Motor u: -0.998
					x: -0.057, xdot: -0.120, angle: 0.087, thetadot: 0.000, Motor u: -87.941
					x: -0.071, xdot: -0.240, angle: 0.112, thetadot: 0.000, Motor u: -208.733
					x: -0.113, xdot: -0.600, angle: 0.061, thetadot: 0.000, Motor u: -428.829
					x: -0.177, xdot: -0.600, angle: -0.055, thetadot: 0.000, Motor u: -345.795
					x: -0.222, xdot: -0.180, angle: -0.118, thetadot: 0.000, Motor u: -24.754
					x: -0.236, xdot: -0.120, angle: -0.096, thetadot: -3.141, Motor u: 528.965
					x: -0.243, xdot: 0.120, angle: -0.099, thetadot: -6.281, Motor u: 1238.889
					x: -0.206, xdot: 0.420, angle: 0.058, thetadot: 22.000, Motor u: -1600.000
					x: -0.200, xdot: -0.300, angle: 0.200, thetadot: 22.000, Motor u: -1600.000
					x: -0.252, xdot: -0.360, angle: 0.087, thetadot: -6.281, Motor u: 693.908
					x: -0.288, xdot: -0.180, angle: 0.027, thetadot: -28.250, Motor u: 1600.000
					*/
					char buf[3];
					char buf2[3];
					char buf3[3];
					char buf4[3];
					char buf5[3];
					*p_serial << "x: " << dtostrf(x[0], 0, 3, buf);
					*p_serial << ", xdot: " << dtostrf(x[1], 0, 3, buf2);
					*p_serial << ", angle: " << dtostrf(x[2] - 3.14159f, 0, 3, buf3);
					*p_serial << ", thetadot: " << dtostrf(x[3], 0, 3, buf4);
					*p_serial << ", Motor u: " << dtostrf(u, 0, 3, buf5) << endl;
				}
							
				if (leftLimitSwitch->get() || rightLimitSwitch->get()) {
					*p_serial << "LIMIT SWITCH HIT ERROR" << endl;
					transition_to(100);
				}

				if (reset->get() == 1) {
					reset->put(0);
					stop->put(0);
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
					stop->put(0);
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