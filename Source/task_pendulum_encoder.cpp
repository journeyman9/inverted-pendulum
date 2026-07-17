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

#include "shared_data_sender.h"				// Shared data header
#include "shared_data_receiver.h"			// Shared data header

#include "task_pendulum_encoder.h"			// Header for this file

task_pendulum_encoder::task_pendulum_encoder(
	const char* a_name,
	unsigned portBASE_TYPE a_priority,
	size_t a_stack_size,
	emstream* p_ser_dev
)
// Call the parent (task base) constructor
: frt_task (a_name, a_priority, a_stack_size, p_ser_dev) {
	// Nothing to do in this constructor other than call the parent constructor
}

void task_pendulum_encoder::run(void) {
	// Make a variable which will hold times to use for precise task scheduling
	portTickType previousTicks = xTaskGetTickCount ();

	PORTE.DIRCLR = (PIN0_bm | PIN1_bm);							// set E0 & E1 as inputs
	PORTE.PIN0CTRL = PORT_ISC_LEVEL_gc;							// set E0 for level sensing
	PORTE.PIN1CTRL = PORT_ISC_LEVEL_gc;							// set E1 for level sensing

	EVSYS.CH2MUX = EVSYS_CHMUX_PORTE_PIN0_gc;					// set PE0 as Multiplexer for Event Chan 2
	EVSYS.CH2CTRL = EVSYS_QDEN_bm | EVSYS_DIGFILT_2SAMPLES_gc;	// enable quad encoder mode with 2-sample filtering

	TCC1.CTRLD = TC_EVACT_QDEC_gc | TC_EVSEL_CH2_gc;			// set TCC1 event action to quad decoding, and event source as Event Chan 1
	TCC1.PER = 0x7CD; 											// Empirically derived
	TCC1.CTRLA = TC_CLKSEL_DIV1_gc;								// start TCC1 with prescaler = 1

	int16_t raw_count = TCC1.CNT;
	int16_t prev_raw_count = raw_count;
	int16_t dcount_raw = 0;
	int16_t dcount_signed = 0;
	int32_t count_unwrapped = 0;
	
	const float counts_per_rev = 1997.0f;
	const float PI = 3.141592f;
	//const float dt = 0.001f;
	
	float theta_unwrapped = 0;
	float omega = 0;

	portTickType currentTicks;
	portTickType lastTicks = previousTicks;
	float dt;
	
	const float alpha = 0.80f; // Must match motor alpha
	const float beta = pow(alpha, 2) / (2.0 - alpha); 
	float theta_hat = 0.0f;
	float omega_hat = 0.0f;
	float omega_raw = 0.0f;
	bool filter_initialized = false;

	while(1) {
		//portTickType workStart = xTaskGetTickCount();
		
		currentTicks = xTaskGetTickCount();
		dt = (float)(currentTicks - lastTicks) * portTICK_RATE_MS / 1000.0f;
		lastTicks = currentTicks;

		raw_count = TCC1.CNT; 							// Read value from hardware
		dcount_raw = raw_count - prev_raw_count;
		
		bool wrapped = false;
		if (dcount_raw > counts_per_rev / 2.0) {
			dcount_raw -= counts_per_rev;
			wrapped = true;
		}
		else if (dcount_raw < -counts_per_rev / 2.0) {
			dcount_raw += counts_per_rev;
			wrapped = true;
		}
		
		dcount_signed = -dcount_raw;
		count_unwrapped += dcount_signed;
		theta_unwrapped = count_unwrapped * (2.0 * PI / counts_per_rev);
		
		if (dt > 0.0f) {
			omega_raw = (dcount_signed * (2.0 * PI / counts_per_rev)) / dt;
			
			if (!filter_initialized) {
				theta_hat = theta_unwrapped;
				omega_hat = omega_raw;
				filter_initialized = true;
			}
			else {
				// Alpha-beta filter prediction step
				float theta_predict = theta_hat + omega_hat * dt;
				float omega_predict = omega_hat;
				
				// Correction step
				float residual = theta_unwrapped - theta_predict;
				theta_hat = theta_predict + alpha * residual;
				omega_hat = omega_predict + (beta / dt) * residual;	
			}
			
			omega = omega_hat;
		}
		else {
			omega = omega_hat;
		}
		
		/*
		if ((runs%100 == 0) && (theta_unwrapped > 3.0)) {	
			if (wrapped || (omega > 2.0f || omega < -2.0f)) {
				*p_serial << "WRAP: raw=" << raw_count << " prev=" << prev_raw_count << " dcount=" << dcount_signed << " omega_raw=" << omega << " omega_filt=" << omega_filtered << " dt=" << dt << endl;
			}
		}
		*/
		
		pendulum_encoder->put((int16_t)count_unwrapped);
		pendulum_encoder_radians->put(theta_unwrapped); 	// Convert to radians
		
		pendulum_encoder_w_radians->put(omega);

		// Section of code used for unit testing, prints out curr count and queue value
		/*
		if(runs%100==0){
			char buf[6];
			//*p_serial << "Pendulum Ticks Counts: " << count_unwrapped << endl;
			*p_serial << "Pendulum Ticks Radians: " << dtostrf(pendulum_encoder_radians->get(), 0, 6, buf) << endl; 
		}
		*/
		
		prev_raw_count = raw_count;
	
		/*
		portTickType workEnd = xTaskGetTickCount();
        uint16_t work_duration = workEnd - workStart;

		if (runs % 500 == 0) {
			*p_serial << "PendEnc: " << work_duration << " ticks" << endl;
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