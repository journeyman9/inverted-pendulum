#ifndef TASKSYSTEMCONTROLLER_H
#define TASKSYSTEMCONTROLLER_H

#include <stdlib.h>                         // Prototype declarations for I/O functions

#include "FreeRTOS.h"                       // Primary header for FreeRTOS
#include "task.h"                           // Header for FreeRTOS task functions
#include "queue.h"                          // FreeRTOS inter-task communication queues

#include "rs232int.h"                       // ME405/507 library for serial comm.
#include "time_stamp.h"                     // Class to implement a microsecond timer
#include "frt_task.h"                       // Header for ME405/507 base task class
#include "frt_queue.h"                      // Header of wrapper for FreeRTOS queues
#include "frt_text_queue.h"                 // Header for a "<<" queue class
#include "frt_shared_data.h"                // Header for thread-safe shared data

#include "shares.h"                         // Global ('extern') queue declarations

#include "math.h"

class task_system_controller : public frt_task{
	protected:
	
	public: 
		// Constructor creates a motor encoder task object
		task_system_controller(const char*, unsigned portBASE_TYPE, size_t, emstream*);
			
		// This gets called by the RTOS once to start this task's state loop
		void run(void);
		
		int16_t left_home;
		int16_t position_set;
		int16_t position_error;
		int16_t integrated_error;
		int16_t Kp = 3;
		int16_t Ki = 1; 
	
	};
	
#endif