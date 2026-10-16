#ifndef TASKENCODERPROCESSING_H
#define TASKENCODERPROCESSING_H

#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "rs232int.h"
#include "time_stamp.h"
#include "frt_task.h"
#include "frt_queue.h"
#include "frt_text_queue.h"
#include "frt_shared_data.h"

#include "shares.h"

class task_encoder_processing : public frt_task {
	protected:

	public:
		task_encoder_processing(const char*, unsigned portBASE_TYPE, size_t, emstream*);

		void run(void);
};

#endif
