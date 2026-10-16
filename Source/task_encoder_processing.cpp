#include <stdlib.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "rs232int.h"
#include "time_stamp.h"
#include "frt_task.h"
#include "frt_text_queue.h"
#include "frt_queue.h"
#include "frt_shared_data.h"
#include "shares.h"

#include "shared_data_sender.h"
#include "shared_data_receiver.h"

#include "task_encoder_processing.h"

task_encoder_processing::task_encoder_processing(
	const char* a_name,
	unsigned portBASE_TYPE a_priority,
	size_t a_stack_size,
	emstream* p_ser_dev
)
: frt_task (a_name, a_priority, a_stack_size, p_ser_dev) {
}

void task_encoder_processing::run(void) {
	// --- Pendulum encoder QDEC setup ---
	PORTE.DIRCLR = (PIN0_bm | PIN1_bm);
	PORTE.PIN0CTRL = PORT_ISC_LEVEL_gc;
	PORTE.PIN1CTRL = PORT_ISC_LEVEL_gc;

	EVSYS.CH2MUX = EVSYS_CHMUX_PORTE_PIN0_gc;
	EVSYS.CH2CTRL = EVSYS_QDEN_bm | EVSYS_DIGFILT_2SAMPLES_gc;

	TCC1.CTRLD = TC_EVACT_QDEC_gc | TC_EVSEL_CH2_gc;
	TCC1.PER = 0x7CD;
	TCC1.CTRLA = TC_CLKSEL_DIV1_gc;

	// --- Motor encoder QDEC setup ---
	PORTE.DIRCLR = PIN2_bm | PIN3_bm;
	PORTE.PIN2CTRL |= PORT_ISC_LEVEL_gc;
	PORTE.PIN3CTRL |= PORT_ISC_LEVEL_gc;

	EVSYS.CH0MUX = EVSYS_CHMUX_PORTE_PIN2_gc;
	EVSYS.CH0CTRL = EVSYS_QDEN_bm | EVSYS_DIGFILT_2SAMPLES_gc;

	TCD0.CTRLD = TC_EVACT_QDEC_gc | TC_EVSEL_CH0_gc;
	TCD0.PER = 0xFFFF;
	TCD0.CTRLA = TC_CLKSEL_DIV1_gc;

	// --- Pendulum state ---
	int16_t pend_raw = TCC1.CNT;
	int16_t pend_prev_raw = pend_raw;
	int32_t pend_count_unwrapped = 0;

	const float pend_counts_per_rev = 1997.0f;
	const float PI = 3.141592f;
	const float dt = 0.001f;  // ISR fires at exactly 1kHz

	// --- Motor state ---
	float motor_prev_x = 0.0f;
	const float meters_per_count = 0.00006f;
	linear_offset->put(0);

	// Signal that we're ready — start the ISR timer now
	// (TCE0 is started in main after this task is created)

	while (1) {
		// Block until ISR gives the semaphore
		xSemaphoreTake(xEncoderSemaphore, portMAX_DELAY);

		// Atomically copy the ISR-captured values (int16_t is not atomic on 8-bit AVR)
		cli();
		int16_t local_pend_raw = isr_pendulum_raw;
		int16_t local_motor_raw = isr_motor_raw;
		sei();

		// ---- Pendulum processing ----
		int16_t pend_dcount_raw = local_pend_raw - pend_prev_raw;

		if (pend_dcount_raw > (int16_t)(pend_counts_per_rev / 2.0f)) {
			pend_dcount_raw -= (int16_t)pend_counts_per_rev;
		}
		else if (pend_dcount_raw < (int16_t)(-pend_counts_per_rev / 2.0f)) {
			pend_dcount_raw += (int16_t)pend_counts_per_rev;
		}

		int16_t pend_dcount_signed = -pend_dcount_raw;
		pend_count_unwrapped += pend_dcount_signed;
		float theta_unwrapped = pend_count_unwrapped * (2.0f * PI / pend_counts_per_rev);
		float omega = (pend_dcount_signed * (2.0f * PI / pend_counts_per_rev)) / dt;

		pendulum_encoder->put((int16_t)pend_count_unwrapped);
		pendulum_encoder_radians->put(theta_unwrapped);
		pendulum_encoder_w_radians->put(omega);

		pend_prev_raw = local_pend_raw;

		// ---- Motor processing ----
		float motor_x = ((float)local_motor_raw) * meters_per_count - linear_offset->get();
		linear_position->put(motor_x);
		linear_velocity->put((motor_x - motor_prev_x) / dt);

		motor_prev_x = motor_x;

		runs++;
	}
}
