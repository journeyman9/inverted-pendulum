#include <stdlib.h>                         // Prototype declarations for I/O functions
#include <avr/io.h>                         // Port I/O for SFR's
#include "lqr.h"

Lqr::Lqr() {
}

Lqr::~Lqr() {
}

float Lqr::calculate_action(float[4] e) {
	// Calculate the control action using the LQR controller
	u = K[0] * e[0] + K[1] * e[1] + K[2] * e[2] + K[3] * e[3];
	return u;
}

void Lqr::lqr_set_value() {
}

int16_t Lqr::lqr_get_value() {
	return 0;
}