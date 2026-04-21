#include <stdlib.h>                         // Prototype declarations for I/O functions
#include <avr/io.h>                         // Port I/O for SFR's
#include "lqr.h"
#include <vector>

Lqr::Lqr() {
}

Lqr::~Lqr() {
}

float Lqr::calculate_action(float x[4], float x_r[4], float position_set, float angle_set) {
	x_tilde = x[0] - position_set;
	theta_tilde = x[2] - 3.1415f;

	// Calculate the control action using the LQR controller
	u = -1.0 * K[0] * (x_tilde - x_r[0]) + K[1] * (x[1] - x_r[1]) + K[2] * (theta_tilde - x_r[2]) + K[3] * (x[3] - x_r[3]);
	
	u *= 1600 / 24;
	
	if (u > 1600) {
		u = 1600;
	}
	else if (u < -1600) {
		u = -1600;
	}
	return u;
}

void Lqr::lqr_set_value() {
}

int16_t Lqr::lqr_get_value() {
	return 0;
}