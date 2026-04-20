#include <stdlib.h>                         // Prototype declarations for I/O functions
#include <avr/io.h>                         // Port I/O for SFR's
#include "planner.h"
#include <vector>

Planner::Planner() {
}

Planner::~Planner() {
}

std::vector<float> Planner::plan(float x[4]) {
	x_r = {0.0f, 0.0f, 0.0f, 0.0f};
	return x_r;
}

void Planner::planner_set_value() {
}

int16_t Planner::planner_get_value() {
	return 0;
}