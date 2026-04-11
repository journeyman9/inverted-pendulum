#include <stdlib.h>                         // Prototype declarations for I/O functions
#include <avr/io.h>                         // Port I/O for SFR's
#include <vector>

class Planner {
public:
    Planner();
    ~Planner();

    void planner_set_value();
    int16_t planner_get_value();
	float plan(float x[4]);
    float x_r[4];
};