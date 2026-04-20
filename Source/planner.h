#include <stdlib.h>                         // Prototype declarations for I/O functions
#include <avr/io.h>                         // Port I/O for SFR's
#include <vector>

class Planner {
public:
    Planner();
    ~Planner();

    void planner_set_value();
    int16_t planner_get_value();
	void plan(float x[4]);
    std::vector<float> x_r = {0.0f, 0.0f, 0.0f, 0.0f};
};