#include <stdlib.h>                         // Prototype declarations for I/O functions
#include <avr/io.h>                         // Port I/O for SFR's
#include <vector>

class Lqr {
public:
    Lqr();
    ~Lqr();

    void lqr_set_value();
    int16_t lqr_get_value();
	float calculate_action(float x[4], std::vector<float> x_r, float position_set, float angle_set);
   
    float u;
    float x_tilde;
    float theta_tilde;
	float K[4] = {-29.19490681, -33.92094379, 141.5476876, 24.66740691};
};