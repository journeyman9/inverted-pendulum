#include <stdlib.h>                         // Prototype declarations for I/O functions
#include <avr/io.h>                         // Port I/O for SFR's
#include <vector>

class Lqr {
public:
    Lqr();
    ~Lqr();

    void lqr_set_value();
    int16_t lqr_get_value();
	float calculate_action(float x[4], float x_r[4], float position_set, float angle_set);
   
    float u;
    float x_tilde;
    float theta_tilde;
	//float K[4] = {-0.99288566, -7.51002462, 19.72965947, 4.27930451}; // Identity
	float K[4] = {-0.314702, -7.048762, 17.918977, 3.78218};
};