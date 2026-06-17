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
	//float K[4] = {-1.000000, -8.241383, 17.004799, 2.793499}; // Identity
	//float K[4] = {-1.414214, -8.297317, 16.761328, 2.480527}; // Broken Byrsons rule
	//float K[4] = {-3.162278, -9.066431, 18.757438, 3.013267}; // Q10 10 R1
	//float K[4] = {-5.000000, -9.504231, 18.081965, 2.773475}; // Q1000 100 R40
	//float K[4] = {-1.723943, -6.363216, 10.681588, 1.693016}; // Pole placement
	//float K[4] = {-17.831110, -12.171456, 21.070032, 3.201973}; // Pole placement, unstable but best
	
	float K[4] = {-11.949841, -10.272218, 17.298045, 2.645009}; // best
};
