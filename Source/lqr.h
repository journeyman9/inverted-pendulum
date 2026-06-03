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
	//float K[4] = {-1.000000, -7.540468, 15.837301, 2.625176}; // 1ft long Identity continuous
	//float K[4] = {-0.316228, -7.136556, 13.933033, 2.140864}; // Continuous R50, Q5,10
	//float K[4] = {-0.316228, -7.135345, 13.952051, 2.136987}; // R100, Q10, 100
	//float K[4] = {-0.316228, -7.853439, 15.015523, 2.308229}; // b = 1.0
	//float K[4] = {-0.316228, -7.455132, 14.413954, 2.215215}; // b = 0.5
	//float K[4] = {-0.316228, -8.650288, 16.219830, 2.494469}; // b = 2.0
	//float K[4] = {-0.316228, -7.852321, 15.032574, 2.304543}; // b = 1.0 R100, Q10, 100
	
	//float K[4] = {-0.316228, -7.853108, 15.223750, 2.334185}; // New J_idler b = 1.0 R100, Q10, 100
	//float K[4] = {-1.000000, -8.241383, 17.004799, 2.793499}; // Identity
	//float K[4] = {-1.414214, -8.297317, 16.761328, 2.480527};
	float K[4] = {-3.162278, -9.066431, 18.757438, 3.013267};
};
