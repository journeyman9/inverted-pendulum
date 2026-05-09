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
	//float K[4] = {-0.993112, -7.62016, 20.20541, 4.410639}; // Identity discrete
	//float K[4] = {-1.000000, -7.648160, 20.296019, 4.431045}; // Identity continuous
	//float K[4] = {-0.316228, -7.175212, 18.448768, 3.925358}; // Continuous R50, Q5,10
	//float K[4] = {-2.236068, -8.107504, 20.797846, 4.415312}; // Unstable
	float K[4] = {-0.316228, -7.173978, 18.463715, 3.922573}; // R100, Q10, 100
};