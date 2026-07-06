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
	
	float K[4] = {-11.949841, -10.272218, 17.298045, 2.645009}; // first gains that worked
	//float K[4] = {-11.949841, -13.066187, 17.298045, 2.645009}; // b = 8
	
	//float K[4] = {-15.082511, -10.894473, 18.379604, 2.798140}; // PP contender
	//float K[4] = {-15.082511, -10.894473, 25.379604, 2.798140}; // David's win increasing 18->25 theta
	//float K[4] = {-25.082511, -11.15, 30.379604, 3.00}; // Kissing instability
	
	//float K[4] = {-7.348469, -10.565024, 21.523943, 3.134255};
};
