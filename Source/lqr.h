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
	float K[4] = {10.25690698, 11.93796898, 51.81724419, 4.74674896}; //R0.007, Q1&10 old eq'n with -1
	//float K[4] = {29.21413192, 24.12018189, 100.78018064, -1.44048738}; //old eqn's works with -1
	//float K[4] = {-29.19490681,-33.92094379, 141.5476876, 24.66740691}; // Designed
};