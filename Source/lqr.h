#include <stdlib.h>                         // Prototype declarations for I/O functions
#include <avr/io.h>                         // Port I/O for SFR's
#include <vector>

class Lqr {
public:
    Lqr();
    ~Lqr();

    void lqr_set_value();
    int16_t lqr_get_value();
	float calculate_action(float e[4]);
   
    float u;
	//float K[4] = {300, 8.2687, -316.2833, -4.087}; // Manual tuning of Q&R
	float K[4] = {252.1769, 15.319, -266.185, -24.608}; // Discrete Sim
};