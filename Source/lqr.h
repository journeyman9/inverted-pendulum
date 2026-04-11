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
	float K[4] = {29.21413192, 24.12018189, 100.78018064, -1.44048738};
	//float K[4] = {29.13059689, 30.67422592, 163.56977604, -4.03851092}; // Unstable
};