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
	//float K[4] = {1, 0.95425908, -1.00477956, -1.06011656};
	float K[4] = {59.9999999, 2.81774755, -30.61428425, -1.52650829};
};