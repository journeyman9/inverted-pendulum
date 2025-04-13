#include <stdlib.h>                         // Prototype declarations for I/O functions
#include <avr/io.h>                         // Port I/O for SFR's
#include <vector>

class lqr {
public:
    lqr();
    ~lqr();

    void lqr_set_value();
    int16_t lqr_get_value();
   
    float u;
	float K[4] = {1, 0.95425908, -1.00477956, -1.06011656};
};