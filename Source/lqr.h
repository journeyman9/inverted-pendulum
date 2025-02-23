#include <stdlib.h>                         // Prototype declarations for I/O functions
#include <avr/io.h>                         // Port I/O for SFR's
#include <vector>

class lqr {
public:
    lqr();
    ~lqr();

    void lqr_set_value();
    int16_t lqr_get_value();
   
    int16_t u;
	std::vector<float> K{21.41, 7.89, 3.43, 44.11};
};