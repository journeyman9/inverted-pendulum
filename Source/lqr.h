#include <stdlib.h>                         // Prototype declarations for I/O functions
#include <avr/io.h>                         // Port I/O for SFR's

class lqr {
public:
    lqr();
    ~lqr();

    void lqr_set_value();
    int16_t lqr_get_value();
   
    int16_t u;
};