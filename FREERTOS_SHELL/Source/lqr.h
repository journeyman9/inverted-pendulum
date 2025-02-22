#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <map>

class lqr {
public:
    lqr();
    ~lqr();

    void lqr_set_value();
    int16_t lqr_get_value();
    
    std::vector<int16_t> K = {0, 0, 0, 0};
    std::vector<int16_t> u;
};