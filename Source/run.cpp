#include <iostream>
#include "kalman.h"

int main() {
    
    std::vector<float> x0{0.0, 0.0, 0.0, 0.0};
    std::vector<float> u{0.01};
    
    Kalman kalman;
    
    std::vector<float> y{-0.005, 0.05};
    
    for (int i=0; i<x0.size(); i++) {
        std::cout << "x_hat_" << i << ": " << observer.getStateEstimate()[i] << std::endl;
    }
    return 0;
    return 0;
}