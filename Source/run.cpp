#include <iostream>
#include "kalman.h"

int main() {
    
    const std::vector<float> x0{0.0, 0.0, 0.0, 0.0};
    Kalman observer(x0);
    
    std::vector<float> u{0.01};
    observer.predict(u);

    std::vector<float> z{0.215, 0.05, 0.9873, -0.05};
    observer.update(z);
    
    for (int i=0; i<x0.size(); i++) {
        std::cout << "x_hat_" << i << ": " << observer.getStateEstimate()[i] << std::endl;
    }
    return 0;
}