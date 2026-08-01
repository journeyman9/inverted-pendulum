#include <iostream>
#include <cmath>
#include <vector>

class Kalman {
public:
    Kalman();
    ~Kalman();
    std::vector<std::vector<float>> F{
        {1.0, 0.000988, 0.0, 0.0},
        {0.0, 0.975533, 0.000818, 0.0},
        {0.0, -0.000052, 1.000023, 0.001},
        {0.0, -0.104024, 0.045185, 1.000023}

    };
    std::vector<std::vector<float>> G{
        {0.000003},
        {0.006322},
        {0.000013},
        {0.026878}
    };
    std::vector<std::vector<float>> Q{
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
    std::vector<std::vector<float>> R{
        {1.0}
    };
    std::vector<std::vector<float>> H{
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
    std::vector<std::vector<float>> P{
        {0.001, 0.0, 0.0, 0.0},
        {0.0, 0.001, 0.0, 0.0},
        {0.0, 0.0, 0.001, 0.0},
        {0.0, 0.0, 0.0, 0.001}
    };
    const std::vector<std::vector<float>> I{
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
    std::vector<float> z;
    std::vector<float> x_hat;
    std::vector<float> u_k;
    std::vector<float> Kf;
    
    std::vector<float> multiplyMatrixVector(const std::vector<std::vector<float>>& A, const std::vector<float>& x);
    std::vector<float> addVectors(const std::vector<float>& a, const std::vector<float>& b);
    std::vector<float> subVectors(const std::vector<float>& a, const std::vector<float>& b);
    std::vector<float> transpose(const std::vector<float>& A);
    
    void predict(std::vector<float>& x_hat, std::vector<float>& u_k);
    void update(std::vector<float>& u_k, std::vector<float>& z)
};