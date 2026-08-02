#include <iostream>
#include <cmath>
#include <vector>
#include <stdexcept>

using Vector = std::vector<float>;
using Matrix = std::vector<std::vector<float>>;

class Kalman {
private:
    Vector multiplyMatrixVector(const Matrix& A, const Vector& x);
    Vector addVectors(const Vector& a, const Vector& b);
    Vector subVectors(const Vector& a, const Vector& b);
    Matrix transpose(const Matrix& A);    
    Matrix inverse(const Matrix& A);
    Matrix multiplyMatrices(const Matrix& A, const Matrix& B);
    Matrix addMatrices(const Matrix& A, const Matrix& B);
    Matrix subMatrices(const Matrix& A, const Matrix& B);
    const Matrix F{
        {1.0, 0.000988, 0.0, 0.0},
        {0.0, 0.975533, 0.000818, 0.0},
        {0.0, -0.000052, 1.000023, 0.001},
        {0.0, -0.104024, 0.045185, 1.000023}

    };
    const Matrix G{
        {0.000003},
        {0.006322},
        {0.000013},
        {0.026878}
    };
    const Matrix Q{
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
    const Matrix R{
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
    const Matrix H{
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
    Matrix P{
        {0.001, 0.0, 0.0, 0.0},
        {0.0, 0.001, 0.0, 0.0},
        {0.0, 0.0, 0.001, 0.0},
        {0.0, 0.0, 0.0, 0.001}
    };
    const Matrix I{
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 1.0, 0.0, 0.0},
        {0.0, 0.0, 1.0, 0.0},
        {0.0, 0.0, 0.0, 1.0}
    };
public:
    Kalman(const std::vector<float>& x0);
    ~Kalman();
    Vector y;
    Vector z;
    Vector x_hat;
    Vector u_k;
    Matrix Kf;
    Vector getStateEstimate();
    void predict(Vector& u_k);
    void update(Vector& z);
};