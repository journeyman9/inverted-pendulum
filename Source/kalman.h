#include <cmath>
#include <array>

using Vector = std::array<float, 4>;
using Matrix = std::array<std::array<float, 4>, 4>;
using AugMatrix = std::array<std::array<float, 8>, 4>;

class Kalman {
private:
    static Vector multiplyMatrixVector(const Matrix& A, const Vector& x);
    static Vector addVectors(const Vector& a, const Vector& b);
    static Vector subVectors(const Vector& a, const Vector& b);
    static Matrix transpose(const Matrix& A);    
    static Matrix inverse(const Matrix& A);
    static Matrix multiplyMatrices(const Matrix& A, const Matrix& B);
    static Matrix addMatrices(const Matrix& A, const Matrix& B);
    static Matrix subMatrices(const Matrix& A, const Matrix& B);
	
    const Matrix F{{
        {{1.0f, 0.007257f, 0.000025f, 0.0f}},
        {{0.0f, 0.820222f, 0.006013f, 0.000025f}},
        {{0.0f, -0.003159f, 1.001441f, 0.008004f}},
        {{0.0f, -0.76469f, 0.359385f, 1.001441f}}
    }};
    const Vector G{{
        0.000192f,
        0.046451f,
        0.000816f,
        0.197582f
    }};
    const Matrix Q{{
        {{0.0001f, 0.0f, 0.0f, 0.0f}},
        {{0.0f, 0.0001f, 0.0f, 0.0f}},
        {{0.0f, 0.0f, 0.0005f, 0.0f}},
        {{0.0f, 0.0f, 0.0f, 0.01f}}
    }};
    const Matrix R{{
        {{std::pow(0.01f, 2), 0.0f, 0.0f, 0.0f}},
        {{0.0f, std::pow(0.1f, 2), 0.0f, 0.0f}},
        {{0.0f, 0.0f, std::pow(0.06f, 2), 0.0f}},
        {{0.0f, 0.0f, 0.0f, std::pow(0.3f, 2)}}
    }};
    // Use H to remove measurement and infer from model
    const Matrix H{{
        {{1.0f, 0.0f, 0.0f, 0.0f}},
        {{0.0f, 1.0f, 0.0f, 0.0f}},
        {{0.0f, 0.0f, 1.0f, 0.0f}},
        {{0.0f, 0.0f, 0.0f, 1.0f}}
    }};
    const Matrix I{{
        {{1.0f, 0.0f, 0.0f, 0.0f}},
        {{0.0f, 1.0f, 0.0f, 0.0f}},
        {{0.0f, 0.0f, 1.0f, 0.0f}},
        {{0.0f, 0.0f, 0.0f, 1.0f}}
    }};
public:
    Kalman(const Vector& x0);
    ~Kalman();
    Matrix P{{
        {{0.001f, 0.0f, 0.0f, 0.0f}},
        {{0.0f, 0.001f, 0.0f, 0.0f}},
        {{0.0f, 0.0f, 0.001f, 0.0f}},
        {{0.0f, 0.0f, 0.0f, 0.001f}}
    }};
    Vector y;
    Vector z;
    Vector x_hat;
    float u_k;
    Matrix Kf;
    Vector getStateEstimate();
    void predict(const float& u_k);
    void update(const Vector& z);
};