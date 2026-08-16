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
        {{1.0f, 0.004703f, 0.00001f, 0.0f}},
        {{0.0f, 0.883504f, 0.003895f, 0.00001f}},
        {{0.0f, -0.001264f, 1.000564f, 0.005001}},
        {{0.0f, -0.495376f, 0.225138f, 1.000564f}}
    }};
    const Vector G{{
        0.000077f,
        0.0301f,
        0.000327f,
        0.127996f
    }};
    const Matrix Q{{
        {{0.000001f, 0.0f, 0.0f, 0.0f}},
        {{0.0f, 0.0001f, 0.0f, 0.0f}},
        {{0.0f, 0.0f, 0.00001f, 0.0f}},
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