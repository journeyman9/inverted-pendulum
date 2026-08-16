#ifndef KALMAN_H
#define KALMAN_H

#include <cmath>
#include <array>

using Vector = std::array<float, 4>;
using Matrix = std::array<std::array<float, 4>, 4>;

class Kalman {
private:
    static void multiplyMatVec(const Matrix& A, const Vector& x, Vector& out);
    static void multiplyMat(const Matrix& A, const Matrix& B, Matrix& out);
    static void addMat(const Matrix& A, const Matrix& B, Matrix& out);
    static void subMat(const Matrix& A, const Matrix& B, Matrix& out);
    static void transposeMat(const Matrix& A, Matrix& out);
    static void inverseMat(const Matrix& A, Matrix& out);

    // Scratch space in .bss, not on the stack
    static Matrix scratch1;
    static Matrix scratch2;
    static Matrix scratch3;
    static Vector vscratch1;

    const Matrix F{{
        {{1.0f, 0.000988f, 0.0f, 0.0f}},
        {{0.0f, 0.975533f, 0.000818f, 0.0f}},
        {{0.0f, -0.000052f, 1.000023f, 0.001f}},
        {{0.0f, -0.104024f, 0.045185f, 1.000023f}}
    }};
    const Vector G{{
        0.000003f,
        0.006322f,
        0.000013f,
        0.026878f
    }};
    const Matrix Q{{
        {{0.000001f, 0.0f, 0.0f, 0.0f}},
        {{0.0f, 0.0001f, 0.0f, 0.0f}},
        {{0.0f, 0.0f, 0.00001f, 0.0f}},
        {{0.0f, 0.0f, 0.0f, 0.01f}}
    }};
    const Matrix R{{
        {{0.0001f, 0.0f, 0.0f, 0.0f}},
        {{0.0f, 0.01f, 0.0f, 0.0f}},
        {{0.0f, 0.0f, 0.0036f, 0.0f}},
        {{0.0f, 0.0f, 0.0f, 0.09f}}
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
    Vector x_hat;
    Matrix Kf;
    Vector getStateEstimate();
    void predict(const float& u_k);
    void update(const Vector& z);
};

#endif
