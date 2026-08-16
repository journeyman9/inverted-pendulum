#include "kalman.h"
#include <string.h>

// Static scratch space lives in .bss — no stack or heap cost
Matrix Kalman::scratch1{};
Matrix Kalman::scratch2{};
Matrix Kalman::scratch3{};
Vector Kalman::vscratch1{};

Kalman::Kalman(const Vector& x0): x_hat(x0) {
}

Kalman::~Kalman() {
}

void Kalman::multiplyMatVec(const Matrix& A, const Vector& x, Vector& out) {
    for (uint8_t i = 0; i < 4; ++i) {
        out[i] = A[i][0]*x[0] + A[i][1]*x[1] + A[i][2]*x[2] + A[i][3]*x[3];
    }
}

void Kalman::multiplyMat(const Matrix& A, const Matrix& B, Matrix& out) {
    for (uint8_t i = 0; i < 4; ++i) {
        for (uint8_t j = 0; j < 4; ++j) {
            out[i][j] = A[i][0]*B[0][j] + A[i][1]*B[1][j]
                       + A[i][2]*B[2][j] + A[i][3]*B[3][j];
        }
    }
}

void Kalman::addMat(const Matrix& A, const Matrix& B, Matrix& out) {
    for (uint8_t i = 0; i < 4; ++i) {
        for (uint8_t j = 0; j < 4; ++j) {
            out[i][j] = A[i][j] + B[i][j];
        }
    }
}

void Kalman::subMat(const Matrix& A, const Matrix& B, Matrix& out) {
    for (uint8_t i = 0; i < 4; ++i) {
        for (uint8_t j = 0; j < 4; ++j) {
            out[i][j] = A[i][j] - B[i][j];
        }
    }
}

void Kalman::transposeMat(const Matrix& A, Matrix& out) {
    for (uint8_t i = 0; i < 4; ++i) {
        for (uint8_t j = 0; j < 4; ++j) {
            out[j][i] = A[i][j];
        }
    }
}

void Kalman::inverseMat(const Matrix& A, Matrix& out) {
    // Gauss-Jordan on augmented matrix [A|I] stored in-place using out + scratch
    // We use a flat augmented approach with a static buffer
    static float aug[4][8];

    for (uint8_t i = 0; i < 4; ++i) {
        for (uint8_t j = 0; j < 4; ++j) {
            aug[i][j] = A[i][j];
            aug[i][j + 4] = (i == j) ? 1.0f : 0.0f;
        }
    }

    for (uint8_t pivot = 0; pivot < 4; ++pivot) {
        uint8_t maxRow = pivot;
        for (uint8_t row = pivot + 1; row < 4; ++row) {
            if (fabsf(aug[row][pivot]) > fabsf(aug[maxRow][pivot])) {
                maxRow = row;
            }
        }

        if (maxRow != pivot) {
            for (uint8_t col = 0; col < 8; ++col) {
                float tmp = aug[pivot][col];
                aug[pivot][col] = aug[maxRow][col];
                aug[maxRow][col] = tmp;
            }
        }

        float pivotVal = aug[pivot][pivot];
        if (fabsf(pivotVal) < 1e-8f) {
            pivotVal = 1e-8f;
        }

        float invPivot = 1.0f / pivotVal;
        for (uint8_t col = 0; col < 8; ++col) {
            aug[pivot][col] *= invPivot;
        }

        for (uint8_t row = 0; row < 4; ++row) {
            if (row == pivot) continue;
            float factor = aug[row][pivot];
            for (uint8_t col = 0; col < 8; ++col) {
                aug[row][col] -= factor * aug[pivot][col];
            }
        }
    }

    for (uint8_t i = 0; i < 4; ++i) {
        for (uint8_t j = 0; j < 4; ++j) {
            out[i][j] = aug[i][j + 4];
        }
    }
}

// With H = I:
//   x_hat = F * x_hat + G * u
//   P = F * P * F^T + Q
void Kalman::predict(const float& u_k) {
    // x_hat = F * x_hat + G * u
    multiplyMatVec(F, x_hat, vscratch1);
    for (uint8_t i = 0; i < 4; ++i) {
        x_hat[i] = vscratch1[i] + G[i] * u_k;
    }

    // P = F * P * F^T + Q
    transposeMat(F, scratch1);       // scratch1 = F^T
    multiplyMat(P, scratch1, scratch2); // scratch2 = P * F^T
    multiplyMat(F, scratch2, scratch1); // scratch1 = F * P * F^T
    addMat(scratch1, Q, P);             // P = F * P * F^T + Q
}

// With H = I:
//   y = z - x_hat
//   S = P + R
//   K = P * S^{-1}
//   x_hat = x_hat + K * y
//   P = (I - K) * P * (I - K)^T + K * R * K^T
void Kalman::update(const Vector& z) {
    // y = z - x_hat
    for (uint8_t i = 0; i < 4; ++i) {
        y[i] = z[i] - x_hat[i];
    }
    // Wrap theta innovation
    while (y[2] > 3.141592f) y[2] -= 6.283184f;
    while (y[2] < -3.141592f) y[2] += 6.283184f;

    // S = P + R -> scratch1
    addMat(P, R, scratch1);

    // K = P * S^{-1}
    inverseMat(scratch1, scratch2);     // scratch2 = S^{-1}
    multiplyMat(P, scratch2, Kf);       // Kf = P * S^{-1}

    // x_hat = x_hat + K * y
    multiplyMatVec(Kf, y, vscratch1);
    for (uint8_t i = 0; i < 4; ++i) {
        x_hat[i] += vscratch1[i];
    }

    // P = (I - K) * P * (I - K)^T + K * R * K^T (Joseph form)
    // scratch1 = I - K
    for (uint8_t i = 0; i < 4; ++i) {
        for (uint8_t j = 0; j < 4; ++j) {
            scratch1[i][j] = ((i == j) ? 1.0f : 0.0f) - Kf[i][j];
        }
    }

    // scratch2 = (I-K) * P
    multiplyMat(scratch1, P, scratch2);

    // scratch3 = (I-K)^T
    transposeMat(scratch1, scratch3);

    // scratch1 = (I-K) * P * (I-K)^T
    multiplyMat(scratch2, scratch3, scratch1);

    // scratch2 = K^T
    transposeMat(Kf, scratch2);

    // scratch3 = R * K^T
    multiplyMat(R, scratch2, scratch3);

    // scratch2 = K * R * K^T
    multiplyMat(Kf, scratch3, scratch2);

    // P = scratch1 + scratch2
    addMat(scratch1, scratch2, P);
}

Vector Kalman::getStateEstimate() {
    return x_hat;
}
