#include "kalman.h"

Kalman::Kalman(const Vector& x0): x_hat(x0) {
}

Kalman::~Kalman() {
}

Vector Kalman::multiplyMatrixVector(const Matrix& A, const Vector& x) {
    Vector result{0.0f, 0.0f, 0.0f, 0.0f};
    for (int i=0; i<A.size(); ++i) {
        for (int j=0; j<x.size(); ++j) {
            result[i] += A[i][j] * x[j];
        }
    }
    return result;
}

Vector Kalman::addVectors(const Vector& a, const Vector& b) {
    Vector result{0.0f, 0.0f, 0.0f, 0.0f};
    for (int i=0; i<a.size(); ++i) {
        result[i] = a[i] + b[i];
    }
    return result;
}

Vector Kalman::subVectors(const Vector& a, const Vector& b) {
    Vector result {};
    for (int i=0; i<a.size(); ++i) {
        result[i] = a[i] - b[i];
    }
    return result;
}

Matrix Kalman::transpose(const Matrix& A) {
    if (A.empty()) {
        return {};
    }

	Matrix result {};

    for (size_t i = 0; i < 4; ++i) {
        for (size_t j = 0; j < 4; ++j) {
            result[j][i] = A[i][j];
        }
    }
    return result;
}

Matrix Kalman::inverse(const Matrix& A) {
    if (A.empty()) {
        return {};
    }
    
    if (A.size() != A[0].size()) {
      //throw std::invalid_argument("Matrix must be square");
    }

    size_t n = A.size();
    AugMatrix aug{};
    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            aug[i][j] = A[i][j];
        }

        aug[i][i + n] = 1.0;
    }

    for (size_t pivot = 0; pivot < n; ++pivot) {

        size_t maxRow = pivot;

        for (size_t row = pivot + 1; row < n; ++row) {
            if (std::abs(aug[row][pivot]) > std::abs(aug[maxRow][pivot])) {
                maxRow = row;
            }
        }

        if (maxRow != pivot) {
            std::swap(aug[pivot], aug[maxRow]);
        }

        float pivotValue = aug[pivot][pivot];

        if (std::abs(pivotValue) < 1e-8) {
            //throw std::runtime_error("Matrix is singular");
        }

        for (size_t col = 0; col < 2 * n; ++col) {
            aug[pivot][col] /= pivotValue;
        }

        for (size_t row = 0; row < n; ++row) {

            if (row == pivot) {
                continue;
            }

            float factor = aug[row][pivot];

            for (size_t col = 0; col < 2 * n; ++col) {
                aug[row][col] -= factor * aug[pivot][col];
            }
        }
    }
    
     Matrix result{};

    for (size_t i = 0; i < n; ++i) {
        for (size_t j = 0; j < n; ++j) {
            result[i][j] = aug[i][j + n];
        }
    }

    return result;
}

Matrix Kalman::multiplyMatrices(const Matrix& A, const Matrix& B) {
    if (A.empty() || B.empty()) {
        return {};
    }

    if (A[0].size() != B.size()) {
        //throw std::invalid_argument("Matrix dimensions do not align");
    }

    size_t rows = A.size();
    size_t cols = B[0].size();
    size_t innerDim = B.size();

    Matrix result {};

    for (size_t i = 0; i < rows; ++i) {
        for (size_t j = 0; j < cols; ++j) {
            for (size_t k = 0; k < innerDim; ++k) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }

    return result;
}

Matrix Kalman::addMatrices(const Matrix& A, const Matrix& B) {
    if (A.size() != B.size() || A[0].size() != B[0].size()) {
        //throw std::invalid_argument("Matrix dimensions must match");
    }

    Matrix result {};

    for (size_t i = 0; i < A.size(); ++i) {
        for (size_t j = 0; j < A[0].size(); ++j) {
            result[i][j] = A[i][j] + B[i][j];
        }
    }

    return result;
}

Matrix Kalman::subMatrices(const Matrix& A, const Matrix& B) {
    if (A.size() != B.size() || A[0].size() != B[0].size()) {
        //throw std::invalid_argument("Matrix dimensions must match");
    }

    Matrix result {};

    for (size_t i = 0; i < A.size(); ++i) {
        for (size_t j = 0; j < A[0].size(); ++j) {
            result[i][j] = A[i][j] - B[i][j];
        }
    }

    return result;
}

void Kalman::predict(float& u_k) {
	Vector Gu {};
	
	for (int i = 0; i < 4; ++i) {
		Gu[i] = G[i] * u_k;
	}
    x_hat = addVectors(multiplyMatrixVector(F, x_hat), Gu);
    P = addMatrices(
        multiplyMatrices(F, multiplyMatrices(P, transpose(F))),
        Q
    );
}

void Kalman::update(const Vector& z) {
    y = subVectors(z, multiplyMatrixVector(H, x_hat));
    Kf = multiplyMatrices(
        multiplyMatrices(P, transpose(H)),
        inverse(
            addMatrices(
                multiplyMatrices(
                    H,
                    multiplyMatrices(P, transpose(H))
                ),
                R
            )
        )
    );
    x_hat = addVectors(
        x_hat,
        multiplyMatrixVector(Kf, y)
    );
    
    P = addMatrices(
        multiplyMatrices(
            subMatrices(
                I,
                multiplyMatrices(Kf, H)
            ),
            multiplyMatrices(
                P, 
                transpose(
                    subMatrices(
                        I,
                        multiplyMatrices(Kf, H)
                    )
                )
            )
        ),
        multiplyMatrices(
            Kf,
            multiplyMatrices(R, transpose(Kf))
        )
    );

}

Vector Kalman::getStateEstimate() {
    return x_hat;
}