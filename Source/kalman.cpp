#include <iostream>
#include "kalman.h"

Kalman::Kalman() {
}

Kalman::~Kalman() {
}

std::vector<float> multiplyMatrixVector(const std::vector<std::vector<float>>& A, const std::vector<float>& x) {
    std::vector<float> result(A.size(), 0.0);
    for (int i=0; i<A.size(); ++i) {
        for (int j=0; j<x.size(); ++j) {
            result[i] += A[i][j] * x[j];
        }
    }
    return result;
}

std::vector<float> addVectors(const std::vector<float>& a, const std::vector<float>& b) {
    std::vector<float> result(a.size(), 0.0);
    for (int i=0; i<a.size(); ++i) {
        result[i] = a[i] + b[i];
    }
    return result;
}

std::vector<float> subVectors(const std::vector<float>& a, const std::vector<float>& b) {
    std::vector<float> result(a.size(), 0.0);
    for (int i=0; i<a.size(); ++i) {
        result[i] = a[i] - b[i];
    }
    return result;
}

std::vector<float> transpose(const std::vector<float>& A) {
    std::vector<float> result(A.size(), 0.0);
    return result;
}

void predict(std::vector<float>& x_hat, std::vector<float>& u_k) {

}

void update(std::vector<float>& u_k, std::vector<float>& z) {

}