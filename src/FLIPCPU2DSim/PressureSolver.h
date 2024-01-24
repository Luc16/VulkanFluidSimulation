//
// Created by luc on 23/01/24.
//

#ifndef VULKANFLUIDSIMULATION_PRESSURESOLVER_H
#define VULKANFLUIDSIMULATION_PRESSURESOLVER_H


#include <cstdint>
#include <vector>
#include "../lib/graphicsDataStructures/Matrices.h"

class PressureSolver {

public:
    template<typename T, uint32_t cells_per_row>
    int solve(SparseMatrix<T, cells_per_row>& A, std::vector<T>& b, std::vector<T>& res, uint32_t maxIterations, double tolerance);

private:
    template<typename T>
    void addAndScale(std::vector<T>& a, std::vector<T>& b, T scale);

    template<typename T>
    T maxAbs(std::vector<T>& a);

    template<typename T>
    T dot(std::vector<T>& a, std::vector<T>& b);
};



template<typename T>
T PressureSolver::dot(std::vector<T>& a, std::vector<T>& b) {
    T acc = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        acc += a[i]*b[i];
    }
    return acc;
}

template<typename T>
void PressureSolver::addAndScale(std::vector<T>& a, std::vector<T>& b, T scale) {
    for (size_t i = 0; i < a.size(); ++i) {
        a[i] += scale*b[i];
    }
}

template<typename T>
T PressureSolver::maxAbs(std::vector<T>& a) {
    T max = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        max = std::max(max, std::abs(a[i]));
    }
    return max;
}

template<typename T, uint32_t cells_per_row>
int PressureSolver::solve(SparseMatrix<T, cells_per_row>& A, std::vector<T>& b, std::vector<T>& res, uint32_t maxIterations, double tolerance) {
    // PCG
    auto maxB = maxAbs(b);
    if (maxB == 0) return 0;

    std::vector<T> r = b;
    std::vector<T> prev_r = b;
    std::vector<T> s = b;

    std::vector<T> w(b.size(), 0);

    A.multiply(s, w);

    T alpha = dot(r, r) / dot(s, w);

    // res = alpha*s
    addAndScale(res, s, alpha);

    // r = r - alpha*w
    addAndScale(r, w, -alpha);

    uint32_t iter = 0;
    maxB = maxAbs(b);
    while (iter++ < maxIterations && maxAbs(r) > tolerance*maxB) {
        T beta = dot(r, r) / dot(prev_r, prev_r);

        // s = r + beta*s
        addAndScale(s, r, beta);

        A.multiply(s, w);

        alpha = dot(r, r) / dot(s, w);

        // res = res + alpha*s
        addAndScale(res, s, alpha);

        // r = r - alpha*w
        prev_r.swap(r);
        for (size_t i = 0; i < r.size(); ++i) {
            r[i] = prev_r[i] - alpha*w[i];
        }
    }

    if (iter >= maxIterations) return 1;
    return 0;
}

#endif //VULKANFLUIDSIMULATION_PRESSURESOLVER_H
