//
// Created by luc on 23/01/24.
//

#ifndef VULKANFLUIDSIMULATION_PRESSURESOLVER_H
#define VULKANFLUIDSIMULATION_PRESSURESOLVER_H


#include <cstdint>
#include <vector>
#include "../../lib/graphicsDataStructures/Matrices.h"

template<typename T, uint32_t cells_per_row>
class PressureSolver {

public:
    int solve(SparseMatrix<T, cells_per_row>& A, HeapMatrix<T, cells_per_row>& r,
              HeapMatrix<T, cells_per_row>& res, uint32_t maxIterations, double tolerance);

private:
    void formPreconditioner(SparseMatrix<T, cells_per_row>& A);
    void applyPreconditioner(SparseMatrix<T, cells_per_row>& A, const HeapMatrix<T, cells_per_row>& x, HeapMatrix<T, cells_per_row>& y, HeapMatrix<T, cells_per_row>& m);
    void addAndScale(HeapMatrix<T, cells_per_row>& a, HeapMatrix<T, cells_per_row>& b, T scale);
    void scaleAndAdd(HeapMatrix<T, cells_per_row>& a, HeapMatrix<T, cells_per_row>& b, T scale);

    T maxAbs(HeapMatrix<T, cells_per_row>& a);

    T dot(HeapMatrix<T, cells_per_row>& a, HeapMatrix<T, cells_per_row>& b);

    HeapMatrix<T, cells_per_row> m_preconditioner;
};


template<typename T, uint32_t cells_per_row>
int PressureSolver<T, cells_per_row>::solve(SparseMatrix<T, cells_per_row>& A, HeapMatrix<T, cells_per_row>& r,
                                            HeapMatrix<T, cells_per_row>& res, uint32_t maxIterations, double tolerance) {
    // PCG
    auto maxB = maxAbs(r);

    if (maxB < 1e-20) return 0;
    m_preconditioner.resize(r.size());
    formPreconditioner(A);

    HeapMatrix<T, cells_per_row> z{r.size()};
    HeapMatrix<T, cells_per_row> m{r.size()};

    applyPreconditioner(A, r, z, m);
    HeapMatrix<T, cells_per_row> s = z;

    T rho = dot(r, z);
    if (rho == 0) return 0;

    for (uint32_t iter = 0; iter < maxIterations; ++iter) {
        A.multiply(s, z);
        T alpha = rho / dot(s, z);

        // res = res + alpha*s
        addAndScale(res, s, alpha);

        // r = r - alpha*z
        addAndScale(r, z, -alpha);

        if (maxAbs(r) < tolerance*maxB) {
//            std::cout << "converged in: " << iter << " iterations\n";
            return 0;
        }

        applyPreconditioner(A,r, z, m);
        T rhoNew = dot(r, z);
        T beta = rhoNew / rho;

        // s = z + beta*s
        scaleAndAdd(s, z, beta);

        rho = rhoNew;
    }

    return 1;
}

template<typename T, uint32_t cells_per_row>
void PressureSolver<T, cells_per_row>::applyPreconditioner(SparseMatrix<T, cells_per_row>& A, const HeapMatrix<T, cells_per_row>& x,
                                                           HeapMatrix<T, cells_per_row>& y, HeapMatrix<T, cells_per_row>& m) {
    // solve L*m=x
    double d;
    for (uint32_t j = 1; j < m_preconditioner.nRows()-1; ++j) {
        for (uint32_t i = 1; i < m_preconditioner.nCols()-1; ++i) {
            m(i, j) = 0;
            if (A.isValidIdx(i, j)) {
                d = x(i, j) - A(i, j, 1) *m_preconditioner(i-1, j) *m(i-1, j)
                            - A(i, j, 2) * m_preconditioner(i, j-1) * m(i, j-1);
                m(i, j) = d*m_preconditioner(i, j);
            }
        }
    }

    // solve L'*y=m
    for (uint32_t j = m_preconditioner.nRows()-2; j > 0; --j) {
        for (uint32_t i = m_preconditioner.nCols()-2; i > 0; --i) {
            y(i, j) = 0;
            if (A.isValidIdx(i, j)) {
                d = m(i, j) - A(i, j, 3) * m_preconditioner(i, j) * y(i+1, j)
                            - A(i, j, 4) * m_preconditioner(i, j) * y(i, j+1);
                y(i, j) = d*m_preconditioner(i, j);
            }
        }
    }
}

template<typename T, uint32_t cells_per_row>
void PressureSolver<T, cells_per_row>::formPreconditioner(SparseMatrix<T, cells_per_row>& A){
    double d;
    const double mic_parameter=0.99;
    for (uint32_t j = 1; j < m_preconditioner.nRows()-1; ++j) {
        for (uint32_t i = 1; i < m_preconditioner.nCols()-1; ++i) {
            if (A.isValidIdx(i, j)) {
                d = A(i, j, 0) - std::pow(A(i, j, 1)*m_preconditioner(i-1, j), 2)
                            - std::pow(A(i, j, 2)*m_preconditioner(i, j-1), 2)
                            - mic_parameter*(A(i, j, 1)*A(i-1, j, 4)*std::pow(m_preconditioner(i-1, j), 2) +
                                             A(i, j, 2)*A(i, j-1, 3)*std::pow(m_preconditioner(i, j-1), 2)
                );
                m_preconditioner(i, j) = 1.0/std::sqrt(d + 1e-6);
            } else {
                m_preconditioner(i, j) = 0;
            }
        }
    }

}


template<typename T, uint32_t cells_per_row>
T PressureSolver<T, cells_per_row>::dot(HeapMatrix<T, cells_per_row>& a, HeapMatrix<T, cells_per_row>& b) {
    T acc = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        acc += a[i]*b[i];
    }
    return acc;
}

template<typename T, uint32_t cells_per_row>
void PressureSolver<T, cells_per_row>::addAndScale(HeapMatrix<T, cells_per_row>& a, HeapMatrix<T, cells_per_row>& b, T scale) {
    for (size_t i = 0; i < a.size(); ++i) {
        a[i] += scale*b[i];
    }
}

template<typename T, uint32_t cells_per_row>
void PressureSolver<T, cells_per_row>::scaleAndAdd(HeapMatrix<T, cells_per_row>& a, HeapMatrix<T, cells_per_row>& b, T scale) {
    for (size_t i = 0; i < a.size(); ++i) {
        a[i] = b[i] + scale*a[i];
    }
}

template<typename T, uint32_t cells_per_row>
T PressureSolver<T, cells_per_row>::maxAbs(HeapMatrix<T, cells_per_row>& a) {
    T res = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        if (std::abs(a[i]) > res) res = std::abs(a[i]);
    }
    return res;
}

#endif //VULKANFLUIDSIMULATION_PRESSURESOLVER_H
