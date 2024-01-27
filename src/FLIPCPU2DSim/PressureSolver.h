//
// Created by luc on 23/01/24.
//

#ifndef VULKANFLUIDSIMULATION_PRESSURESOLVER_H
#define VULKANFLUIDSIMULATION_PRESSURESOLVER_H


#include <cstdint>
#include <vector>
#include "../lib/graphicsDataStructures/Matrices.h"

template<typename T, uint32_t cells_per_row>
class PressureSolver {

public:
    int solve(SparseMatrix<T, cells_per_row>& A, HeapMatrix<T, cells_per_row>& b,
              HeapMatrix<T, cells_per_row>& res, uint32_t maxIterations, double tolerance);

private:
    void formPreconditioner(SparseMatrix<T, cells_per_row>& A);

    void applyPreconditioner(SparseMatrix<T, cells_per_row>& A, HeapMatrix<T, cells_per_row>& x, HeapMatrix<T, cells_per_row>& y, HeapMatrix<T, cells_per_row>& m);

    void addAndScale(HeapMatrix<T, cells_per_row>& a, HeapMatrix<T, cells_per_row>& b, T scale);

    T maxAbs(HeapMatrix<T, cells_per_row>& a);

    T dot(HeapMatrix<T, cells_per_row>& a, HeapMatrix<T, cells_per_row>& b);

    HeapMatrix<T, cells_per_row> m_preconditioner;
};


template<typename T, uint32_t cells_per_row>
int PressureSolver<T, cells_per_row>::solve(SparseMatrix<T, cells_per_row>& A, HeapMatrix<T, cells_per_row>& b,
                                            HeapMatrix<T, cells_per_row>& res, uint32_t maxIterations, double tolerance) {
    // PCG
    auto maxB = maxAbs(b);
    if (maxB == 0) return 0;
    m_preconditioner.resize(b.size());
    formPreconditioner(A);

    HeapMatrix<T, cells_per_row> r = b;
    HeapMatrix<T, cells_per_row> z = b;
    HeapMatrix<T, cells_per_row> m = b;

    applyPreconditioner(A, r, z, m);
    HeapMatrix<T, cells_per_row> s = z;

    T rho = dot(r, z);
    if (rho == 0) return 0;

    for (uint32_t iter = 0; iter < maxIterations; ++iter) {
        A.multiply(s.getVector(), z.getVector());
        T alpha = rho / dot(s, z);

        // res = res + alpha*s
        addAndScale(res, s, alpha);

        // r = r - alpha*z
        addAndScale(r, z, -alpha);

        if (maxAbs(r) < tolerance*maxB) {
            return 0;
        }

        applyPreconditioner(A,r, z, m);
        T rho_new = dot(r, z);
        T beta = rho_new / rho;

        // s = r + beta*s
        addAndScale(s, z, beta);

        rho = rho_new;
    }

    return 1;
}

template<typename T, uint32_t cells_per_row>
void PressureSolver<T, cells_per_row>::applyPreconditioner(SparseMatrix<T, cells_per_row>& A, HeapMatrix<T, cells_per_row>& x,
                                                           HeapMatrix<T, cells_per_row>& y, HeapMatrix<T, cells_per_row>& m) {
    // solve L*m=x
    double d;
    for (uint32_t j = 1; j < m_preconditioner.nRows()-1; ++j) {
        for (uint32_t i = 1; i < m_preconditioner.nCols()-1; ++i) {
            m(i, j) = 0;
            uint32_t mIdx = i + j*A.cellsPerRow();
            if (A.getIdx(mIdx, mIdx) != -1) {
                d = x(i, j) - A(mIdx-1, mIdx) *m_preconditioner(i-1, j) *m(i-1, j)
                            - A(mIdx-A.cellsPerRow(), mIdx) * m_preconditioner(i, j-1) * m(i, j-1);
                m(i, j) = d*m_preconditioner(i, j);
            }
        }
    }
    // solve L'*y=m
    for (uint32_t j = m_preconditioner.nRows()-2; j > 0; --j) {
        for (uint32_t i = m_preconditioner.nCols()-1; i > 0; --i) {
            y(i, j) = 0;
            uint32_t mIdx = i + j*A.cellsPerRow();
            if (A.getIdx(mIdx, mIdx) != -1) {
                d = m(i, j) - A(mIdx, mIdx+1) * m_preconditioner(i, j) * y(i+1, j)
                            - A(mIdx, mIdx+A.cellsPerRow()) * m_preconditioner(i, j) * m(i, j+1);
                y(i, j) = d*m_preconditioner(i, j);
            }
        }
    }
}

template<typename T, uint32_t cells_per_row>
void PressureSolver<T, cells_per_row>::formPreconditioner(SparseMatrix<T, cells_per_row>& A){
    double d;
    const double mic_parameter=0.99;
    for (uint32_t i = A.cellsPerRow()+1; i < m_preconditioner.size(); i++) {
        if (A.getIdx(i, i) != -1) {
            d = A(i, i) - std::pow(A(i-1, i)*m_preconditioner[i-1], 2)
                        - std::pow(A(i-A.cellsPerRow(), i)*m_preconditioner[i-A.cellsPerRow()], 2)
                        - mic_parameter*( A(i-1, i)*A(i-1, i-1+A.cellsPerRow())*std::pow(m_preconditioner[i-1], 2) +
                                          A(i-A.cellsPerRow(), i)*A(i-A.cellsPerRow(), i-A.cellsPerRow()+1)*std::pow(m_preconditioner[i-A.cellsPerRow()], 2)
                                         );

            m_preconditioner[i] = 1.0/std::sqrt(d + 1e-6);
        } else {
            m_preconditioner[i] = 0;
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
T PressureSolver<T, cells_per_row>::maxAbs(HeapMatrix<T, cells_per_row>& a) {
    T max = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        max = std::max(max, std::abs(a[i]));
    }
    return max;
}

#endif //VULKANFLUIDSIMULATION_PRESSURESOLVER_H
