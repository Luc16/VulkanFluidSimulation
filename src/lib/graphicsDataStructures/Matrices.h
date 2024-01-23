//
// Created by luc on 14/04/23.
//

#ifndef VULKANFLUIDSIMULATION_MATRICES_H
#define VULKANFLUIDSIMULATION_MATRICES_H
#include "../utils.h"

template<typename T, size_t totalSize, uint32_t row>
class Matrix{
public:

    void swap(Matrix& other){
        m_matrix.swap(other.m_matrix);
    }

    constexpr T& operator() (uint32_t i, uint32_t j) {
        return m_matrix[i + row*j];
    }
    constexpr const T& operator()(uint32_t i, uint32_t j) const{
        return m_matrix[i + row*j];
    }
    constexpr T& operator() (glm::ivec2& vec) {
        return m_matrix[vec.x + row*vec.y];
    }
    constexpr const T& operator()(glm::ivec2& vec) const{
        return m_matrix[vec.x + row*vec.y];
    }
    constexpr T& operator() (const glm::ivec2& vec) {
        return m_matrix[vec.x + row*vec.y];
    }
    constexpr const T& operator()(const glm::ivec2& vec) const{
        return m_matrix[vec.x + row*vec.y];
    }
    constexpr T& operator[] (uint32_t i) {
        return m_matrix[i];
    }
    constexpr const T& operator[](uint32_t i) const{
        return m_matrix[i];
    }

    [[nodiscard]] size_t size() const {
        return m_matrix.size();
    }

private:
    std::array<T, totalSize> m_matrix{};
};

template<typename T, uint32_t row>
class HeapMatrix{
public:
    explicit HeapMatrix(size_t size): m_matrix(size) {};
    HeapMatrix() = default;

    void swap(HeapMatrix& other){
        m_matrix.swap(other.m_matrix);
    }

    void resize(size_t size) {
        m_matrix.resize(size);
    }

    constexpr T& operator() (uint32_t i, uint32_t j) {
        return m_matrix[i + row*j];
    }
    constexpr const T& operator()(uint32_t i, uint32_t j) const{
        return m_matrix[i + row*j];
    }
    constexpr T& operator() (glm::ivec2& vec) {
        return m_matrix[vec.x + row*vec.y];
    }
    constexpr const T& operator()(glm::ivec2& vec) const{
        return m_matrix[vec.x + row*vec.y];
    }
    constexpr T& operator() (const glm::ivec2& vec) {
        return m_matrix[vec.x + row*vec.y];
    }
    constexpr const T& operator()(const glm::ivec2& vec) const{
        return m_matrix[vec.x + row*vec.y];
    }
    constexpr T& operator[] (uint32_t i) {
        return m_matrix[i];
    }
    constexpr const T& operator[](uint32_t i) const{
        return m_matrix[i];
    }

    std::vector<T>& getVector() {
        return m_matrix;
    }

    [[nodiscard]] size_t size() const {
        return m_matrix.size();
    }

private:
    std::vector<T> m_matrix{};
};

template<typename T>
class Matrix3D{
public:
    Matrix3D(uint32_t row, uint32_t col, uint32_t depth): m_row(row), m_col(col), m_depth(depth), m_rowCol(row*col), m_data(row*col*depth) {}
    Matrix3D() = default;
    Matrix3D(const Matrix3D<T> &) = delete;
    Matrix3D &operator=(const Matrix3D<T> &) = delete;

    void resize(uint32_t row, uint32_t col, uint32_t depth) {
        m_row = row;
        m_col = col;
        m_depth = depth;
        m_rowCol = row*col;
        m_data.resize(row*col*depth);
    }

    void swap(Matrix3D& other){
        m_data.swap(other.m_data);
    }

    constexpr T& operator() (uint32_t i, uint32_t j, uint32_t k) {
        return m_data[i + m_row*j + m_rowCol*k];
    }
    constexpr const T& operator()(uint32_t i, uint32_t j, uint32_t k) const{
        return m_data[i + m_row*j + m_rowCol*k];
    }
    constexpr T& operator() (glm::ivec3& vec) {
        return m_data[vec.x + m_row*vec.y + m_rowCol*vec.z];
    }
    constexpr const T& operator()(glm::ivec3& vec) const{
        return m_data[vec.x + m_row*vec.y + m_rowCol*vec.z];
    }
    constexpr T& operator[] (uint32_t i) {
        return m_data[i];
    }
    constexpr const T& operator[](uint32_t i) const{
        return m_data[i];
    }

private:
    std::vector<T> m_data{};
    uint32_t m_row{}, m_col{}, m_depth{}, m_rowCol{};
};


template<typename T, uint32_t cells_per_row, uint32_t nonzero_per_sparse_row = 5>
class SparseMatrix{
public:

    SparseMatrix() = default;
    SparseMatrix(size_t fluid_cells, size_t total_cells) {
        resize(fluid_cells, total_cells);
    }

    void resize(uint32_t fluid_cells, uint32_t total_cells) {
        m_indices.resize(total_cells, -1);
        m_matrix.resize(fluid_cells*nonzero_per_sparse_row);
    }

    void set(uint32_t i, uint32_t j, T val) {
        if (m_indices[i] == -1) m_indices[i] = curSize++;
        int idx = getIdx(i, j);
        if (idx == -1) return;
        m_matrix[idx] = val;
    }

    constexpr T operator()(uint32_t i, uint32_t j) const{
        int idx = getIdx(i, j);
        return (idx != -1) ? m_matrix[idx] : 0;

    }

    constexpr T& operator[] (uint32_t i) {
        return m_matrix[i];
    }
    constexpr const T& operator[](uint32_t i) const{
        return m_matrix[i];
    }

    constexpr void multiply(std::vector<T>& v, std::vector<T>& res) const {
        for (uint32_t i = 0; i < v.size(); i++) {
            if (m_indices[i] == -1) continue;
            uint32_t idx = m_indices[i]*nonzero_per_sparse_row;
            res[i] =
                    m_matrix[idx]*v[i] +
                    ((i > 0) ? m_matrix[idx+2]*v[i-1] : 0) +
                    ((i < v.size() - 1) ? m_matrix[idx+1]*v[i+1] : 0) +
                    ((i >= cells_per_row) ? m_matrix[idx+4]*v[i-cells_per_row] : 0) +
                    ((i < v.size() - cells_per_row) ? m_matrix[idx+3]*v[i+cells_per_row] : 0);
        }
    }

private:
    [[nodiscard]] int getIdx(uint32_t i, uint32_t j) const {
        if (m_indices[i] < 0) return -1;
//        if (j > i) std::swap(j, i);
        std::array<uint32_t, 5> conditions = {
                i-j == 0,
                i-j == 1,
                i-j == cells_per_row,
                i-j == -1,
                i-j == -cells_per_row
        };
        uint32_t comb = conditions[0] || conditions[1] || conditions[2] || conditions[3] || conditions[4];
        return -1*(!comb) + m_indices[i]*nonzero_per_sparse_row*comb + conditions[1] + 2*conditions[3] + 3*conditions[2] + 4*conditions[4];
    }

    std::vector<T> m_matrix{};
    uint32_t curSize = 0;
    std::vector<int> m_indices{};
};
#endif //VULKANFLUIDSIMULATION_MATRICES_H
