//
// Created by luc on 14/04/23.
//

#ifndef VULKANFLUIDSIMULATION_MATRICES_H
#define VULKANFLUIDSIMULATION_MATRICES_H
#include "../lib/utils.h"

template<typename T, size_t size, uint32_t row>
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

private:
    std::array<T, size> m_matrix{};
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

#endif //VULKANFLUIDSIMULATION_MATRICES_H
