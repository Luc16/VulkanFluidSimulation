//
// Created by luc on 22/10/22.
//

#ifndef VULKANBASE_UTILS_H
#define VULKANBASE_UTILS_H

#include <vulkan/vulkan.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/hash.hpp>


#include <iostream>
#include <stdexcept>
#include <cstdlib>
#include <vector>
#include <optional>
#include <set>
#include <limits>
#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <cstring>


#include <functional>
#include <random>


// from: https://stackoverflow.com/a/57595105
template <typename T, typename... Rest>
void hashCombine(std::size_t& seed, const T& v, const Rest&... rest) {
    seed ^= std::hash<T>{}(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    (hashCombine(seed, rest), ...);
};




inline double randomDouble(){
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<double> dis_double;
    return dis_double(gen);
}

inline double randomDouble(double min, double max) {
    return min + randomDouble() * (max - min);
}

inline int randomInt(int min, int max) {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<int> dis_int(min, max);
    return dis_int(gen);
}


#endif //VULKANBASE_UTILS_H
