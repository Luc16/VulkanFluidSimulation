//
// Created by luc on 11/01/24.
//

#ifndef VULKANFLUIDSIMULATION_STRUCTS_H
#define VULKANFLUIDSIMULATION_STRUCTS_H

#include <iostream>

constexpr static uint32_t workGroupSize = 256;


struct ComputeUniformBufferObject {
    uint32_t size;
    uint32_t numParticles;
    float overCellSize;
    float cellSize;
    float dt;
    float flipRatio;
    alignas(8) glm::ivec2 dim;
};

struct UniformBufferObject {
    glm::mat4 view;
    glm::mat4 proj;
    float radius;
};

struct Point {
    alignas(16) glm::vec3 position, color;

    static VkVertexInputBindingDescription getBindingDescription() {
        VkVertexInputBindingDescription bindingDescription{};
        bindingDescription.binding = 0;
        bindingDescription.stride = sizeof(Point);
        bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

        return bindingDescription;
    }

    static std::vector<VkVertexInputAttributeDescription> getAttributeDescriptions() {
        std::vector<VkVertexInputAttributeDescription> attributeDescriptions{2};

        attributeDescriptions[0].binding = 0;
        attributeDescriptions[0].location = 0;
        attributeDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
        attributeDescriptions[0].offset = offsetof(Point, position);

        attributeDescriptions[1].binding = 0;
        attributeDescriptions[1].location = 1;
        attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
        attributeDescriptions[1].offset = offsetof(Point, color);

        return attributeDescriptions;
    }
};


struct Line {
    Point p1, p2;

    void setColor(glm::vec3 color) {
        p1.color = color;
        p2.color = color;
    }

    static Line fromRay(glm::vec3 origin, glm::vec3 vec, glm::vec3 color) {
        Line line{};
        line.p1 = {origin, color};
        line.p2 = {origin + vec, color};
        return line;
    }

    static std::array<Line, 2> arrowFromRay(glm::vec3 origin, glm::vec3 vec, glm::vec3 color, glm::vec3 arrowColor) {
        std::array<Line, 2> lines{};
        lines[0] = {{origin}, {origin + vec * 0.8f}};
        lines[1] = {{origin + vec * 0.8f}, {origin + vec}};
        lines[0].setColor(color);
        lines[1].setColor(arrowColor);
        return lines;
    }
};

struct GridQuad {
    GridQuad(glm::vec3 pos, uint32_t gridSpacing, glm::vec3 color) {
        // generate two counterclockwise triangles
        p1 = {pos, color};
        p2 = {pos + glm::vec3(gridSpacing, 0, 0), color};
        p3 = {pos + glm::vec3(gridSpacing, gridSpacing, 0), color};
        p4 = {pos, color};
        p5 = {pos + glm::vec3(gridSpacing, gridSpacing, 0), color};
        p6 = {pos + glm::vec3(0, gridSpacing, 0), color};
    }

    Point p1{}, p2{}, p3{}, p4{}, p5{}, p6{};


};

#endif //VULKANFLUIDSIMULATION_STRUCTS_H
