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
    alignas(16) glm::ivec3 dim;
};

struct UniformBufferObject {
    alignas(16) glm::mat4 view;
    alignas(16) glm::mat4 proj;
    alignas(16) glm::mat4 inverseView;
    alignas(16) glm::vec3 lightDir = glm::vec3(1.0f, -1.0f, 0.0f);
    float radius;
    float screenHeight;
    float screenWidth;
    float tanHalfFov = std::tan(glm::radians(50.0f)/2);
    uint32_t renderType = 8;
    float zNear = 0.1f;
    float zFar = 500.0f;
    uint32_t blurMode = 0;
    int filterRadius = 5;
    float blurScale = 0.2;
    float blurDepthFalloff = 1000;
    alignas(16) glm::vec3 planeSize;
    float transparency = 0.8f;
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

#endif //VULKANFLUIDSIMULATION_STRUCTS_H
