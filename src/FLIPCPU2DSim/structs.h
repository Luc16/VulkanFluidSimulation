//
// Created by luc on 11/01/24.
//

#ifndef VULKANFLUIDSIMULATION_STRUCTS_H
#define VULKANFLUIDSIMULATION_STRUCTS_H

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
};

#endif //VULKANFLUIDSIMULATION_STRUCTS_H
