//
// Created by luc on 28/10/22.
//

#ifndef VULKANBASE_MODEL_H
#define VULKANBASE_MODEL_H

#include <vulkan/vulkan.h>
#include "utils.h"
#include "Buffer.h"
#include <GLFW/glfw3.h>

namespace vkb {
    class Model {
    public:
        struct Vertex {
            glm::vec3 pos;
            glm::vec3 color;
            glm::vec3 normal;
            glm::vec2 texCoord;

            bool operator ==(const Vertex& other) const{
                return pos == other.pos && color == other.color && texCoord == other.texCoord;
            }

            static VkVertexInputBindingDescription getBindingDescription();
            static std::vector<VkVertexInputAttributeDescription> getAttributeDescriptions();
        };

        Model(const Device &device, std::vector<Vertex>& vertices, std::vector<uint32_t>& indices);
        Model(const Device &device, std::vector<Vertex>& vertices);
        ~Model() = default;

        Model(const Model &) = delete;
        Model &operator=(const Model &) = delete;

        static std::unique_ptr<Model> createModelFromFile(const Device &device, const std::string &filepath);

        void bind(VkCommandBuffer commandBuffer);
        void draw(VkCommandBuffer commandBuffer, size_t numInstances = 1) const;
        void updateVertexBuffer(std::vector<Vertex>& vertices, bool createBuffer = false);
    private:
        void createIndexBuffer(std::vector<uint32_t>& indices);

        const Device& m_deviceRef;

        std::unique_ptr<vkb::Buffer> m_vertexBuffer;
        uint32_t m_vertexCount{};

        bool m_hasIndexBuffer = false;
        std::unique_ptr<vkb::Buffer> m_indexBuffer;
        uint32_t m_indexCount{};
    };
}



#endif //VULKANBASE_MODEL_H
