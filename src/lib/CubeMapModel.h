//
// Created by luc on 24/10/23.
//

#ifndef VULKANFLUIDSIMULATION_CUBEMAPMODEL_H
#define VULKANFLUIDSIMULATION_CUBEMAPMODEL_H

#include <vulkan/vulkan.h>
#include "utils.h"
#include "Buffer.h"
#include "Image.h"
#include <GLFW/glfw3.h>
#include <GLFW/glfw3.h>

namespace vkb {
    class CubeMapModel {
    public:
        CubeMapModel(const Device &device, const std::vector<std::string>& filePaths, bool flip = false);
        ~CubeMapModel();

        CubeMapModel(const CubeMapModel &) = delete;
        CubeMapModel &operator=(const CubeMapModel &) = delete;

        [[nodiscard]] VkImage image() const { return m_cubeMapImage->image(); }
        [[nodiscard]] VkImageView view() const { return m_cubeMapImage->view(); }
        [[nodiscard]] VkSampler sampler() const { return m_cubeMapSampler; }
        [[nodiscard]] VkDescriptorImageInfo descriptorInfo() const { return VkDescriptorImageInfo {
                    sampler(), m_cubeMapImage->view(), m_cubeMapImage->layout()
            };
        }
        void bindAndDraw(VkCommandBuffer commandBuffer);

    private:
        static constexpr uint32_t NUM_CUBEMAP_IMAGES = 6;
        static constexpr uint32_t NUM_VERTICES = 36;

        void createCubeMapImage(const std::vector<std::string>& filePaths);
        void createCubeMapSampler();
        void createVertexBuffer();

        const Device& m_deviceRef;
        std::unique_ptr<vkb::Image> m_cubeMapImage;
        std::unique_ptr<vkb::Buffer> m_vertexBuffer;
        bool m_flipImage{};


        VkSampler m_cubeMapSampler{};

        std::vector<glm::vec3> vertices = {
                // positions
                {-1.0f,  1.0f, -1.0f},
                {-1.0f, -1.0f, -1.0f},
                { 1.0f, -1.0f, -1.0f},
                { 1.0f, -1.0f, -1.0f},
                { 1.0f,  1.0f, -1.0f},
                {-1.0f,  1.0f, -1.0f},

                {-1.0f, -1.0f,  1.0f},
                {-1.0f, -1.0f, -1.0f},
                {-1.0f,  1.0f, -1.0f},
                {-1.0f,  1.0f, -1.0f},
                {-1.0f,  1.0f,  1.0f},
                {-1.0f, -1.0f,  1.0f},

                {1.0f, -1.0f, -1.0f},
                {1.0f, -1.0f,  1.0f},
                {1.0f,  1.0f,  1.0f},
                {1.0f,  1.0f,  1.0f},
                {1.0f,  1.0f, -1.0f},
                {1.0f, -1.0f, -1.0f},

                {-1.0f, -1.0f,  1.0f},
                {-1.0f,  1.0f,  1.0f},
                { 1.0f,  1.0f,  1.0f},
                { 1.0f,  1.0f,  1.0f},
                { 1.0f, -1.0f,  1.0f},
                {-1.0f, -1.0f,  1.0f},

                {-1.0f,  1.0f, -1.0f},
                { 1.0f,  1.0f, -1.0f},
                { 1.0f,  1.0f,  1.0f},
                { 1.0f,  1.0f,  1.0f},
                {-1.0f,  1.0f,  1.0f},
                {-1.0f,  1.0f, -1.0f},

                {-1.0f, -1.0f, -1.0f},
                {-1.0f, -1.0f,  1.0f},
                { 1.0f, -1.0f, -1.0f},
                { 1.0f, -1.0f, -1.0f},
                {-1.0f, -1.0f,  1.0f},
                { 1.0f, -1.0f,  1.0f}
        };
    };
}



#endif //VULKANFLUIDSIMULATION_CUBEMAPMODEL_H
