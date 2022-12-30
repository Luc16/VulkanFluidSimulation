//
// Created by luc on 29/10/22.
//

#ifndef VULKANBASE_TEXTURE_H
#define VULKANBASE_TEXTURE_H

#include <vulkan/vulkan.h>
#include "utils.h"
#include "Buffer.h"
#include "Image.h"
#include <GLFW/glfw3.h>

namespace vkb {
    class Texture {
    public:
        Texture(const Device &device, const std::string& texPath);
        ~Texture();

        Texture(const Texture &) = delete;
        Texture &operator=(const Texture &) = delete;

        [[nodiscard]] VkImage image() const { return m_textureImage->image(); }
        [[nodiscard]] VkImageView view() const { return m_textureImage->view(); }
        [[nodiscard]] VkSampler sampler() const { return m_textureSampler; }
        [[nodiscard]] VkDescriptorImageInfo descriptorInfo() const { return VkDescriptorImageInfo {
            m_textureSampler, m_textureImage->view(), m_textureImage->layout()
            };
        }

    private:

        void createTextureImage(const std::string& texPath);
        void createTextureSampler();
        void generateMipMaps(VkImage image, VkFormat imageFormat, int32_t texWidth, int32_t texHeight, uint32_t mipLevels);

        const Device& m_deviceRef;
        uint32_t m_mipLevels{};
        std::unique_ptr<vkb::Image> m_textureImage;

        VkSampler m_textureSampler{};
    };
}


#endif //VULKANBASE_TEXTURE_H
