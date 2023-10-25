//
// Created by luc on 26/10/22.
//

#ifndef VULKANBASE_IMAGE_H
#define VULKANBASE_IMAGE_H

#include <vulkan/vulkan.h>
#include "Device.h"

namespace vkb {
    class Image {
    public:
        Image(const Device& device, uint32_t width, uint32_t height, uint32_t mipLevels, VkSampleCountFlagBits numSamples,
              VkFormat format, VkImageTiling tiling, VkImageUsageFlags usage, VkMemoryPropertyFlags properties, VkImageAspectFlags aspectFlags,
              bool createView = true, VkImageCreateFlags flags = 0, uint32_t arrayLayers = 1);
        Image(const Image &) = delete;
        Image &operator=(const Image &) = delete;
        ~Image();

        [[nodiscard]] VkImage image() const { return m_image; }
        [[nodiscard]] VkImageView view() const { return m_imageView; }
        [[nodiscard]] VkImageLayout layout() const { return m_layout; }
        void setLayout(VkImageLayout layout) { m_layout = layout; }


        void transitionLayout(VkFormat format, VkImageLayout oldLayout, VkImageLayout newLayout, uint32_t mipLevels, bool useGraphicsQueue = false);
        static VkImageView createSwapChainImageView(VkDevice device, VkImage image, VkFormat format, VkImageAspectFlags aspectFlags, uint32_t mipLevels);

        void createImageView(VkFormat format, VkImageAspectFlags aspectFlags, uint32_t mipLevels);
        void createImageView(const VkImageViewCreateInfo& viewInfo);
    private:
        void createImage(const Device &device, uint32_t width, uint32_t height, uint32_t mipLevels,
                         VkSampleCountFlagBits numSamples, VkFormat format, VkImageTiling tiling,
                         VkImageUsageFlags usage, VkMemoryPropertyFlags properties, VkImageCreateFlags flags,
                         uint32_t arrayLayers);
        static bool hasStencilComponent(VkFormat format);

        VkImage m_image{};
        VkDeviceMemory m_imageMemory{};
        VkImageView m_imageView{};
        VkImageLayout m_layout{};
        uint32_t m_layerCount{};

        const vkb::Device &m_deviceRef;
    };
}



#endif //VULKANBASE_IMAGE_H
