//
// Created by luc on 23/10/22.
//

#ifndef VULKANBASE_BUFFER_H
#define VULKANBASE_BUFFER_H

#include <vulkan/vulkan.h>
#include <vector>
#include "utils.h"
#include "Device.h"
#include <GLFW/glfw3.h>

namespace vkb {
    class Buffer {
    public:

        Buffer(const Device& device, VkDeviceSize bufferSize, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties);
        Buffer(const Buffer &) = delete;
        Buffer &operator=(const Buffer &) = delete;
        ~Buffer();

        [[nodiscard]] VkBuffer getBuffer() const { return m_buffer; }
        [[nodiscard]] VkDeviceMemory getMemory() const { return m_memory; }
        [[nodiscard]] VkDeviceSize getSize() const { return m_bufferSize; }
        [[nodiscard]] VkDescriptorBufferInfo descriptorInfo(VkDeviceSize size = VK_WHOLE_SIZE, VkDeviceSize offset = 0) const;

        void map();
        void unmap();
        void write(void* data, VkDeviceSize size = VK_WHOLE_SIZE, VkDeviceSize offset = 0);
        void singleWrite(void* data);

    private:
        VkBuffer m_buffer = VK_NULL_HANDLE;
        VkDeviceMemory m_memory = VK_NULL_HANDLE;
        void* m_mapped = nullptr;

        VkDeviceSize m_bufferSize;
        const Device& m_deviceRef;
    };
}



#endif //VULKANBASE_BUFFER_H
