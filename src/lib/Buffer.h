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
        [[nodiscard]] std::pair<VkBuffer, VkDeviceSize> getBarrierData() { return {m_buffer, m_bufferSize}; };


        void map();
        void unmap();
        void write(void* data, VkDeviceSize size = VK_WHOLE_SIZE, VkDeviceSize offset = 0);
        void singleWrite(void* data);
        void singleRead(void* data);

        template<typename T>
        static void writeVectorToBuffer(const Device& device, const std::unique_ptr<Buffer>& buffer, std::vector<T>& vec);
        template<typename T>
        static void writeBufferToVector(const Device& device, const std::unique_ptr<Buffer>& buffer, std::vector<T>& vec);

    private:
        VkBuffer m_buffer = VK_NULL_HANDLE;
        VkDeviceMemory m_memory = VK_NULL_HANDLE;
        void* m_mapped = nullptr;

        VkDeviceSize m_bufferSize;
        const Device& m_deviceRef;
    };

    template<typename T>
    void Buffer::writeVectorToBuffer(const Device& device, const std::unique_ptr<Buffer>& buffer, std::vector<T>& vec) {
        VkDeviceSize bufferSize = vec.size() * sizeof(T);

        if (buffer->getSize() < bufferSize) {
            throw std::runtime_error("Buffer too small to receive data");
        }

        vkb::Buffer stagingBuffer(device, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                  VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

        stagingBuffer.singleWrite(vec.data());

        device.copyBuffer(stagingBuffer.getBuffer(), buffer->getBuffer(), bufferSize);
    }

    template<typename T>
    void Buffer::writeBufferToVector(const Device& device, const std::unique_ptr<Buffer>& buffer, std::vector<T>& vec) {
        VkDeviceSize bufferSize = vec.size() * sizeof(T);

        if (bufferSize < buffer->getSize()) {
            throw std::runtime_error("Vector too small to receive data, vector has " +
                                     std::to_string(bufferSize) +
                                     " bytes, buffer has " +
                                     std::to_string(buffer->getSize()) +
                                     " bytes");
        }

        vkb::Buffer stagingBuffer(device, buffer->getSize(), VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                  VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

        device.copyBuffer(buffer->getBuffer(), stagingBuffer.getBuffer(), buffer->getSize());

        stagingBuffer.singleRead(vec.data());
    }
}





#endif //VULKANBASE_BUFFER_H
