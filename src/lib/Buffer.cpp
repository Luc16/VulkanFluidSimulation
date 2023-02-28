//
// Created by luc on 23/10/22.
//

#include "Buffer.h"

namespace vkb {

    Buffer::Buffer(const Device& device, VkDeviceSize bufferSize, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties):
    m_bufferSize(bufferSize), m_deviceRef(device)
    {
        m_deviceRef.createBuffer(bufferSize, usage, properties, m_buffer, m_memory);
    }

    Buffer::~Buffer() {
        vkDestroyBuffer(m_deviceRef.device(), m_buffer, nullptr);
        vkFreeMemory(m_deviceRef.device(), m_memory, nullptr);
    }

    VkDescriptorBufferInfo Buffer::descriptorInfo(VkDeviceSize size, VkDeviceSize offset) const {
        return VkDescriptorBufferInfo{
                m_buffer, offset, size
        };
    }

    void Buffer::map() {
        if (m_mapped != nullptr)
            throw std::runtime_error("Trying to map mapped buffer memory!");
        auto res = vkMapMemory(m_deviceRef.device(), m_memory, 0, m_bufferSize, 0, &m_mapped);
        if (res != VK_SUCCESS || m_mapped == nullptr)
            throw std::runtime_error("Failed to map memory");

    }

    void Buffer::unmap() {
        if (m_mapped == nullptr)
            throw std::runtime_error("Trying to unmap unmapped buffer memory!");
        vkUnmapMemory(m_deviceRef.device(), m_memory);
        m_mapped = nullptr;
    }

    void Buffer::write(void *data, VkDeviceSize size, VkDeviceSize offset) {
        if (m_mapped == nullptr)
            throw std::runtime_error("Trying to write to unmapped buffer memory!");

        if (size == VK_WHOLE_SIZE)
            memcpy(m_mapped, data, (size_t) m_bufferSize);
        else {
            char *memOffset = (char *)m_mapped;
            memOffset += offset;
            memcpy(m_mapped, data, (size_t) size);

        }
    }

    void Buffer::singleWrite(void *data) {
        map();
        write(data);
        unmap();
    }
}