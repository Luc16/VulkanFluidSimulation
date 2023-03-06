//
// Created by luc on 10/11/22.
//

#include "DescriptorWriter.h"

namespace vkb {

    DescriptorWriter &DescriptorWriter::writeBuffer(uint32_t binding, VkDescriptorBufferInfo *bufferInfo) {
        if (m_layout.m_bindings.count(binding) == 0) throw std::runtime_error("No layout for this binding");

        auto& bindingDescription = m_layout.m_bindings[binding];

        if (bindingDescription.descriptorCount != 1) throw std::runtime_error("Binding single descriptor info, but binding expects multiple");

        VkWriteDescriptorSet write{};
        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        write.dstBinding = binding;
        write.dstArrayElement = 0;
        write.descriptorType = bindingDescription.descriptorType;
        write.descriptorCount = 1;
        write.pBufferInfo = bufferInfo;

        m_writes.push_back(write);
        return *this;
    }

    DescriptorWriter &DescriptorWriter::writeImage(uint32_t binding, VkDescriptorImageInfo *imageInfo) {
        if (m_layout.m_bindings.count(binding) == 0) throw std::runtime_error("No layout for this binding");

        auto& bindingDescription = m_layout.m_bindings[binding];

        if (bindingDescription.descriptorCount != 1) throw std::runtime_error("Binding single descriptor info, but binding expects multiple");

        VkWriteDescriptorSet write{};
        write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
        write.dstBinding = binding;
        write.dstArrayElement = 0;
        write.descriptorType = bindingDescription.descriptorType;
        write.descriptorCount = 1;
        write.pImageInfo = imageInfo;

        m_writes.push_back(write);
        return *this;
    }

    void DescriptorWriter::build(VkDescriptorSet &set, bool allocDescriptor) {
        if (allocDescriptor) m_pool.allocateDescriptor(m_layout.descriptorSetLayout(), set);
        update(set);
    }

    void DescriptorWriter::update(VkDescriptorSet &set) {
        for (auto &write: m_writes) {
            write.dstSet = set;
        }
        vkUpdateDescriptorSets(m_pool.m_deviceRef.device(),
                               static_cast<uint32_t>(m_writes.size()),
                               m_writes.data(), 0, nullptr);
    }
}