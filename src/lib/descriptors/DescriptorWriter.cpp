//
// Created by luc on 10/11/22.
//

#include "DescriptorWriter.h"

namespace vkb {

    DescriptorWriter &DescriptorWriter::writeBuffer(uint32_t binding, VkDescriptorBufferInfo *bufferInfo) {
        if (m_layout.m_bindings.count(binding) == 0)
            throw std::runtime_error("No layout for this binding");

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

    VkDescriptorSet DescriptorWriter::createSingleDescriptorSet(const std::unique_ptr<DescriptorPool>& pool,
                                                              DescriptorSetLayout &layout,
                                                              std::vector<VkDescriptorBufferInfo> bufferInfos) {
        VkDescriptorSetLayout setLayout = layout.descriptorSetLayout();
        VkDescriptorSetAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        allocInfo.descriptorPool = pool->m_descriptorPool;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &setLayout;

        VkDescriptorSet set;
        if (vkAllocateDescriptorSets(pool->m_deviceRef.device(), &allocInfo, &set) != VK_SUCCESS) {
            throw std::runtime_error("failed to allocate descriptor sets!");
        }


        auto writer = DescriptorWriter(layout, *pool);
        for (uint32_t i = 0; i < bufferInfos.size(); i++){
            writer.writeBuffer(i, &bufferInfos[i]);
        }

        writer.build(set, false);

        return set;

    }
}