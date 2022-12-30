//
// Created by luc on 09/11/22.
//

#include "DescriptorPool.h"

namespace vkb {

    DescriptorPool::DescriptorPool(const Device &device, const std::vector<VkDescriptorPoolSize> &poolSizes,
                                   VkDescriptorPoolCreateFlags poolFlags, uint32_t poolMaxSets): m_deviceRef(device) {

        VkDescriptorPoolCreateInfo poolInfo = {};
        poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
        poolInfo.flags = poolFlags;
        poolInfo.maxSets = poolMaxSets;
        poolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size());
        poolInfo.pPoolSizes = poolSizes.data();

        if (vkCreateDescriptorPool(device.device(), &poolInfo, nullptr, &m_descriptorPool) != VK_SUCCESS){
            throw std::runtime_error("failed to create descriptor pool!");
        }
    }

    DescriptorPool::~DescriptorPool() {
        vkDestroyDescriptorPool(m_deviceRef.device(), m_descriptorPool, nullptr);
    }

    void DescriptorPool::allocateDescriptor(VkDescriptorSetLayout descriptorSetLayout,
                                            VkDescriptorSet &descriptor) const {
        VkDescriptorSetAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        allocInfo.descriptorPool = m_descriptorPool;
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &descriptorSetLayout;

        if (vkAllocateDescriptorSets(m_deviceRef.device(), &allocInfo, &descriptor) != VK_SUCCESS) {
            throw std::runtime_error("failed to allocate descriptor sets!");
        }
    }

    void DescriptorPool::freeDescriptors(std::vector<VkDescriptorSet> &descriptors) const {
        vkFreeDescriptorSets(
                m_deviceRef.device(),
                m_descriptorPool,
                static_cast<uint32_t>(descriptors.size()),
                descriptors.data());
    }

    void DescriptorPool::resetPool() {
        vkResetDescriptorPool(m_deviceRef.device(), m_descriptorPool, 0);
    }

    DescriptorPool::Builder &DescriptorPool::Builder::addPoolSize(VkDescriptorPoolSize poolSize) {
        for (const auto &item: poolSizes) {
            if (item.type == poolSize.type){
                throw std::runtime_error("Trying to add two VkDescriptorPoolSize with the same type");
            }
        }
        poolSizes.push_back(poolSize);
        return *this;
    }


    std::unique_ptr<DescriptorPool> DescriptorPool::Builder::build() const {
        return std::make_unique<DescriptorPool>(m_deviceRef, poolSizes, poolFlags, poolMaxSets);
    }
}