//
// Created by luc on 10/11/22.
//

#ifndef VULKANBASE_DESCRIPTORWRITER_H
#define VULKANBASE_DESCRIPTORWRITER_H

#include "DescriptorSetLayout.h"
#include "DescriptorPool.h"

namespace vkb {
    class DescriptorWriter {
    public:
        DescriptorWriter(DescriptorSetLayout& layout, DescriptorPool& pool): m_layout(layout), m_pool(pool) {}

        DescriptorWriter &writeBuffer(uint32_t binding, VkDescriptorBufferInfo *bufferInfo);
        DescriptorWriter &writeImage(uint32_t binding, VkDescriptorImageInfo *imageInfo);

        void build(VkDescriptorSet &set, bool allocateDescriptor=true);
        void update(VkDescriptorSet &set);
    private:
        DescriptorSetLayout& m_layout;
        DescriptorPool& m_pool;
        std::vector<VkWriteDescriptorSet> m_writes;
    };
}



#endif //VULKANBASE_DESCRIPTORWRITER_H
