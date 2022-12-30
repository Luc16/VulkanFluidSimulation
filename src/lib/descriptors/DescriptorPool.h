//
// Created by luc on 09/11/22.
//

#ifndef VULKANBASE_DESCRIPTORPOOL_H
#define VULKANBASE_DESCRIPTORPOOL_H

#include "../Device.h"

namespace vkb {
    class DescriptorPool {
    public:
        class Builder {
        public:
            explicit Builder(const Device& device): m_deviceRef(device) {}

            Builder& addPoolSize(VkDescriptorPoolSize poolSize);
            Builder& setFlags(VkDescriptorPoolCreateFlags flags) { poolFlags = flags; return *this; }
            Builder& setMaxSets(uint32_t maxSets) { poolMaxSets = maxSets; return *this; }
            Builder& setMaxSetsTimesSizes(uint32_t maxSets) { poolMaxSets = maxSets*poolSizes.size(); return *this; }
            [[nodiscard]] std::unique_ptr<DescriptorPool> build() const;

        private:
            const Device& m_deviceRef;
            std::vector<VkDescriptorPoolSize> poolSizes{};
            VkDescriptorPoolCreateFlags poolFlags = 0;
            uint32_t poolMaxSets = 1000;
        };

        DescriptorPool(const Device &device, const std::vector<VkDescriptorPoolSize>& poolSizes, VkDescriptorPoolCreateFlags poolFlags, uint32_t poolMaxSets);
        ~DescriptorPool();
        DescriptorPool(const DescriptorPool &) = delete;
        DescriptorPool &operator=(const DescriptorPool &) = delete;

        [[nodiscard]] VkDescriptorPool descriptorPool() const { return m_descriptorPool; }

        void allocateDescriptor(VkDescriptorSetLayout descriptorSetLayout, VkDescriptorSet &descriptor) const;
        void freeDescriptors(std::vector<VkDescriptorSet> &descriptors) const;
        void resetPool();
    private:
        const Device& m_deviceRef;

        VkDescriptorPool m_descriptorPool{};

        friend class DescriptorWriter;
    };
}

#endif //VULKANBASE_DESCRIPTORPOOL_H
