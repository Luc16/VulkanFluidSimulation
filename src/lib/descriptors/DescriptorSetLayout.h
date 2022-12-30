//
// Created by luc on 07/11/22.
//

#ifndef VULKANBASE_DESCRIPTORSETLAYOUT_H
#define VULKANBASE_DESCRIPTORSETLAYOUT_H

#include "../Device.h"

namespace vkb {
    class DescriptorSetLayout {
    public:
        class Builder {
        public:
            explicit Builder(const Device& device): m_deviceRef(device) {}

            Builder& addBinding(VkDescriptorSetLayoutBinding binding);
            [[nodiscard]] DescriptorSetLayout build() const;

        private:
            const Device& m_deviceRef;
            std::unordered_map<uint32_t, VkDescriptorSetLayoutBinding> bindings{};
        };

        DescriptorSetLayout(const Device &device, const std::unordered_map<uint32_t, VkDescriptorSetLayoutBinding>& bindings);
        ~DescriptorSetLayout();
        DescriptorSetLayout(const DescriptorSetLayout &) = delete;
        DescriptorSetLayout &operator=(const DescriptorSetLayout &) = delete;

        [[nodiscard]] VkDescriptorSetLayout descriptorSetLayout() const { return m_layout; }


    private:
        const Device& m_deviceRef;
        VkDescriptorSetLayout m_layout{};
        std::unordered_map<uint32_t, VkDescriptorSetLayoutBinding> m_bindings;

        friend class DescriptorWriter;
    };
}



#endif //VULKANBASE_DESCRIPTORSETLAYOUT_H
