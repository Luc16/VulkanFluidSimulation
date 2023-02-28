//
// Created by luc on 07/11/22.
//

#include "DescriptorSetLayout.h"

namespace vkb {

    DescriptorSetLayout::DescriptorSetLayout(const Device &device, const std::unordered_map<uint32_t,
                                             VkDescriptorSetLayoutBinding>& bindings): m_deviceRef(device), m_bindings(bindings) {
        std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings{};
        setLayoutBindings.reserve(bindings.size());
        for (auto kv : bindings) {
            setLayoutBindings.push_back(kv.second);
        }
        VkDescriptorSetLayoutCreateInfo layoutInfo{};
        layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
        layoutInfo.bindingCount = static_cast<uint32_t>(setLayoutBindings.size());
        layoutInfo.pBindings = setLayoutBindings.data();

        if (vkCreateDescriptorSetLayout(device.device(), &layoutInfo, nullptr, &m_layout) != VK_SUCCESS){
            throw std::runtime_error("failed to create descriptor set layout!");
        }
    }

    DescriptorSetLayout::~DescriptorSetLayout() {
        vkDestroyDescriptorSetLayout(m_deviceRef.device(), m_layout, nullptr);
    }

    DescriptorSetLayout::Builder &DescriptorSetLayout::Builder::addBinding(VkDescriptorSetLayoutBinding binding) {
        if (bindings.count(binding.binding) == 1) throw std::runtime_error("Trying to add two VkDescriptorSetLayoutBinding with the same binding");

        bindings[binding.binding] = binding;
        return *this;
    }

    DescriptorSetLayout DescriptorSetLayout::Builder::build() const {
        return {m_deviceRef, bindings};
    }
}