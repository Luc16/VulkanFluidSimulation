//
// Created by luc on 03/03/23.
//

#ifndef VULKANFLUIDSIMULATION_COMPUTESYSTEM_H
#define VULKANFLUIDSIMULATION_COMPUTESYSTEM_H

#include <utility>

#include "utils.h"
#include "Device.h"
#include "GraphicsPipeline.h"

namespace vkb {
    class ComputeSystem {
    public:
        explicit ComputeSystem(const Device &device);
        ~ComputeSystem();
        ComputeSystem(const ComputeSystem &) = delete;
        ComputeSystem &operator=(const ComputeSystem &) = delete;

        void bindAndDispatch(VkCommandBuffer commandBuffer, VkDescriptorSet *descriptorSet, uint32_t x, uint32_t y, uint32_t z);

        void createPipelineLayout(VkDescriptorSetLayout computeSetLayout);

        void createPipeline(const std::string& computeShaderPath);

    private:

        const Device &m_deviceRef;

        VkPipeline m_computePipeline{};
        VkPipelineLayout m_computePipelineLayout{};

        bool m_layoutCreated = false;
        bool m_pipelineCreated = false;

    };
}


#endif //VULKANFLUIDSIMULATION_COMPUTESYSTEM_H
