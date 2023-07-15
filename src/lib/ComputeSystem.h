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
        ComputeSystem(const Device &device, std::string shaderPath);
        ~ComputeSystem();
        ComputeSystem(const ComputeSystem &) = delete;
        ComputeSystem &operator=(const ComputeSystem &) = delete;

        void bindAndDispatch(VkCommandBuffer commandBuffer, VkDescriptorSet *descriptorSet, uint32_t x, uint32_t y, uint32_t z);

        void createPipelineWithLayout(VkDescriptorSetLayout computeSetLayout);

    private:

        const Device& m_deviceRef;
        const std::string m_shaderPath;

        VkPipeline m_computePipeline{};
        VkPipelineLayout m_computePipelineLayout{};

        bool m_layoutCreated = false;
        bool m_pipelineCreated = false;

    };
}


#endif //VULKANFLUIDSIMULATION_COMPUTESYSTEM_H
