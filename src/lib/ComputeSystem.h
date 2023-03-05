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
        struct ShaderPaths {
            ShaderPaths(std::string vertShaderPath, std::string fragShaderPath) : vertPath(std::move(vertShaderPath)),
                                                                                  fragPath(std::move(fragShaderPath)) {}

            const std::string fragPath;
            const std::string vertPath;
        };

        explicit ComputeSystem(const Device &device);
        ~ComputeSystem();
        ComputeSystem(const ComputeSystem &) = delete;
        ComputeSystem &operator=(const ComputeSystem &) = delete;

        [[nodiscard]] std::vector<VkSemaphore> currentSemaphore(uint32_t currentFrame) { return {m_computeFinishedSemaphores[currentFrame]}; }
        [[nodiscard]] std::vector<VkPipelineStageFlags> waitStages() { return {VK_PIPELINE_STAGE_VERTEX_INPUT_BIT}; }

        void runCompute(VkDescriptorSet *descriptorSet, uint32_t currentFrame, uint32_t x, uint32_t y, uint32_t z);

        void bind(VkCommandBuffer commandBuffer, VkDescriptorSet *descriptorSet);
        void dispatch(VkCommandBuffer commandBuffer, uint32_t x, uint32_t y, uint32_t z);

        void createPipelineLayout(VkDescriptorSetLayout computeSetLayout);

        void createPipeline(const std::string& computeShaderPath);

    private:

        const Device &m_deviceRef;

        VkPipeline m_computePipeline{};
        VkPipelineLayout m_computePipelineLayout{};

        std::vector<VkFence> m_computeInFlightFences;
        std::vector<VkSemaphore> m_computeFinishedSemaphores;

        std::vector<VkCommandBuffer> m_computeCommandBuffers{};

        bool m_layoutCreated = false;

    };
}


#endif //VULKANFLUIDSIMULATION_COMPUTESYSTEM_H
