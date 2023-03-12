//
// Created by luc on 06/03/23.
//

#ifndef VULKANFLUIDSIMULATION_COMPUTESHADERHANDLER_H
#define VULKANFLUIDSIMULATION_COMPUTESHADERHANDLER_H

#include <utility>

#include "utils.h"
#include "Device.h"
#include "GraphicsPipeline.h"
#include "Buffer.h"

namespace vkb {
    class ComputeShaderHandler {
    public:
        explicit ComputeShaderHandler(const Device &device);
        ~ComputeShaderHandler();
        ComputeShaderHandler(const ComputeShaderHandler &) = delete;
        ComputeShaderHandler &operator=(const ComputeShaderHandler &) = delete;

        [[nodiscard]] std::vector<VkSemaphore> currentSemaphore(uint32_t currentFrame) { return {m_computeFinishedSemaphores[currentFrame]}; }
        [[nodiscard]] static std::vector<VkPipelineStageFlags> waitStages() { return {VK_PIPELINE_STAGE_VERTEX_INPUT_BIT}; }

        void runCompute(uint32_t currentFrame, const std::function<void(VkCommandBuffer computeCommandBuffer)>& func);

        static void computeBarrier(VkCommandBuffer commandBuffer, const std::vector<std::unique_ptr<Buffer>>& buffers);
        static void computeBarrier(VkCommandBuffer commandBuffer, const std::unique_ptr<Buffer>& buffer);

    private:

        const Device &m_deviceRef;

        std::vector<VkFence> m_computeInFlightFences;
        std::vector<VkSemaphore> m_computeFinishedSemaphores;

        std::vector<VkCommandBuffer> m_computeCommandBuffers{};

    };
}


#endif //VULKANFLUIDSIMULATION_COMPUTESHADERHANDLER_H
