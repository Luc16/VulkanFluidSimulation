//
// Created by luc on 03/03/23.
//

#include "ComputeSystem.h"
#include "SwapChain.h"

namespace vkb {
    ComputeSystem::ComputeSystem(const Device &device) : m_deviceRef(device) {
        m_computeInFlightFences.resize(SwapChain::MAX_FRAMES_IN_FLIGHT);
        m_computeFinishedSemaphores.resize(SwapChain::MAX_FRAMES_IN_FLIGHT);
        m_computeCommandBuffers.resize(SwapChain::MAX_FRAMES_IN_FLIGHT);

        VkSemaphoreCreateInfo semaphoreInfo{};
        semaphoreInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;

        VkFenceCreateInfo fenceInfo{};
        fenceInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
        fenceInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

        for (size_t i = 0; i < SwapChain::MAX_FRAMES_IN_FLIGHT; i++) {
            if (vkCreateSemaphore(device.device(), &semaphoreInfo, nullptr, &m_computeFinishedSemaphores[i]) != VK_SUCCESS ||
                vkCreateFence(device.device(), &fenceInfo, nullptr, &m_computeInFlightFences[i]) != VK_SUCCESS) {
                throw std::runtime_error("failed to create compute synchronization objects for a frame!");
            }
        }

        VkCommandBufferAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.commandPool = m_deviceRef.graphicsCommandPool();
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandBufferCount = (uint32_t)m_computeCommandBuffers.size();

        if (vkAllocateCommandBuffers(device.device(), &allocInfo, m_computeCommandBuffers.data()) != VK_SUCCESS) {
            throw std::runtime_error("failed to allocate compute command buffers!");
        }
    }


    ComputeSystem::~ComputeSystem() {
        vkDestroyPipelineLayout(m_deviceRef.device(), m_computePipelineLayout, nullptr);
        vkDestroyPipeline(m_deviceRef.device(), m_computePipeline, nullptr);
        for (size_t i = 0; i < SwapChain::MAX_FRAMES_IN_FLIGHT; i++) {
            vkDestroySemaphore(m_deviceRef.device(), m_computeFinishedSemaphores[i], nullptr);
            vkDestroyFence(m_deviceRef.device(), m_computeInFlightFences[i], nullptr);
        }
    }

    void ComputeSystem::runCompute(VkDescriptorSet *descriptorSet, uint32_t currentFrame, uint32_t x, uint32_t y, uint32_t z) {
        vkWaitForFences(m_deviceRef.device(), 1, &m_computeInFlightFences[currentFrame], VK_TRUE, UINT64_MAX);

        vkResetFences(m_deviceRef.device(), 1, &m_computeInFlightFences[currentFrame]);

        auto& commandBuffer = m_computeCommandBuffers[currentFrame];

        vkResetCommandBuffer(commandBuffer, /*VkCommandBufferResetFlagBits*/ 0);

        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

        if (vkBeginCommandBuffer(commandBuffer, &beginInfo) != VK_SUCCESS) {
            throw std::runtime_error("failed to begin recording compute command buffer!");
        }

        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_computePipeline);

        vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_computePipelineLayout, 0, 1, descriptorSet, 0, nullptr);

        vkCmdDispatch(commandBuffer, x, y, z);

        if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) {
            throw std::runtime_error("failed to record compute command buffer!");
        }

        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBuffer;
        submitInfo.signalSemaphoreCount = 1;
        submitInfo.pSignalSemaphores = &m_computeFinishedSemaphores[currentFrame];

        if (vkQueueSubmit(m_deviceRef.computeQueue(), 1, &submitInfo, m_computeInFlightFences[currentFrame]) != VK_SUCCESS) {
            throw std::runtime_error("failed to submit compute command buffer!");
        };
    }

    void ComputeSystem::bind(VkCommandBuffer commandBuffer, VkDescriptorSet *descriptorSet) {
        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_computePipeline);
        vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE,
                                m_computePipelineLayout, 0, 1, descriptorSet, 0, nullptr);
    }

    void ComputeSystem::dispatch(VkCommandBuffer commandBuffer, uint32_t x, uint32_t y, uint32_t z) {
        vkCmdDispatch(commandBuffer, x, y, z);
    }

    void ComputeSystem::createPipelineLayout(VkDescriptorSetLayout computeSetLayout) {
        m_layoutCreated = true;
        VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
        pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        pipelineLayoutInfo.setLayoutCount = 1;
        pipelineLayoutInfo.pSetLayouts = &computeSetLayout;

        if (vkCreatePipelineLayout(m_deviceRef.device(), &pipelineLayoutInfo, nullptr, &m_computePipelineLayout) != VK_SUCCESS) {
            throw std::runtime_error("failed to create compute pipeline layout!");
        }
    }

    void ComputeSystem::createPipeline(const std::string& computeShaderPath) {
        if (!m_layoutCreated) throw std::runtime_error("Need to create pipeline layout before creating pipeline!");

        auto computeShaderCode = GraphicsPipeline::readFile(computeShaderPath);

        VkShaderModule computeShaderModule = GraphicsPipeline::createShaderModule(m_deviceRef, computeShaderCode);

        VkPipelineShaderStageCreateInfo computeShaderStageInfo{};
        computeShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        computeShaderStageInfo.stage = VK_SHADER_STAGE_COMPUTE_BIT;
        computeShaderStageInfo.module = computeShaderModule;
        computeShaderStageInfo.pName = "main";

        VkComputePipelineCreateInfo pipelineInfo{};
        pipelineInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
        pipelineInfo.layout = m_computePipelineLayout;
        pipelineInfo.stage = computeShaderStageInfo;

        if (vkCreateComputePipelines(m_deviceRef.device(), VK_NULL_HANDLE, 1,
                                     &pipelineInfo, nullptr, &m_computePipeline) != VK_SUCCESS) {
            throw std::runtime_error("failed to create compute pipeline!");
        }
        vkDestroyShaderModule(m_deviceRef.device(), computeShaderModule, nullptr);


    }
}

