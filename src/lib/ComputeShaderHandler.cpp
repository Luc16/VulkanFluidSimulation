//
// Created by luc on 06/03/23.
//

#include "ComputeShaderHandler.h"
#include "SwapChain.h"

namespace vkb {
    ComputeShaderHandler::ComputeShaderHandler(const Device &device): m_deviceRef(device) {
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

    ComputeShaderHandler::~ComputeShaderHandler() {
        for (size_t i = 0; i < SwapChain::MAX_FRAMES_IN_FLIGHT; i++) {
            vkDestroySemaphore(m_deviceRef.device(), m_computeFinishedSemaphores[i], nullptr);
            vkDestroyFence(m_deviceRef.device(), m_computeInFlightFences[i], nullptr);
        }
    }

    void ComputeShaderHandler::runCompute(uint32_t currentFrame, const std::function<void(VkCommandBuffer computeCommandBuffer)>& func) {
        vkWaitForFences(m_deviceRef.device(), 1, &m_computeInFlightFences[currentFrame], VK_TRUE, UINT64_MAX);

        vkResetFences(m_deviceRef.device(), 1, &m_computeInFlightFences[currentFrame]);

        auto& commandBuffer = m_computeCommandBuffers[currentFrame];

        vkResetCommandBuffer(commandBuffer, /*VkCommandBufferResetFlagBits*/ 0);

        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

        if (vkBeginCommandBuffer(commandBuffer, &beginInfo) != VK_SUCCESS) {
            throw std::runtime_error("failed to begin recording compute command buffer!");
        }

        func(commandBuffer);

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

    void ComputeShaderHandler::computeBarrier(VkCommandBuffer commandBuffer, const std::vector<std::unique_ptr<Buffer>>& buffers) {
        std::array<VkBufferMemoryBarrier, vkb::SwapChain::MAX_FRAMES_IN_FLIGHT> bufferMemoryBarriers{};
        for (uint32_t i = 0; i < buffers.size(); i++){
            bufferMemoryBarriers[i].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
            bufferMemoryBarriers[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            bufferMemoryBarriers[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            bufferMemoryBarriers[i].buffer = buffers[i]->getBuffer();
            bufferMemoryBarriers[i].size = buffers[i]->getSize();
            bufferMemoryBarriers[i].srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
            bufferMemoryBarriers[i].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        }

        vkCmdPipelineBarrier(
                commandBuffer,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                0,
                0, nullptr,
                bufferMemoryBarriers.size(), bufferMemoryBarriers.data(),
                0, nullptr);
    }

    void ComputeShaderHandler::computeBarrier(VkCommandBuffer commandBuffer, const std::unique_ptr<Buffer>& buffer) {
        VkBufferMemoryBarrier bufferMemoryBarrier{};
        bufferMemoryBarrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
        bufferMemoryBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        bufferMemoryBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        bufferMemoryBarrier.buffer = buffer->getBuffer();
        bufferMemoryBarrier.size = buffer->getSize();
        bufferMemoryBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
        bufferMemoryBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        vkCmdPipelineBarrier(
                commandBuffer,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                0,
                0, nullptr,
                1, &bufferMemoryBarrier,
                0, nullptr);
    }

}
