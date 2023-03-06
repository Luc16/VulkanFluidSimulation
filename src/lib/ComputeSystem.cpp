//
// Created by luc on 03/03/23.
//

#include "ComputeSystem.h"
#include "SwapChain.h"

namespace vkb {
    ComputeSystem::ComputeSystem(const Device &device) : m_deviceRef(device) {}


    ComputeSystem::~ComputeSystem() {
        vkDestroyPipelineLayout(m_deviceRef.device(), m_computePipelineLayout, nullptr);
        vkDestroyPipeline(m_deviceRef.device(), m_computePipeline, nullptr);
    }

    void ComputeSystem::bindAndDispatch(VkCommandBuffer commandBuffer, VkDescriptorSet *descriptorSet, uint32_t x, uint32_t y, uint32_t z) {
        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, m_computePipeline);
        vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE,
                                m_computePipelineLayout, 0, 1, descriptorSet, 0, nullptr);
        vkCmdDispatch(commandBuffer, x, y, z);

    }

    void ComputeSystem::createPipelineLayout(VkDescriptorSetLayout computeSetLayout) {
        if (m_layoutCreated) vkDestroyPipelineLayout(m_deviceRef.device(), m_computePipelineLayout, nullptr);
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
        if (m_pipelineCreated) vkDestroyPipeline(m_deviceRef.device(), m_computePipeline, nullptr);
        m_pipelineCreated = true;

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

