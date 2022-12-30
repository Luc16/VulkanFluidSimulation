//
// Created by luc on 13/11/22.
//

#include "RenderSystem.h"
namespace vkb {
    RenderSystem::RenderSystem(const Device& device): m_deviceRef(device) {}

    RenderSystem::~RenderSystem() {
        vkDestroyPipelineLayout(m_deviceRef.device(), m_pipelineLayout, nullptr);
    }

    void RenderSystem::bind(VkCommandBuffer commandBuffer, VkDescriptorSet* descriptorSet) {
        m_pipeline->bind(commandBuffer);
        vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipelineLayout,
                                0, 1, descriptorSet, 0, nullptr);
    }

    void RenderSystem::createPipelineLayout(VkDescriptorSetLayout globalSetLayout, uint32_t pushConstantSize) {
        m_pushConstantSize = pushConstantSize;
        VkPushConstantRange pushConstantRange{};
        pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
        pushConstantRange.offset = 0;
        pushConstantRange.size = pushConstantSize;

        std::vector<VkDescriptorSetLayout> descriptorSetLayouts{globalSetLayout};
        VkPipelineLayoutCreateInfo pipelineLayoutInfo{};
        pipelineLayoutInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
        pipelineLayoutInfo.setLayoutCount = static_cast<uint32_t>(descriptorSetLayouts.size());
        pipelineLayoutInfo.pSetLayouts = descriptorSetLayouts.data();
        pipelineLayoutInfo.pushConstantRangeCount = (pushConstantSize > 0);
        pipelineLayoutInfo.pPushConstantRanges = (pushConstantSize > 0) ? &pushConstantRange : nullptr;

        if (vkCreatePipelineLayout(m_deviceRef.device(), &pipelineLayoutInfo, nullptr, &m_pipelineLayout) != VK_SUCCESS) {
            throw std::runtime_error("failed to create pipeline layout!");
        }

        m_layoutCreated = true;
    }

    void RenderSystem::createPipeline(VkRenderPass renderPass, const ShaderPaths& shaderPaths,
                                      const std::function<void(Pipeline::PipelineConfigInfo&)>& configurePipeline) {
        if (!m_layoutCreated) throw std::runtime_error("Need to create pipeline layout before creating pipeline!");

        auto pipelineConfigInfo = Pipeline::defaultConfigInfo(m_pipelineLayout, renderPass);

        if (configurePipeline) configurePipeline(pipelineConfigInfo);

        m_pipeline = std::make_unique<Pipeline>(m_deviceRef, shaderPaths.vertPath,
                                                     shaderPaths.fragPath, pipelineConfigInfo);
    }
}

