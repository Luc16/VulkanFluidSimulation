//
// Created by luc on 12/03/24.
//

#include "FlipRenderer.h"

void FlipRenderer::createRenderSystems(vkb::DescriptorPool& pool, const vkb::Renderer& renderer, const std::unique_ptr<vkb::Buffer>& uniformBuffer) {
    auto defaultDescriptorLayout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
            .build();
    m_particleDescriptorSets = vkb::VulkanApp::createDescriptorSets(pool, defaultDescriptorLayout,{uniformBuffer->descriptorInfo()});
    m_particleSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), 0);
    m_particleSystem.createPipeline(renderer.renderPass(), {m_shaderPaths[0], m_shaderPaths[1]}, [](vkb::GraphicsPipeline::PipelineConfigInfo& configInfo){
        configInfo.inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;

        configInfo.attributeDescription.clear();
        configInfo.attributeDescription.push_back({0, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 0});
        configInfo.bindingDescription.clear();
        configInfo.bindingDescription.push_back({0, sizeof(glm::vec4), VK_VERTEX_INPUT_RATE_VERTEX});

    });
}

void FlipRenderer::render(VkCommandBuffer& commandBuffer, const FlipSolver& solver, uint32_t frame) {
    m_particleSystem.bind(commandBuffer, &m_particleDescriptorSets[frame]);
    VkBuffer vb = solver.particleBuffer();
    VkDeviceSize offsets[] = {0};
    vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
    vkCmdDraw(commandBuffer, solver.getParticleCount(), 1, 0, 0);
}
