//
// Created by luc on 12/03/24.
//

#include "FlipRenderer.h"

void FlipRenderer::runOffscreenPasses(VkCommandBuffer commandBuffer, const FlipSolver &solver, uint32_t currentFrame,
                                      const vkb::CubeMapModel& skybox, const vkb::DrawableObject& plane, vkb::RenderSystem& defaultRenderSystem,
                                      std::vector<VkDescriptorSet>& defaultDescriptorSets, std::vector<VkDescriptorSet>& skyboxDescriptorSets, bool renderSkybox) {
    VkBuffer vb = solver.particleBuffer();
    VkDeviceSize offsets[] = {0};

    m_depthPass.run(commandBuffer, &simulationDescriptorSets[currentFrame],
                  [&vb, &offsets, &solver](VkCommandBuffer &commandBuffer) {
                      vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
                      vkCmdDraw(commandBuffer, solver.getParticleCount(), 1, 0, 0);
                  });

    if (blurIterations > 0) {
        m_smoothPass.run(commandBuffer, &simulationDescriptorSets[currentFrame],
                       [](VkCommandBuffer commandBuffer) {
                           vkCmdDraw(commandBuffer, 3, 1, 0, 0);
                       });
    }

    for (uint32_t _ = 1; _ < blurIterations; _++){
        m_smoothPass.runAlternating(commandBuffer, {&smooth1DescriptorSets[currentFrame], &smooth2DescriptorSets[currentFrame]},
                                  [](VkCommandBuffer commandBuffer) {
                                      vkCmdDraw(commandBuffer, 3, 1, 0, 0);

                                  });
    }

    m_thicknessPass.run(commandBuffer,
                      (blurIterations % 2 == 0) ? &smooth1DescriptorSets[currentFrame] : &smooth2DescriptorSets[currentFrame],
                      [&vb, &offsets, &solver](VkCommandBuffer &commandBuffer) {
                          vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
                          vkCmdDraw(commandBuffer, solver.getParticleCount(), 1, 0, 0);
                      });

    m_scenePass.run(commandBuffer,
                  &defaultDescriptorSets[currentFrame],
                  [&](VkCommandBuffer &commandBuffer) {
                      if (renderSkybox) {
                          m_skyboxTexSystem.bind(commandBuffer, &skyboxDescriptorSets[currentFrame]);
                          skybox.bindAndDraw(commandBuffer);
                      }
                      m_scenePass.bindRenderSystem(commandBuffer, &defaultDescriptorSets[currentFrame]);
                      plane.render(defaultRenderSystem, commandBuffer);
                  });
}

void FlipRenderer::render(VkCommandBuffer& commandBuffer, const FlipSolver& solver, uint32_t frame, uint32_t renderType) {
    if (renderType == 0) {
        m_particleSystem.bind(commandBuffer, &m_particleDescriptorSets[frame]);
        VkBuffer vb = solver.particleBuffer();
        VkDeviceSize offsets[] = {0};
        vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
        vkCmdDraw(commandBuffer, solver.getParticleCount(), 1, 0, 0);
    } else {
        VkDescriptorSet* finalSceneDescriptorSet;
        if (blurIterations == 0) finalSceneDescriptorSet = &shadingDescriptorSets[0][frame];
        else finalSceneDescriptorSet = &shadingDescriptorSets[(blurIterations % 2 == 0) + 1][frame];
        m_shadingRenderSystem.bind(commandBuffer, finalSceneDescriptorSet);
        vkCmdDraw(commandBuffer, 3, 1, 0, 0);
    }


}

void FlipRenderer::initialize(vkb::DescriptorPool& pool, const vkb::Renderer& renderer, const std::unique_ptr<vkb::Buffer>& uniformBuffer,
                              const vkb::CubeMapModel& skybox, const vkb::DrawableObject& plane){
    auto defaultDescriptorLayout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
            .addBinding({1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr})
            .build();
    createOffscreenPasses(defaultDescriptorLayout);

    auto particleDescriptorLayout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
            .build();
    m_particleDescriptorSets = vkb::VulkanApp::createDescriptorSets(pool, particleDescriptorLayout,{uniformBuffer->descriptorInfo()});
    m_particleSystem.createPipelineLayout(particleDescriptorLayout.descriptorSetLayout(), 0);
    m_particleSystem.createPipeline(renderer.renderPass(), {m_shaderPaths[6], m_shaderPaths[7]}, [](vkb::GraphicsPipeline::PipelineConfigInfo& configInfo){
        configInfo.inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;

        configInfo.attributeDescription.clear();
        configInfo.attributeDescription.push_back({0, 0, VK_FORMAT_R32G32B32A32_SFLOAT, 0});
        configInfo.bindingDescription.clear();
        configInfo.bindingDescription.push_back({0, sizeof(glm::vec4), VK_VERTEX_INPUT_RATE_VERTEX});

    });

    vkb::RenderSystem::ShaderPaths shadingShaderPaths = {m_shaderPaths[14], m_shaderPaths[15]};
    auto ssfDescriptorLayout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
            .addBinding({1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr})
            .addBinding({2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr})
            .addBinding({3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr})
            .addBinding({4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr})
            .addBinding({5, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr})
            .addBinding({6, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr})
            .build();

    m_shadingRenderSystem.createPipelineLayout(ssfDescriptorLayout.descriptorSetLayout(), 0);
    m_shadingRenderSystem.createPipeline(renderer.renderPass(), shadingShaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& info) {
        info.bindingDescription.clear();
        info.attributeDescription.clear();
        info.rasterizer.cullMode = VK_CULL_MODE_NONE;
    });

    shadingDescriptorSets[0] = vkb::VulkanApp::createDescriptorSets(pool,
            ssfDescriptorLayout,
            {uniformBuffer->descriptorInfo()},
            {m_depthPass.descriptorInfo(), m_thicknessPass.descriptorInfo(), m_scenePass.descriptorInfo(), m_depthPass.descriptorInfo(),
             plane.textureInfo(), skybox.descriptorInfo()}
    );
    shadingDescriptorSets[1] = vkb::VulkanApp::createDescriptorSets(pool,
            ssfDescriptorLayout,
            {uniformBuffer->descriptorInfo()},
            {m_depthPass.descriptorInfo(), m_thicknessPass.descriptorInfo(), m_scenePass.descriptorInfo(), m_smoothPass.descriptorInfo(),
             plane.textureInfo(), skybox.descriptorInfo()}
    );
    shadingDescriptorSets[2] = vkb::VulkanApp::createDescriptorSets(pool,
            ssfDescriptorLayout,
            {uniformBuffer->descriptorInfo()},
            {m_depthPass.descriptorInfo(), m_thicknessPass.descriptorInfo(), m_scenePass.descriptorInfo(), m_smoothPass.additionalImageDescriptorInfo(),
             plane.textureInfo(), skybox.descriptorInfo()}
    );

    simulationDescriptorSets = vkb::VulkanApp::createDescriptorSets(pool, defaultDescriptorLayout,
                                                    {uniformBuffer->descriptorInfo()}, {m_depthPass.descriptorInfo()});
    smooth1DescriptorSets = vkb::VulkanApp::createDescriptorSets(pool, defaultDescriptorLayout,
                                                 {uniformBuffer->descriptorInfo()}, {m_smoothPass.descriptorInfo()});
    smooth2DescriptorSets = vkb::VulkanApp::createDescriptorSets(pool, defaultDescriptorLayout,
                                                 {uniformBuffer->descriptorInfo()}, {m_smoothPass.additionalImageDescriptorInfo()});

    vkb::RenderSystem::ShaderPaths skyboxShaderPaths = {m_shaderPaths[4], m_shaderPaths[5]};
    m_skyboxTexSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), 0);
    m_skyboxTexSystem.createPipeline(m_scenePass.renderPass(), skyboxShaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& info) {
        info.depthStencilInfo.depthTestEnable = VK_FALSE;
        info.depthStencilInfo.depthWriteEnable = VK_FALSE;
        info.depthStencilInfo.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
        info.colorBlendAttachment.blendEnable = VK_FALSE;
        info.multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;

        info.bindingDescription.clear();
        info.bindingDescription.push_back({0, sizeof(glm::vec3), VK_VERTEX_INPUT_RATE_VERTEX});
        info.attributeDescription.clear();
        info.attributeDescription.push_back({0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0});
    });

}

void FlipRenderer::createOffscreenPasses(const vkb::DescriptorSetLayout& defaultDescriptorLayout) {
    vkb::RenderSystem::ShaderPaths depthShaderPaths = {m_shaderPaths[8], m_shaderPaths[9]};
    m_depthPass.createPass(defaultDescriptorLayout.descriptorSetLayout(), depthShaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& info) {
        info.inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
        info.bindingDescription.clear();
        info.bindingDescription.push_back({0, sizeof(glm::vec4), VK_VERTEX_INPUT_RATE_VERTEX});
        info.attributeDescription.clear();
        info.attributeDescription.push_back({0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0});
        info.multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        info.colorBlending.attachmentCount = 0;
        info.rasterizer.cullMode = VK_CULL_MODE_NONE;
        info.depthStencilInfo.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
        // Enable depth bias
        info.rasterizer.depthBiasEnable = VK_TRUE;
        // Add depth bias to dynamic state, so we can change it at runtime
        info.dynamicStateEnables.push_back(VK_DYNAMIC_STATE_DEPTH_BIAS);
        info.dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
        info.dynamicState.dynamicStateCount = static_cast<uint32_t>(info.dynamicStateEnables.size());
        info.dynamicState.pDynamicStates = info.dynamicStateEnables.data();
    });

    vkb::RenderSystem::ShaderPaths thicknessShaderPaths = {m_shaderPaths[10], m_shaderPaths[11]};
    m_thicknessPass.createPass(defaultDescriptorLayout.descriptorSetLayout(), thicknessShaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& info) {
        info.inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
        info.bindingDescription.clear();
        info.bindingDescription.push_back({0, sizeof(glm::vec4), VK_VERTEX_INPUT_RATE_VERTEX});
        info.attributeDescription.clear();
        info.attributeDescription.push_back({0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0});
        info.multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        info.rasterizer.cullMode = VK_CULL_MODE_NONE;
//            info.enableAlphaBlending();
        // additive blending
        info.colorBlendAttachment.blendEnable = VK_TRUE;
        info.colorBlendAttachment.colorWriteMask =
                VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT |
                VK_COLOR_COMPONENT_A_BIT;
        info.colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
        info.colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE;
        info.colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
        info.colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        info.colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        info.colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;

    });

    vkb::RenderSystem::ShaderPaths shaderPaths = {m_shaderPaths[0], m_shaderPaths[1]};
    m_scenePass.createPass(defaultDescriptorLayout.descriptorSetLayout(), shaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& info){
        info.multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
    }, sizeof(vkb::DrawableObject::PushConstantData));

    vkb::RenderSystem::ShaderPaths smoothShaderPaths = {m_shaderPaths[12], m_shaderPaths[13]};
    m_smoothPass.createPass(defaultDescriptorLayout.descriptorSetLayout(), smoothShaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& info) {
        info.bindingDescription.clear();
        info.attributeDescription.clear();
        info.multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        info.colorBlending.attachmentCount = 0;
        info.rasterizer.cullMode = VK_CULL_MODE_NONE;
        info.depthStencilInfo.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
        // Enable depth bias
        info.rasterizer.depthBiasEnable = VK_TRUE;
        // Add depth bias to dynamic state, so we can change it at runtime
        info.dynamicStateEnables.push_back(VK_DYNAMIC_STATE_DEPTH_BIAS);
        info.dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
        info.dynamicState.dynamicStateCount = static_cast<uint32_t>(info.dynamicStateEnables.size());
        info.dynamicState.pDynamicStates = info.dynamicStateEnables.data();
    });
}


