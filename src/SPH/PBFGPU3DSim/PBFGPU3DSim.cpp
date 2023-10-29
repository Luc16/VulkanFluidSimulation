//
// Created by luc on 01/06/23.
//

#include "PBFGPU3DSim.h"

void PBFGPU3DSim::onCreate() {
    vkDeviceWaitIdle(device.device());
    compileShaders();
//    camera.m_translation = {-174.012f, 194.745f, 193.005f};
//    camera.m_rotation = {0.569391f, 1.95856f, 3.14159f};
    camera.m_translation = {-9.31845f, 14.2878f, 15.5649f};
    camera.m_rotation = {0.569391f, 2.26887f, 3.14159f};
    camera.updateView();
    gUbo.radius = cUbo.H;
    gUbo.screenHeight = (float) window.height();
    gUbo.screenWidth = (float) window.width();
    auto extent = window.extent();
    camera.setOrthographicProjection(0.0f, (float) extent.width, (float) extent.height, 0.0f, gUbo.zNear, gUbo.zFar);
    createUniformBuffers();
    initializeObjects(true);

    auto defaultDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
            .addBinding({1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr})
            .build();
    defaultDescriptorSets = createDescriptorSets(defaultDescriptorLayout,
                                                 {graphicsUniformBuffers[0]->descriptorInfo()}, {plane.textureInfo()});
    {
        defaultSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(),
                                           sizeof(vkb::DrawableObject::PushConstantData));
        defaultSystem.createPipeline(renderer.renderPass(), shaderPaths);
    }

    {
        particleSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), 0);
        particleSystem.createPipeline(renderer.renderPass(), particleShaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& info) {
            info.inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
            info.bindingDescription.clear();
            info.bindingDescription.push_back({0, sizeof(glm::vec4), VK_VERTEX_INPUT_RATE_VERTEX});
            info.bindingDescription.push_back({1, sizeof(float), VK_VERTEX_INPUT_RATE_VERTEX});
            info.attributeDescription.clear();
            info.attributeDescription.push_back({0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0});
            info.attributeDescription.push_back({1, 1, VK_FORMAT_R32_SFLOAT, 0});
        });
    }

    {
        skyboxSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), 0);
        skyboxSystem.createPipeline(renderer.renderPass(), skyboxShaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& info) {
            info.depthStencilInfo.depthTestEnable = VK_FALSE;
            info.depthStencilInfo.depthWriteEnable = VK_FALSE;
            info.depthStencilInfo.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
            info.colorBlendAttachment.blendEnable = VK_FALSE;

            info.bindingDescription.clear();
            info.bindingDescription.push_back({0, sizeof(glm::vec3), VK_VERTEX_INPUT_RATE_VERTEX});
            info.attributeDescription.clear();
            info.attributeDescription.push_back({0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0});
        });
    }

    // create offscreen passes
    {
        depthPass.createPass(defaultDescriptorLayout.descriptorSetLayout(), depthShaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& info) {
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

        thicknessPass.createPass(defaultDescriptorLayout.descriptorSetLayout(), thicknessShaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& info) {
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

        normalsPass.createPass(defaultDescriptorLayout.descriptorSetLayout(), normalsShaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& info) {
            info.bindingDescription.clear();
            info.attributeDescription.clear();
            info.multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
            info.rasterizer.cullMode = VK_CULL_MODE_NONE;
        });

        smoothPass.createPass(defaultDescriptorLayout.descriptorSetLayout(), smoothShaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& info) {
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

    // debug render system
    {
        auto debugDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
                .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
                .addBinding({1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr})
                .addBinding({2, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr})
                .addBinding({3, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr})
                .addBinding({4, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr})
                .addBinding({5, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr})
                .addBinding({6, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr})
                .build();

        debugRenderSystem.createPipelineLayout(debugDescriptorLayout.descriptorSetLayout(), 0);
        debugRenderSystem.createPipeline(renderer.renderPass(), quadShaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& info) {
            info.bindingDescription.clear();
            info.attributeDescription.clear();
            info.rasterizer.cullMode = VK_CULL_MODE_NONE;
        });
        debugDescriptorSets = createDescriptorSets(
                debugDescriptorLayout,
                {graphicsUniformBuffers[0]->descriptorInfo()},
                {depthPass.descriptorInfo(), thicknessPass.descriptorInfo(), normalsPass.descriptorInfo(), smoothPass.descriptorInfo(),
                 plane.textureInfo(), skybox.descriptorInfo()}
        );

    }

    skyboxDescriptorSets = createDescriptorSets(defaultDescriptorLayout,
                                                {graphicsUniformBuffers[0]->descriptorInfo()}, {skybox.descriptorInfo()});

    simulationDescriptorSets = createDescriptorSets(defaultDescriptorLayout,
                                                    {graphicsUniformBuffers[0]->descriptorInfo()}, {depthPass.descriptorInfo()});
    smooth1DescriptorSets = createDescriptorSets(defaultDescriptorLayout,
                                                 {graphicsUniformBuffers[0]->descriptorInfo()}, {smoothPass.descriptorInfo()});
    smooth2DescriptorSets = createDescriptorSets(defaultDescriptorLayout,
                                                 {graphicsUniformBuffers[0]->descriptorInfo()}, {smoothPass.additionalImageDescriptorInfo()});



    // create compute systems and descriptors
    {
        createComputeDescriptorSets();

        predictPositionKernel.createPipeline();
        lambdaKernel.createPipeline();
        posCorrectionKernel.createPipeline();
        updateVelocitiesKernel.createPipeline();
        applyViscAndComputeVortKernel.createPipeline();
        applyVortAndUpdatePos.createPipeline();

        gridHandler.createSystems();
    }
}

void PBFGPU3DSim::createComputeDescriptorSets() {
    for (uint32_t i = 0; i < positionBuffers.size(); i++) {
        predictPositionKernel.descSets[i] = vkb::DescriptorWriter::createSingleDescriptorSet(globalDescriptorPool, predictPositionKernel.layout, {
                {computeUniformBuffer->descriptorInfo()},
                {positionBuffers[i]->descriptorInfo()},
                {velocityBuffers[i]->descriptorInfo()},
                {predPosBuffers[i]->descriptorInfo()},
        });

        lambdaKernel.descSets[i] = vkb::DescriptorWriter::createSingleDescriptorSet(globalDescriptorPool, lambdaKernel.layout, {
                {computeUniformBuffer->descriptorInfo()},
                {gridHandler.gridDescriptorInfo()},
                {predPosBuffers[(i + 1) % predPosBuffers.size()]->descriptorInfo()},
                {lambdaBuffer->descriptorInfo()},
                {densityBuffer->descriptorInfo()},
                {gridIdxBuffer->descriptorInfo()},
        });

        posCorrectionKernel.descSets[i] = vkb::DescriptorWriter::createSingleDescriptorSet(globalDescriptorPool, posCorrectionKernel.layout, {
                {computeUniformBuffer->descriptorInfo()},
                {gridHandler.gridDescriptorInfo()},
                {predPosBuffers[(i + 1) % predPosBuffers.size()]->descriptorInfo()},
                {correctionBuffer->descriptorInfo()},
                {lambdaBuffer->descriptorInfo()},
                {gridIdxBuffer->descriptorInfo()},
        });

        updateVelocitiesKernel.descSets[i] = vkb::DescriptorWriter::createSingleDescriptorSet(globalDescriptorPool, updateVelocitiesKernel.layout, {
                {computeUniformBuffer->descriptorInfo()},
                {positionBuffers[(i + 1) % positionBuffers.size()]->descriptorInfo()},
                {velocityBuffers[(i + 1) % velocityBuffers.size()]->descriptorInfo()},
                {predPosBuffers[(i + 1) % predPosBuffers.size()]->descriptorInfo()},
        });

        applyViscAndComputeVortKernel.descSets[i] = vkb::DescriptorWriter::createSingleDescriptorSet(globalDescriptorPool, applyViscAndComputeVortKernel.layout, {
                {computeUniformBuffer->descriptorInfo()},
                {gridHandler.gridDescriptorInfo()},
                {predPosBuffers[(i + 1) % predPosBuffers.size()]->descriptorInfo()},
                {velocityBuffers[(i + 1) % velocityBuffers.size()]->descriptorInfo()},
                {correctionBuffer->descriptorInfo()},
                {vorticityBuffer->descriptorInfo()},
                {gridIdxBuffer->descriptorInfo()},
        });

        applyVortAndUpdatePos.descSets[i] = vkb::DescriptorWriter::createSingleDescriptorSet(globalDescriptorPool, applyVortAndUpdatePos.layout, {
                {computeUniformBuffer->descriptorInfo()},
                {gridHandler.gridDescriptorInfo()},
                {predPosBuffers[(i + 1) % predPosBuffers.size()]->descriptorInfo()},
                {positionBuffers[(i + 1) % positionBuffers.size()]->descriptorInfo()},
                {velocityBuffers[(i + 1) % velocityBuffers.size()]->descriptorInfo()},
                {correctionBuffer->descriptorInfo()},
                {vorticityBuffer->descriptorInfo()},
                {gridIdxBuffer->descriptorInfo()},
        });
    }
}

void PBFGPU3DSim::initializeObjects(bool activateRandomOffsets) {
    gUbo.view = camera.getView();
    gUbo.inverseView = glm::inverse(gUbo.view);
    gUbo.proj = camera.getProjection();
    gUbo.planeSize = cUbo.BOUNDARY_SIZE;
    cUbo.numParticles = INSTANCE_COUNT;
    GRID_SIZE = uint32_t(cUbo.BOUNDARY_SIZE.x/cUbo.H)*uint32_t(cUbo.BOUNDARY_SIZE.y/cUbo.H)*uint32_t(cUbo.BOUNDARY_SIZE.z/cUbo.H) + 1;

    vkDeviceWaitIdle(device.device());

    particles.resize(INSTANCE_COUNT);

    plane.setScale(cUbo.BOUNDARY_SIZE);

    auto accPos = initialPos;

    uint32_t count = 0;
    for (uint32_t i = 0; i < particles.position.size(); i++) {
        particles.position[i] = accPos;
        if (activateRandomOffsets) particles.position[i] += glm::vec4(
                    randomFloat((accPos.x != cUbo.EPS) ? -cUbo.H/5 : 0.0f, cUbo.H/5),
                    randomFloat((accPos.y != cUbo.EPS) ? -cUbo.H/5 : 0.0f, cUbo.H/5),
                    randomFloat((accPos.z != cUbo.EPS) ? -cUbo.H/5 : 0.0f, cUbo.H/5),
                    0.0f);
        particles.velocity[i] = glm::vec4(0.0f);
        accPos.x += particleSpacing;

        if (i % numParticlesXZ.x == numParticlesXZ.x - 1) {
            count++;
            accPos.z += particleSpacing;
            accPos.x = initialPos.x;
            if (count == numParticlesXZ.y) {
                count = 0;
                accPos.y += particleVerticalSpacing;
                accPos.z = initialPos.z;
            }
        }
    }

    computeFrameIdx = 0;

    //create buffers
    {
        auto makeBuffer = [this](size_t size) {
            return std::make_unique<vkb::Buffer>(device, size,
                                                 VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        };

        for (uint32_t i = 0; i < positionBuffers.size(); i++){
            positionBuffers[i] = std::make_unique<vkb::Buffer>(device, particles.position.size() * sizeof(glm::vec4),
                                                               VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                                               VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
            velocityBuffers[i] = makeBuffer(particles.velocity.size() * sizeof(glm::vec4));
            predPosBuffers[i] = makeBuffer(particles.predPos.size() * sizeof(glm::vec4));
            if (i == 0) {
                vkb::Buffer::writeVectorToBuffer(device, positionBuffers[i], particles.position);
                vkb::Buffer::writeVectorToBuffer(device, velocityBuffers[i], particles.velocity);
            }

        }
        correctionBuffer = makeBuffer(particles.correction.size() * sizeof(glm::vec4));
        vorticityBuffer = makeBuffer(particles.vorticity.size()*sizeof(glm::vec4));
        densityBuffer = std::make_unique<vkb::Buffer>(device, particles.density.size() * sizeof(float),
                                                      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                                      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        lambdaBuffer = makeBuffer(particles.lambda.size()*sizeof(float));
        gridIdxBuffer = makeBuffer(particles.idx.size() * sizeof(uint32_t));
    }

    // create barriers
    for (uint32_t i = 0; i < particleBarrierData.size(); i++){
        particleBarrierData[i] = {
                positionBuffers[(i+1) % positionBuffers.size()]->getBarrierData(),
                velocityBuffers[(i+1) % velocityBuffers.size()]->getBarrierData(),
                predPosBuffers[(i+1) % predPosBuffers.size()]->getBarrierData(),
                correctionBuffer->getBarrierData(),
                vorticityBuffer->getBarrierData(),
                densityBuffer->getBarrierData(),
                lambdaBuffer->getBarrierData(),
        };
    }


    if (activateRandomOffsets){
        gridHandler.createDescriptorsAndBuffers(
                globalDescriptorPool,
                GRID_SIZE,
                {cUbo.numParticles, cUbo.H, cUbo.BOUNDARY_SIZE},
                gridIdxBuffer,
                positionBuffers,
                velocityBuffers,
                predPosBuffers);
    }

    //free descriptor sets
    if (objectsInitialized) {
        globalDescriptorPool->freeDescriptors({
            predictPositionKernel.descSets[0],
            predictPositionKernel.descSets[1],
            lambdaKernel.descSets[0],
            lambdaKernel.descSets[1],
            posCorrectionKernel.descSets[0],
            posCorrectionKernel.descSets[1],
            updateVelocitiesKernel.descSets[0],
            updateVelocitiesKernel.descSets[1],
            applyViscAndComputeVortKernel.descSets[0],
            applyViscAndComputeVortKernel.descSets[1],
            applyVortAndUpdatePos.descSets[0],
            applyVortAndUpdatePos.descSets[1],
        });
    }
    createComputeDescriptorSets();

    std::vector<uint32_t> idxs(INSTANCE_COUNT);
    std::iota(std::begin(idxs), std::end(idxs), 0);
    idxBuffer = std::make_unique<vkb::Buffer>(device, idxs.size() * sizeof(uint32_t),
                                              VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                              VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    vkb::Buffer::writeVectorToBuffer(device, idxBuffer, idxs);

    objectsInitialized = true;

}

void PBFGPU3DSim::createUniformBuffers() {
    VkDeviceSize cBufferSize = sizeof(ComputeUniformBufferObject);

    computeUniformBuffer = std::make_unique<vkb::Buffer>(device, cBufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                             VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    computeUniformBuffer->map();


    VkDeviceSize gBufferSize = sizeof(UniformBufferObject);
    graphicsUniformBuffers.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (size_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; ++i) {

        graphicsUniformBuffers[i] = std::make_unique<vkb::Buffer>(device, gBufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                                  VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        graphicsUniformBuffers[i]->map();
    }
}

void PBFGPU3DSim::mainLoop(float deltaTime) {
    auto currentTime = std::chrono::high_resolution_clock::now();

    cameraController.moveCamera(window.window(), deltaTime, camera);
    updateUniformBuffers(renderer.currentFrame(), deltaTime);

    if (activateTimer) {
        auto time = std::chrono::high_resolution_clock::now();
        cpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(time - currentTime).count();
        currentTime = time;
    }

    if (!pausedSimulation) {
        updateSimulation();
        if (activateTimer) {
            auto time = std::chrono::high_resolution_clock::now();
            computeTime = std::chrono::duration<float, std::chrono::milliseconds::period>(time - currentTime).count();
            currentTime = time;
        }

        renderer.runFrame([this](VkCommandBuffer commandBuffer){
            renderObjects(commandBuffer);
        }, computeHandler.currentSemaphore(renderer.currentFrame()), vkb::ComputeShaderHandler::waitStages());
    } else {
        if (controlMode) initializeObjects(false);
        renderer.runFrame([this](VkCommandBuffer commandBuffer){
            renderObjects(commandBuffer);
        });
    }

    if (activateTimer) drawTime = std::chrono::duration<float, std::chrono::milliseconds::period>(std::chrono::high_resolution_clock::now() - currentTime).count();
}

void PBFGPU3DSim::renderObjects(VkCommandBuffer commandBuffer) {
    showImGui();
    VkBuffer vb = positionBuffers[computeFrameIdx]->getBuffer();
    VkDeviceSize offsets[] = {0};

    depthPass.run(commandBuffer, &simulationDescriptorSets[renderer.currentFrame()],
                  [this, &vb, &offsets](VkCommandBuffer &commandBuffer) {
        vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
        vkCmdDraw(commandBuffer, INSTANCE_COUNT, 1, 0, 0);
    });

    smoothPass.run(commandBuffer, &simulationDescriptorSets[renderer.currentFrame()],
                   [](VkCommandBuffer commandBuffer) {
        vkCmdDraw(commandBuffer, 3, 1, 0, 0);

    });

    for (uint32_t _ = 0; _ < blurIterations; _++){
        smoothPass.runAlternating(commandBuffer, {&smooth1DescriptorSets[renderer.currentFrame()], &smooth2DescriptorSets[renderer.currentFrame()]},
                                  [](VkCommandBuffer commandBuffer) {
            vkCmdDraw(commandBuffer, 3, 1, 0, 0);

        });
    }

    thicknessPass.run(commandBuffer,
                      (blurIterations % 2 == 0) ? &smooth1DescriptorSets[renderer.currentFrame()] : &smooth2DescriptorSets[renderer.currentFrame()],
                      [this, &vb, &offsets](VkCommandBuffer &commandBuffer) {
        vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
        vkCmdDraw(commandBuffer, INSTANCE_COUNT, 1, 0, 0);
    });

    normalsPass.run(commandBuffer,
                    (blurIterations % 2 == 0) ? &smooth1DescriptorSets[renderer.currentFrame()] : &smooth2DescriptorSets[renderer.currentFrame()],
                    [](VkCommandBuffer &commandBuffer) {
        vkCmdDraw(commandBuffer, 3, 1, 0, 0);
    });

    renderer.runRenderPass([this, &vb, &offsets](VkCommandBuffer& commandBuffer){
        skyboxSystem.bind(commandBuffer, &skyboxDescriptorSets[renderer.currentFrame()]);
        skybox.bindAndDraw(commandBuffer);

        if (debugScene) {
            debugRenderSystem.bind(commandBuffer, &debugDescriptorSets[renderer.currentFrame()]);
            vkCmdDraw(commandBuffer, 3, 1, 0, 0);

        } else {
            particleSystem.bind(commandBuffer, &simulationDescriptorSets[renderer.currentFrame()]);
            vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
            VkBuffer vbDens = densityBuffer->getBuffer();
            vkCmdBindVertexBuffers(commandBuffer, 1, 1, &vbDens, offsets);
            vkCmdBindIndexBuffer(commandBuffer, idxBuffer->getBuffer(), 0, VK_INDEX_TYPE_UINT32);
            vkCmdDrawIndexed(commandBuffer, INSTANCE_COUNT, 1, 0, 0, 0);
        }

        defaultSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);
        plane.render(defaultSystem, commandBuffer);

    });

}

void PBFGPU3DSim::updateSimulation() {

    computeHandler.runCompute(renderer.currentFrame(), [this](VkCommandBuffer computeCommandBuffer){
        uint32_t blockSize = INSTANCE_COUNT/256 + (1 - (INSTANCE_COUNT%256 == 0));

        predictPositionKernel.bindAndDispatch(computeCommandBuffer, computeFrameIdx, blockSize, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[(computeFrameIdx + 1) % 2]);

        gridHandler.createGrid(computeCommandBuffer, computeFrameIdx);

        vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);

        for (uint32_t i = 0; i < jacobiIterations; i++){

            lambdaKernel.bindAndDispatch(computeCommandBuffer, computeFrameIdx, blockSize, 1, 1);

            vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);

            posCorrectionKernel.bindAndDispatch(computeCommandBuffer, computeFrameIdx, blockSize, 1, 1);

            vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);
        }

        updateVelocitiesKernel.bindAndDispatch(computeCommandBuffer, computeFrameIdx, blockSize, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);

        applyViscAndComputeVortKernel.bindAndDispatch(computeCommandBuffer, computeFrameIdx, blockSize, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);

        applyVortAndUpdatePos.bindAndDispatch(computeCommandBuffer, computeFrameIdx, blockSize, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);

    });

    computeFrameIdx = (computeFrameIdx + 1) % positionBuffers.size();
}

void PBFGPU3DSim::updateUniformBuffers(uint32_t frameIndex, float deltaTime){
    camera.setPerspectiveProjection(glm::radians(50.f), renderer.getSwapChainAspectRatio(), 0.1f, 500.f);
    gUbo.view = camera.getView();
    gUbo.inverseView = glm::inverse(gUbo.view);
    gUbo.proj = camera.getProjection();
    gUbo.restDens = cUbo.REST_DENS;
    graphicsUniformBuffers[frameIndex]->write(&gUbo);

    cUbo.planeY = plane.m_translation.y;
//    cUbo.DT = std::clamp(deltaTime, 0.001f, 0.016f);
//    cUbo.G = 0.0f*glm::vec3(0.0f, -9.8f, 0.0f);
    computeUniformBuffer->write(&cUbo);
}

void PBFGPU3DSim::showImGui(){
    ImGui::Begin("Control Panel");

    ImGui::Text("Rendering %d instances", INSTANCE_COUNT);
    ImGui::Text("Grid size: %d", GRID_SIZE);
    ImGui::Checkbox("Display time", &activateTimer);
    if(activateTimer){
        ImGui::Text("Cpu time: %f ms", cpuTime);
        ImGui::Text("Compute time: %f ms", computeTime);
        ImGui::Text("Draw time: %f ms", drawTime);
    }

    if (ImGui::Button("Reset and enter control mode")) {
        pausedSimulation = true;
        controlMode = true;
    }

    if (ImGui::CollapsingHeader("Simulation constants", ImGuiTreeNodeFlags_DefaultOpen)) {
        int iJac = int(jacobiIterations);
        ImGui::DragInt("Num iterations", &iJac, 1, 1, 5);
        jacobiIterations = iJac;
        ImGui::DragFloat("Gravity Factor", &gravityFactor, 0.001f, 0.001f, 5.0f);
        ImGui::DragFloat("Viscosity", &cUbo.VISC, 0.001f, 0.001f, 5.0f);
        ImGui::DragFloat("DeltaTime", &cUbo.DT, 0.00005f, 0.0005f, 0.02f, "%.5f");
        ImGui::DragFloat("Rest Density", &cUbo.REST_DENS, 0.1f, 0.1f, 1000.0f, "%.4f");
        if (cUbo.REST_DENS == 0) cUbo.REST_DENS = 0.1f;
        ImGui::DragFloat("Mass", &cUbo.MASS, 0.001f, 0.001f, 40.0f, "%.1f");
        ImGui::DragFloat("CFM", &cUbo.CFM, 0.005f, 0.005f, 10.0f, "%.4f");
        if (cUbo.CFM == 0) cUbo.CFM = 0.1;
        ImGui::DragFloat("ARTIFICIAL PRESSURE", &cUbo.ART_PRESSURE_COEF, 0.0001f, 0.0001f, 10.0f, "%.4f");
        ImGui::DragFloat("VORTICITY COEF", &cUbo.VORTICITY_COEF, 0.000001f, 0.000001f, 0.1f, "%.6f");
    }

    if (ImGui::CollapsingHeader("Boundaries", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SliderFloat("Plane Y", &plane.m_translation.y, -100.0f, 100.0f);
    }

    if (ImGui::CollapsingHeader("Rendering", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::DragFloat("Particle Radius", &gUbo.radius, 0.1f, 0.1f, 100.0f);

        ImGui::Text("View mode");
        static std::array<std::string, 9> renderTypes = {"Particles", "Depth", "Thickness", "Normals", "Smooth",
                                                         "Reflection", "Refraction", "Fresnel Scale", "Fresnel"};
        std::string curItem = renderTypes[gUbo.renderType];
        if (ImGui::BeginCombo("##combo", curItem.c_str())) {
            for (uint32_t i = 0; i < renderTypes.size(); i++){
                bool isSelected = (curItem == renderTypes[i]);
                if (ImGui::Selectable(renderTypes[i].c_str(), isSelected)) {
                    gUbo.renderType = i;
                }
                if (isSelected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }
        debugScene = gUbo.renderType != 0;

        ImGui::SetCursorPosX(10.0f);
        if (ImGui::CollapsingHeader("Blur Options", ImGuiTreeNodeFlags_DefaultOpen)) {
            ImGui::SetCursorPosX(15.0f);
            ImGui::Text("Blur mode");
            static std::array<std::string, 3> blurTypes = {"Bilateral", "Gaussian", "Bilateral 2"};
            curItem = blurTypes[gUbo.blurMode];
            ImGui::SetCursorPosX(15.0f);
            if (ImGui::BeginCombo("##combo2", curItem.c_str())) {
                for (uint32_t i = 0; i < blurTypes.size(); i++){
                    bool isSelected = (curItem == blurTypes[i]);
                    if (ImGui::Selectable(blurTypes[i].c_str(), isSelected)) {
                        gUbo.blurMode = i;
                    }
                    if (isSelected)
                        ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
            ImGui::SetCursorPosX(15.0f);
            ImGui::DragInt("Blur Iterations", &blurIterations, 1, 0, 20);
            ImGui::SetCursorPosX(15.0f);
            ImGui::DragInt("Smoothing Radius", &gUbo.filterRadius, 1, 0, 20);
            ImGui::SetCursorPosX(15.0f);
            ImGui::DragFloat("Blur Scale", &gUbo.blurScale, 0.01f, 0.01f, 5.0f, "%.3f");
            ImGui::SetCursorPosX(15.0f);
            ImGui::DragFloat("Blur Fall Off", &gUbo.blurDepthFalloff, 10.0f, 100.0f, 10000.0f);
        }
    }

    if (singleStep) {
        pausedSimulation = true;
        singleStep = false;
    }

    static bool pWasReleased = false;
    if (ImGui::Button("Step") || pWasReleased && glfwGetKey(window.window(), GLFW_KEY_P) == GLFW_PRESS) {
        singleStep = true;
        pausedSimulation = false;

    } else if (glfwGetKey(window.window(), GLFW_KEY_P) == GLFW_RELEASE){
        pWasReleased = true;
    }

    if (ImGui::Button("Reset")) {
        initializeObjects(true);
        controlMode = false;
        pausedSimulation = false;
    }

    static bool rWasReleased = false;
    if (rWasReleased && glfwGetKey(window.window(), GLFW_KEY_R) == GLFW_PRESS) {
        initializeObjects(true);
        controlMode = false;
    } else if (glfwGetKey(window.window(), GLFW_KEY_R) == GLFW_RELEASE){
        rWasReleased = true;
    }

    if (hardResetFrame > 2) {
        hardResetFrame = 2;
        enableEmergencyExit();
    }
    if (ImGui::Button("Hard Reset")) {
        hardResetFrame = 0;
        disableEmergencyExit();
        onCreate();
        controlMode = false;
        pausedSimulation = false;
    }
    hardResetFrame++;



    ImGui::Text("Using %s", device.getPhysicalDeviceName().c_str());
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                ImGui::GetIO().Framerate);
    ImGui::End();

    static bool wasReleased = false;
    if (wasReleased && glfwGetKey(window.window(), GLFW_KEY_SPACE) == GLFW_PRESS) {
        pausedSimulation = !pausedSimulation;
        wasReleased = false;
        if (controlMode) {
            initializeObjects(true);
            controlMode = false;
        }

//        std::cout << "camera.m_translation = {" << camera.m_translation.x << "f, " << camera.m_translation.y << "f, " << camera.m_translation.z << "f};\n";
//        std::cout << "camera.m_rotation = {" << camera.m_rotation.x << "f, " << camera.m_rotation.y << "f, " << camera.m_rotation.z << "f};\n";
    } else if (glfwGetKey(window.window(), GLFW_KEY_SPACE) == GLFW_RELEASE){
        wasReleased = true;
    }


    if (controlMode) {
        ImGui::Begin("Control Mode");

        auto getParticleShapeSize = [this]() -> glm::vec3 {
            return {float(numParticlesXZ.x) * particleSpacing + cUbo.H,
                    float(INSTANCE_COUNT) / float(numParticlesXZ.x * numParticlesXZ.y) * particleSpacing + cUbo.H,
                    float(numParticlesXZ.y) * particleSpacing + cUbo.H
            };
        };

        int temp = (int) INSTANCE_COUNT;
        ImGui::SliderInt("Num Particles", &temp, 16, MAX_PARTICLES);

        ImGui::SliderFloat("Spacing", &particleSpacing, cUbo.H, 2*cUbo.H);
        ImGui::SliderFloat("Spacing vert", &particleVerticalSpacing, cUbo.H, 2 * cUbo.H);

        auto particleShapeSize = getParticleShapeSize();

        if (temp != INSTANCE_COUNT) {
            INSTANCE_COUNT = (uint32_t) temp;

            numParticlesXZ = glm::ivec2(int(std::cbrt(float(temp))));
            particleShapeSize = getParticleShapeSize();
            for (int i = 0; i < 3; i++)
                if (cUbo.BOUNDARY_SIZE[i] < particleShapeSize[i] + 2*cUbo.EPS)
                    cUbo.BOUNDARY_SIZE[i] = particleShapeSize[i] + 2*cUbo.EPS;
        }


        glm::vec3 newBoundSize = cUbo.BOUNDARY_SIZE;
        auto maxBound = glm::vec3(MAX_BOUND);

        ImGui::CDragFloatRanged3("Boundary Size", &newBoundSize[0], 1, &particleShapeSize[0], &maxBound[0]);
        for (int i = 0; i < 3; i++){
            if (newBoundSize[i] != cUbo.BOUNDARY_SIZE[i] && newBoundSize[i] >= particleShapeSize[i] + 2*cUbo.EPS) {
                cUbo.BOUNDARY_SIZE[i] = newBoundSize[i];
            }
        }

        auto minPos = glm::vec3(cUbo.EPS);
        auto maxPos = glm::vec3(cUbo.BOUNDARY_SIZE.x - particleShapeSize.x, std::max(cUbo.BOUNDARY_SIZE.y - particleShapeSize.y - 3*cUbo.EPS, cUbo.EPS), cUbo.BOUNDARY_SIZE.z - particleShapeSize.z);
        ImGui::CSliderFloatRanged3("Initial Pos", &initialPos[0], &minPos[0], &maxPos[0]);

        auto numParticleMin = glm::ivec2(2);
        auto numParticleMax = glm::ivec2(int(cUbo.BOUNDARY_SIZE.x/cUbo.H - cUbo.H), int(cUbo.BOUNDARY_SIZE.z/cUbo.H - cUbo.H));
        ImGui::CSliderIntRanged2("Num Particles XZ", &numParticlesXZ[0], &numParticleMin[0], &numParticleMax[0]);

        particleShapeSize = getParticleShapeSize();

        for (int i = 0; i < 3; i++){
            float newEps = (i == 1) ? 3*cUbo.EPS : cUbo.EPS;
            if (initialPos[i] > cUbo.BOUNDARY_SIZE[i] - particleShapeSize[i] - newEps) {
                if (cUbo.BOUNDARY_SIZE[i] - initialPos[i] - particleShapeSize[i] < newEps)
                    cUbo.BOUNDARY_SIZE[i] = initialPos[i] + particleShapeSize[i] + newEps;
                else
                    initialPos[i] = cUbo.BOUNDARY_SIZE[i] - particleShapeSize[i]  + newEps;
            }
        }

        if (ImGui::Button("Launch and close")){
            initializeObjects(true);
            controlMode = false;
            pausedSimulation = false;
        }
        ImGui::End();
    }

}

void PBFGPU3DSim::onResize(int width, int height) {
    gUbo.screenHeight = (float) height;
    gUbo.screenWidth = (float) width;
    depthPass.changeImageSize(renderer.getSwapChainExtent());
    thicknessPass.changeImageSize(renderer.getSwapChainExtent());
    normalsPass.changeImageSize(renderer.getSwapChainExtent());
    smoothPass.changeImageSize(renderer.getSwapChainExtent());
    auto debugDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
            .addSameTypeBindings(1, 6, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT)
            .build();
    debugDescriptorSets = createDescriptorSets(
            debugDescriptorLayout,
            {graphicsUniformBuffers[0]->descriptorInfo()},
            {depthPass.descriptorInfo(), thicknessPass.descriptorInfo(), normalsPass.descriptorInfo(), smoothPass.descriptorInfo(),
             plane.textureInfo(), skybox.descriptorInfo()}
    );

    auto defaultDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
            .addBinding({1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr})
            .build();
    simulationDescriptorSets = createDescriptorSets(defaultDescriptorLayout,
                                                {graphicsUniformBuffers[0]->descriptorInfo()}, {depthPass.descriptorInfo()});
    smooth1DescriptorSets = createDescriptorSets(defaultDescriptorLayout,
                                                 {graphicsUniformBuffers[0]->descriptorInfo()}, {smoothPass.descriptorInfo()});
    smooth2DescriptorSets = createDescriptorSets(defaultDescriptorLayout,
                                                 {graphicsUniformBuffers[0]->descriptorInfo()}, {smoothPass.additionalImageDescriptorInfo()});
}

void PBFGPU3DSim::compileShaders() {
    for (uint32_t i = 0; i < shaders.size(); i++) {
        std::string command{"glslc "};
        if (i < gridShaderStartIdx) {
            command += RENDER_SHADER_DIR;
        } else if (i < computeShaderStartIdx) {
            command += GRID_SHADER_DIR;
        } else {
            command += SIMULATIONS_SHADER_DIR;
        }
        command += shaders[i];
        command += " --target-env=vulkan1.1 ";
        command += " -o ";
        command += COMPILED_SHADER_DIR;
        command += shaders[i];
        command += ".spv";
        int status = system(command.c_str());
        if (status != 0) {
            throw std::runtime_error("Error compiling shader " + shaders[i]);
        }
    }
}


