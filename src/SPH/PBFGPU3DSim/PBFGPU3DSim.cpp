//
// Created by luc on 01/06/23.
//

#include "PBFGPU3DSim.h"

void PBFGPU3DSim::onCreate() {
    vkDeviceWaitIdle(device.device());
    presets.clear();
    for (const auto & entry : std::filesystem::directory_iterator(PRESET_DIR)){
        presets.emplace_back(entry.path());
    }

//    rigidObjects.clear();
//    addRigidObject(0);
//    rigidObjects[0].translate({10.0f,0.0f, 6.0f});

    compileShaders();
//    camera.m_translation = {-9.31845f, 14.2878f, 15.5649f};
//    camera.m_rotation = {0.569391f, 2.26887f, 3.14159f};
    camera.m_translation = {-13.6108f, 6.82289f, 7.21078f};
    camera.m_rotation = {0.284774f, 1.74578f, 3.14159f};
    camera.updateView();
    gUbo.radius = cUbo.H/2;
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
    rockDescriptorSets = createDescriptorSets(defaultDescriptorLayout,
                                                {graphicsUniformBuffers[0]->descriptorInfo()}, {rockTex->descriptorInfo()});
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
            info.bindingDescription.push_back({2, sizeof(uint32_t), VK_VERTEX_INPUT_RATE_VERTEX});
            info.attributeDescription.clear();
            info.attributeDescription.push_back({0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0});
            info.attributeDescription.push_back({1, 1, VK_FORMAT_R32_SFLOAT, 0});
            info.attributeDescription.push_back({2, 2, VK_FORMAT_R32_UINT, 0});

        });
    }

    // create offscreen passes
    {
        depthPass.createPass(defaultDescriptorLayout.descriptorSetLayout(), depthShaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& info) {
            info.inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
            info.bindingDescription.clear();
            info.bindingDescription.push_back({0, sizeof(glm::vec4), VK_VERTEX_INPUT_RATE_VERTEX});
            info.bindingDescription.push_back({1, sizeof(uint32_t), VK_VERTEX_INPUT_RATE_VERTEX});
            info.attributeDescription.clear();
            info.attributeDescription.push_back({0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0});
            info.attributeDescription.push_back({1, 1, VK_FORMAT_R32_UINT, 0});
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
            info.bindingDescription.push_back({1, sizeof(uint32_t), VK_VERTEX_INPUT_RATE_VERTEX});
            info.attributeDescription.clear();
            info.attributeDescription.push_back({0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0});
            info.attributeDescription.push_back({1, 1, VK_FORMAT_R32_UINT, 0});
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

        scenePass.createPass(defaultDescriptorLayout.descriptorSetLayout(), shaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& info){
                info.multisampling.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
        }, sizeof(vkb::DrawableObject::PushConstantData));

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

    //create skybox
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

        skyboxTexSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), 0);
        skyboxTexSystem.createPipeline(scenePass.renderPass(), skyboxShaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& info) {
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

        shadingRenderSystem.createPipelineLayout(debugDescriptorLayout.descriptorSetLayout(), 0);
        shadingRenderSystem.createPipeline(renderer.renderPass(), quadShaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& info) {
            info.bindingDescription.clear();
            info.attributeDescription.clear();
            info.rasterizer.cullMode = VK_CULL_MODE_NONE;
        });

        shadingDescriptorSets[0] = createDescriptorSets(
                debugDescriptorLayout,
                {graphicsUniformBuffers[0]->descriptorInfo()},
                {depthPass.descriptorInfo(), thicknessPass.descriptorInfo(), scenePass.descriptorInfo(), depthPass.descriptorInfo(),
                 plane.textureInfo(), skybox.descriptorInfo()}
        );
        shadingDescriptorSets[1] = createDescriptorSets(
                debugDescriptorLayout,
                {graphicsUniformBuffers[0]->descriptorInfo()},
                {depthPass.descriptorInfo(), thicknessPass.descriptorInfo(), scenePass.descriptorInfo(), smoothPass.descriptorInfo(),
                 plane.textureInfo(), skybox.descriptorInfo()}
        );
        shadingDescriptorSets[2] = createDescriptorSets(
                debugDescriptorLayout,
                {graphicsUniformBuffers[0]->descriptorInfo()},
                {depthPass.descriptorInfo(), thicknessPass.descriptorInfo(), scenePass.descriptorInfo(), smoothPass.additionalImageDescriptorInfo(),
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
                {particleTypeBuffers[i]->descriptorInfo()},
        });

        for (uint32_t  j = 0; j < gaussPartition; j++) {
            uint32_t idx = gaussPartition*i + j;
            lambdaKernel.descSets[idx] = vkb::DescriptorWriter::createSingleDescriptorSet(globalDescriptorPool, lambdaKernel.layout, {
                    {computeUniformBuffer->descriptorInfo()},
                    {solverUniformBuffers[j]->descriptorInfo()},
                    {gridHandler.gridDescriptorInfo()},
                    {predPosBuffers[(i + 1) % predPosBuffers.size()]->descriptorInfo()},
                    {lambdaBuffer->descriptorInfo()},
                    {densityBuffer->descriptorInfo()},
                    {gridIdxBuffer->descriptorInfo()},
                    {particleTypeBuffers[(i + 1) % particleTypeBuffers.size()]->descriptorInfo()},
                    {avgDensBuffer->descriptorInfo()},
            });

            posCorrectionKernel.descSets[idx] = vkb::DescriptorWriter::createSingleDescriptorSet(globalDescriptorPool, posCorrectionKernel.layout, {
                    {computeUniformBuffer->descriptorInfo()},
                    {solverUniformBuffers[j]->descriptorInfo()},
                    {gridHandler.gridDescriptorInfo()},
                    {predPosBuffers[(i + 1) % predPosBuffers.size()]->descriptorInfo()},
                    {lambdaBuffer->descriptorInfo()},
                    {gridIdxBuffer->descriptorInfo()},
                    {particleTypeBuffers[(i + 1) % particleTypeBuffers.size()]->descriptorInfo()},
                    {deltaPBuffers[(i + 1) % deltaPBuffers.size()]->descriptorInfo()},
            });
        }


        updateVelocitiesKernel.descSets[i] = vkb::DescriptorWriter::createSingleDescriptorSet(globalDescriptorPool, updateVelocitiesKernel.layout, {
                {computeUniformBuffer->descriptorInfo()},
                {positionBuffers[(i + 1) % positionBuffers.size()]->descriptorInfo()},
                {velocityBuffers[(i + 1) % velocityBuffers.size()]->descriptorInfo()},
                {predPosBuffers[(i + 1) % predPosBuffers.size()]->descriptorInfo()},
                {particleTypeBuffers[(i + 1) % particleTypeBuffers.size()]->descriptorInfo()},
        });

        applyViscAndComputeVortKernel.descSets[i] = vkb::DescriptorWriter::createSingleDescriptorSet(globalDescriptorPool, applyViscAndComputeVortKernel.layout, {
                {computeUniformBuffer->descriptorInfo()},
                {gridHandler.gridDescriptorInfo()},
                {predPosBuffers[(i + 1) % predPosBuffers.size()]->descriptorInfo()},
                {velocityBuffers[(i + 1) % velocityBuffers.size()]->descriptorInfo()},
                {vorticityBuffer->descriptorInfo()},
                {gridIdxBuffer->descriptorInfo()},
                {particleTypeBuffers[(i + 1) % particleTypeBuffers.size()]->descriptorInfo()},
        });

        applyVortAndUpdatePos.descSets[i] = vkb::DescriptorWriter::createSingleDescriptorSet(globalDescriptorPool, applyVortAndUpdatePos.layout, {
                {computeUniformBuffer->descriptorInfo()},
                {gridHandler.gridDescriptorInfo()},
                {predPosBuffers[(i + 1) % predPosBuffers.size()]->descriptorInfo()},
                {positionBuffers[(i + 1) % positionBuffers.size()]->descriptorInfo()},
                {velocityBuffers[(i + 1) % velocityBuffers.size()]->descriptorInfo()},
                {vorticityBuffer->descriptorInfo()},
                {gridIdxBuffer->descriptorInfo()},
                {particleTypeBuffers[(i + 1) % particleTypeBuffers.size()]->descriptorInfo()},
        });
    }
}

void PBFGPU3DSim::initializeObjects(bool activateRandomOffsets) {
    vkDeviceWaitIdle(device.device());
    computeFrameIdx = 0;
    gUbo.view = camera.getView();
    gUbo.inverseView = glm::inverse(gUbo.view);
    gUbo.proj = camera.getProjection();
    activateWaves = false;

    // initialize particles
    static bool testing = true;
    if (!testing) {
        if (!objectsInitialized) PBFSceneManager::loadScene(*this, PRESET_DIR + curFile.data());
        particles.resize(NUM_PARTICLES);
        plane.setScale(cUbo.BOUNDARY_SIZE);
        NUM_FLUID_PARTICLES = NUM_PARTICLES - NUM_RIGID_PARTICLES;
        cUbo.numParticles = NUM_FLUID_PARTICLES;

        cUbo.DT = 1/(60.0f*float(substeps));
        cUbo.numParticles = initialParticles;
        GRID_SIZE = uint32_t(cUbo.BOUNDARY_SIZE.x/cUbo.H)*uint32_t(cUbo.BOUNDARY_SIZE.y/cUbo.H)*uint32_t(cUbo.BOUNDARY_SIZE.z/cUbo.H) + 1;
        gUbo.planeSize = cUbo.BOUNDARY_SIZE;
    }
    else {
        particles.resize(NUM_PARTICLES);
        plane.setScale(cUbo.BOUNDARY_SIZE);

        // run once to create the files
        NUM_FLUID_PARTICLES = NUM_PARTICLES - NUM_RIGID_PARTICLES;
        cUbo.numParticles = NUM_FLUID_PARTICLES;
        PbfInitializer pbfInitializer{particles};

        cUbo.DT = 1/(60.0f*float(substeps));
        cUbo.numParticles = initialParticles;
        GRID_SIZE = uint32_t(cUbo.BOUNDARY_SIZE.x/cUbo.H)*uint32_t(cUbo.BOUNDARY_SIZE.y/cUbo.H)*uint32_t(cUbo.BOUNDARY_SIZE.z/cUbo.H) + 1;
        gUbo.planeSize = cUbo.BOUNDARY_SIZE;

        NUM_RIGID_PARTICLES = 0;
        for (const auto& rigidObj : rigidObjects) {
            NUM_RIGID_PARTICLES += rigidObj.numParticles();
        }
        NUM_FLUID_PARTICLES = NUM_PARTICLES - NUM_RIGID_PARTICLES;
        cUbo.numParticles = NUM_FLUID_PARTICLES;
    //    sourceParticleSum = pbfInitializer.waterFallInitializer(cUbo, activateRandomOffsets);
        sourceParticleSum = pbfInitializer.cityInitializer(cUbo, true);
        initialParticles = cUbo.numParticles;

        cUbo.numParticles += NUM_RIGID_PARTICLES;



        for (int i = int(NUM_PARTICLES) - 1; i >= int(NUM_RIGID_PARTICLES); i--) {
            particles.position[i] = particles.position[i - NUM_RIGID_PARTICLES];
            particles.density[i] = particles.density[i - NUM_RIGID_PARTICLES];
            particles.type[i] = particles.type[i - NUM_RIGID_PARTICLES];
            particles.velocity[i] = particles.velocity[i - NUM_RIGID_PARTICLES];
        }

        if (!rigidObjects.empty()) {
            auto rigidObjParticles = rigidObjects[0].getPositions();
            uint32_t rigidObjIdx = 0;
            uint32_t initialIdx = 0;
            for (uint32_t i = 0; i < NUM_RIGID_PARTICLES; i++) {
                particles.type[i] = 1;

                if (i - initialIdx >= rigidObjParticles.size()) {
                    rigidObjIdx++;
                    initialIdx = i;
                    rigidObjParticles = rigidObjects[rigidObjIdx].getPositions();
                }
                particles.position[i] = glm::vec4(rigidObjParticles[i - initialIdx], 0);
            }
        }

    }

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
            predPosBuffers[i] = makeBuffer(particles.position.size() * sizeof(glm::vec4));
            deltaPBuffers[i] = makeBuffer(particles.position.size() * sizeof(glm::vec4));
            particleTypeBuffers[i] = std::make_unique<vkb::Buffer>(device, particles.type.size() * sizeof(uint32_t),
                                                                   VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                                                   VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

            if (i == 0) {
                vkb::Buffer::writeVectorToBuffer(device, positionBuffers[i], particles.position);
                vkb::Buffer::writeVectorToBuffer(device, predPosBuffers[i], particles.position);
                vkb::Buffer::writeVectorToBuffer(device, velocityBuffers[i], particles.velocity);
                vkb::Buffer::writeVectorToBuffer(device, particleTypeBuffers[i], particles.type);
            }

        }
        vorticityBuffer = makeBuffer(particles.position.size()*sizeof(glm::vec4));
        densityBuffer = std::make_unique<vkb::Buffer>(device, particles.density.size() * sizeof(float),
                                                      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                                      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        avgDensBuffer = std::make_unique<vkb::Buffer>(device, sizeof(float),
                                                      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                                      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        lambdaBuffer = makeBuffer(particles.position.size()*sizeof(float));
        gridIdxBuffer = makeBuffer(particles.gIdx.size() * sizeof(uint32_t));
    }

    // create barriers
    for (uint32_t i = 0; i < particleBarrierData.size(); i++){
        particleBarrierData[i] = {
                positionBuffers[(i+1) % positionBuffers.size()]->getBarrierData(),
                velocityBuffers[(i+1) % velocityBuffers.size()]->getBarrierData(),
                predPosBuffers[(i+1) % predPosBuffers.size()]->getBarrierData(),
                deltaPBuffers[(i+1) % deltaPBuffers.size()]->getBarrierData(),
                particleTypeBuffers[(i+1) % particleTypeBuffers.size()]->getBarrierData(),
                vorticityBuffer->getBarrierData(),
                densityBuffer->getBarrierData(),
                lambdaBuffer->getBarrierData(),
        };
    }


    // create grid
    if (activateRandomOffsets){
        gridHandler.createDescriptorsAndBuffers(
                globalDescriptorPool,
                GRID_SIZE,
                {cUbo.numParticles, cUbo.H, cUbo.BOUNDARY_SIZE},
                gridIdxBuffer,
                positionBuffers,
                velocityBuffers,
                predPosBuffers,
                particleTypeBuffers,
                deltaPBuffers);
    }

    //free descriptor sets
    if (objectsInitialized) {
        std::vector<VkDescriptorSet> descriptorsToFree = {
                predictPositionKernel.descSets[0],
                predictPositionKernel.descSets[1],
                updateVelocitiesKernel.descSets[0],
                updateVelocitiesKernel.descSets[1],
                applyViscAndComputeVortKernel.descSets[0],
                applyViscAndComputeVortKernel.descSets[1],
                applyVortAndUpdatePos.descSets[0],
                applyVortAndUpdatePos.descSets[1],
        };
        for (uint32_t i = 0; i < lambdaKernel.descSets.size(); i++) {
            descriptorsToFree.push_back(lambdaKernel.descSets[i]);
            descriptorsToFree.push_back(posCorrectionKernel.descSets[i]);
        }


        globalDescriptorPool->freeDescriptors(descriptorsToFree);
    }
    createComputeDescriptorSets();

    // create idx buffer for rendering
    std::vector<uint32_t> idxs(NUM_PARTICLES);
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

    std::vector<SolverUniformBufferObject> sUbo = {{gaussPartition, 0}};
    solverUniformBuffers.clear();
    for (uint32_t i = 0; i < gaussPartition; i++) {
        solverUniformBuffers.emplace_back(std::make_unique<vkb::Buffer>(device, sizeof(SolverUniformBufferObject), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                              VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT));
        vkb::Buffer::writeVectorToBuffer(device, solverUniformBuffers[i], sUbo);
        sUbo[0].step++;
    }


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

    keyboardControl(deltaTime);
    updateUniformBuffers(renderer.currentFrame(), deltaTime);

    if (activateTimer) {
        auto time = std::chrono::high_resolution_clock::now();
        cpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(time - currentTime).count();
        currentTime = time;
    }

    if (!pausedSimulation) {
        updateSimulation(deltaTime);
        if (activateTimer) {
            auto time = std::chrono::high_resolution_clock::now();
            computeTime = std::chrono::duration<float, std::chrono::milliseconds::period>(time - currentTime).count();
            currentTime = time;
        }

        if (!test)
            renderer.runFrame([this](VkCommandBuffer commandBuffer){
                showImGui();
                renderObjects(commandBuffer);
            }, computeHandler.currentSemaphore(renderer.currentFrame()), vkb::ComputeShaderHandler::waitStages());
    } else {
        if (!test)
            renderer.runFrame([this](VkCommandBuffer commandBuffer){
                showImGui();
                if (controlMode) initializeObjects(false);
                renderObjects(commandBuffer);
            });
    }

    if (activateTimer) drawTime = std::chrono::duration<float, std::chrono::milliseconds::period>(std::chrono::high_resolution_clock::now() - currentTime).count();
}

void PBFGPU3DSim::renderObjects(VkCommandBuffer commandBuffer) {
    VkBuffer vb = positionBuffers[computeFrameIdx]->getBuffer();
    VkBuffer vbType = particleTypeBuffers[computeFrameIdx]->getBuffer();
    VkDeviceSize offsets[] = {0};

    if (!showParticles) {
        depthPass.run(commandBuffer, &simulationDescriptorSets[renderer.currentFrame()],
                      [this, &vb, &vbType, &offsets](VkCommandBuffer &commandBuffer) {
                          vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
                          vkCmdBindVertexBuffers(commandBuffer, 1, 1, &vbType, offsets);
                          vkCmdDraw(commandBuffer, cUbo.numParticles, 1, 0, 0);
                      });

        if (blurIterations > 0) {
            smoothPass.run(commandBuffer, &simulationDescriptorSets[renderer.currentFrame()],
                           [](VkCommandBuffer commandBuffer) {
                               vkCmdDraw(commandBuffer, 3, 1, 0, 0);
                           });
        }

        for (uint32_t _ = 1; _ < blurIterations; _++){
            smoothPass.runAlternating(commandBuffer, {&smooth1DescriptorSets[renderer.currentFrame()], &smooth2DescriptorSets[renderer.currentFrame()]},
                                      [](VkCommandBuffer commandBuffer) {
                                          vkCmdDraw(commandBuffer, 3, 1, 0, 0);

                                      });
        }

        thicknessPass.run(commandBuffer,
                          (blurIterations % 2 == 0) ? &smooth1DescriptorSets[renderer.currentFrame()] : &smooth2DescriptorSets[renderer.currentFrame()],
                          [this, &vb, &vbType, &offsets](VkCommandBuffer &commandBuffer) {
                              vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
                              vkCmdBindVertexBuffers(commandBuffer, 1, 1, &vbType, offsets);
                              vkCmdDraw(commandBuffer, cUbo.numParticles, 1, 0, 0);
                          });

        scenePass.run(commandBuffer,
                      &defaultDescriptorSets[renderer.currentFrame()],
                      [this](VkCommandBuffer &commandBuffer) {
                          if (renderSkybox) {
                              skyboxTexSystem.bind(commandBuffer, &skyboxDescriptorSets[renderer.currentFrame()]);
                              skybox.bindAndDraw(commandBuffer);
                          }
                          scenePass.bindRenderSystem(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);
                          plane.render(defaultSystem, commandBuffer);
                          scenePass.bindRenderSystem(commandBuffer, &rockDescriptorSets[renderer.currentFrame()]);
                          for (const auto& rigidObject : rigidObjects) {
                              rigidObject.render(defaultSystem, commandBuffer);
                          }
                      });
    }


    renderer.runRenderPass([this, &vb, &vbType, &offsets](VkCommandBuffer& commandBuffer){
        if (renderSkybox) {
            skyboxSystem.bind(commandBuffer, &skyboxDescriptorSets[renderer.currentFrame()]);
            skybox.bindAndDraw(commandBuffer);
        }

        if (showParticles) {
            particleSystem.bind(commandBuffer, &simulationDescriptorSets[renderer.currentFrame()]);
            vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
            VkBuffer vbDens = densityBuffer->getBuffer();
            vkCmdBindVertexBuffers(commandBuffer, 1, 1, &vbDens, offsets);

            vkCmdBindVertexBuffers(commandBuffer, 2, 1, &vbType, offsets);
            vkCmdBindIndexBuffer(commandBuffer, idxBuffer->getBuffer(), 0, VK_INDEX_TYPE_UINT32);
            vkCmdDrawIndexed(commandBuffer, cUbo.numParticles, 1, 0, 0, 0);
        } else {
            VkDescriptorSet* finalSceneDescriptorSet;
            if (blurIterations == 0) finalSceneDescriptorSet = &shadingDescriptorSets[0][renderer.currentFrame()];
            else finalSceneDescriptorSet = &shadingDescriptorSets[(blurIterations % 2 == 0) + 1][renderer.currentFrame()];
            shadingRenderSystem.bind(commandBuffer, finalSceneDescriptorSet);
            vkCmdDraw(commandBuffer, 3, 1, 0, 0);
        }

        defaultSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);
        plane.render(defaultSystem, commandBuffer);
        defaultSystem.bind(commandBuffer, &rockDescriptorSets[renderer.currentFrame()]);
        for (const auto& rigidObject : rigidObjects) {
            rigidObject.render(defaultSystem, commandBuffer);
        }

    });

}

void PBFGPU3DSim::updateSimulation(float deltaTime) {

    auto simulationStep = [this](VkCommandBuffer computeCommandBuffer){
        uint32_t blockSize = cUbo.numParticles / 256 + (1 - (cUbo.numParticles % 256 == 0));
        uint32_t solverBlockSize = cUbo.numParticles/(gaussPartition*256) + (1 - (cUbo.numParticles % (gaussPartition*256) == 0));

        for (uint32_t _ = 0; _ < substeps; _++) {
            predictPositionKernel.bindAndDispatch(computeCommandBuffer, computeFrameIdx, blockSize, 1, 1);

            vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[(computeFrameIdx + 1) % 2]);

            gridHandler.createGrid(computeCommandBuffer, computeFrameIdx);

            vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);

            for (uint32_t i = 0; i < jacobiIterations; i++){

                for (uint32_t j = 0; j < gaussPartition; j++) {
                    uint32_t descSetIdx = computeFrameIdx*gaussPartition + j;
                    lambdaKernel.bindAndDispatch(computeCommandBuffer, descSetIdx, solverBlockSize, 1, 1);

//                    vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);

                    posCorrectionKernel.bindAndDispatch(computeCommandBuffer, descSetIdx, solverBlockSize, 1, 1);

                }
            }
//            vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);

            updateVelocitiesKernel.bindAndDispatch(computeCommandBuffer, computeFrameIdx, blockSize, 1, 1);

            vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);

            if (activateVisc || activateVorticity) {
                applyViscAndComputeVortKernel.bindAndDispatch(computeCommandBuffer, computeFrameIdx, blockSize, 1, 1);

                vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);
            }
            if (activateVorticity) {
                applyVortAndUpdatePos.bindAndDispatch(computeCommandBuffer, computeFrameIdx, blockSize, 1, 1);

                vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);
            }

            computeFrameIdx = (computeFrameIdx + 1) % positionBuffers.size();
        }
    };

    if (!test) {
        static uint32_t frame = 0;
        static float totalTime = 0;
        computeHandler.runCompute(renderer.currentFrame(), simulationStep);
        if (frame++ >= 2) {
            totalTime += deltaTime;
        }

        if (frame >= 800) {
            std::cout << std::fixed << std::setprecision(6) << 1000*totalTime/float(frame - 2) << "ms\n";
            std::cout << std::setprecision(2) << 1000*totalTime/float(frame - 2) << "\n";
            endProgram();
        }

    } else {
        computeHandler.runComputeIsolated(renderer.currentFrame(), simulationStep);
        std::vector<float> avgDens(1);
        static uint32_t frame = 0;
        vkb::Buffer::writeBufferToVector(device, avgDensBuffer, avgDens);
    //    std::cout << "Frame: " <<  frame++ << " Average density: " << avgDens[0]/float(substeps*jacobiIterations*NUM_FLUID_PARTICLES) << std::endl;
        if (frame++ > 0) {
            auto val = std::to_string(avgDens[0]/(float(substeps*jacobiIterations*NUM_FLUID_PARTICLES)*cUbo.REST_DENS));
    //        val.replace(val.find('.'), 1, ",");
            std::cout << frame - 1 << "\t" << val << std::endl;
        }
        static int sla = 0;
        static float avgTime = 0;
        static uint32_t count = 0;
        if (frame >= 2) {
            avgTime += deltaTime;
            count++;
        }
        uint32_t numFrames = 800;
        if (frame >= numFrames) {
            std::cout << "end\t" << avgTime/float(count) << "\n";
            if (sla == 1) endProgram();
            sla++;
            avgTime = 0;
            count = 0;
            gaussPartition = 1;
            hardResetFrame = 0;
            disableEmergencyExit();
            onCreate();
            controlMode = false;
            pausedSimulation = false;
            frame = 0;
        }
        avgDens[0] = 0;
        vkb::Buffer::writeVectorToBuffer(device, avgDensBuffer, avgDens);
    }


}

void PBFGPU3DSim::updateUniformBuffers(uint32_t frameIndex, float deltaTime){
    camera.setPerspectiveProjection(glm::radians(50.f), renderer.getSwapChainAspectRatio(), 0.1f, 500.f);
    gUbo.view = camera.getView();
    gUbo.inverseView = glm::inverse(gUbo.view);
    gUbo.proj = camera.getProjection();
    gUbo.restDens = cUbo.REST_DENS;
    graphicsUniformBuffers[frameIndex]->write(&gUbo);

    static float accTime = 0;

    if (!pausedSimulation) {
        if (activateWaves) {
            cUbo.wallX += curSpeed * cUbo.DT;
            if (cUbo.wallX >= wallLimit) {
                curSpeed = -wallBackwardSpeed;
            } else if (cUbo.wallX <= 0) {
                cUbo.wallX = 0;
                curSpeed = wallForwardSpeed;
            }
        } else cUbo.wallX = 0;

//        if (delay)
//        accTime += deltaTime;
//        std::cout << "accTime: " << accTime << std::endl;
        auto addParticles = uint32_t((float(sourceParticleSum)));
        cUbo.numParticles = std::min(NUM_PARTICLES, cUbo.numParticles + addParticles);
        accTime = 0;
//        if (accTime > 1/80.0f) {
//        }
//        delay = !delay;

//        cUbo.DT = std::clamp(deltaTime, 0.001f, 0.02f);
//        cUbo.G = 0.0f*glm::vec3(0.0f, -9.8f, 0.0f);

        computeHandler.waitForCompute();
        computeUniformBuffer->write(&cUbo);
        gridHandler.updateUniforms(cUbo.numParticles, cUbo.H, cUbo.BOUNDARY_SIZE);

    }
}


void PBFGPU3DSim::keyboardControl(float deltaTime) {
    if (disableKeyboardControl) return;
    cameraController.moveCamera(window.window(), deltaTime, camera);

//    static bool pWasReleased = false;
//    if (!controlMode && pWasReleased && glfwGetKey(window.window(), GLFW_KEY_P) == GLFW_PRESS) {
//        singleStep = true;
//        pausedSimulation = false;
//
//    } else if (glfwGetKey(window.window(), GLFW_KEY_P) == GLFW_RELEASE){
//        pWasReleased = true;
//    }

//    static bool rWasReleased = false;
//    if (rWasReleased && glfwGetKey(window.window(), GLFW_KEY_R) == GLFW_PRESS) {
//        initializeObjects(true);
//        controlMode = false;
//    } else if (glfwGetKey(window.window(), GLFW_KEY_R) == GLFW_RELEASE){
//        rWasReleased = true;
//    }

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
}

void PBFGPU3DSim::showImGui(){
    ImGui::Begin("Control Panel");

    ImGui::Text("Rendering %d instances", NUM_PARTICLES);
    ImGui::Text("Grid size: %d", GRID_SIZE);
    ImGui::Checkbox("Display time", &activateTimer);
    if(activateTimer) {
        ImGui::Text("Cpu time: %f ms", cpuTime);
        ImGui::Text("Compute time: %f ms", computeTime);
        ImGui::Text("Draw time: %f ms", drawTime);
    }

    if (ImGui::Button("Load Configurations")) {
        pausedSimulation = true;
        isLoadWindowOpen = true;
        saveFileName = "Enter file name";
        disableKeyboardControl = true;

    }
    bool hardReset = false;

    if (ImGui::CollapsingHeader("Simulation constants")) {
        int sub = int(substeps);
        ImGui::DragInt("Num substeps", &sub, 0.1, 1, 8);
        substeps = sub;

        int gauss = int(gaussPartition);
        ImGui::DragInt("Num gaussPartition", &gauss, 0.1, 1, 8);
        if (gauss > gaussPartition) hardReset = true;
        gaussPartition = gauss;

        int iJac = int(jacobiIterations);
        ImGui::DragInt("Num iterations", &iJac, 0.1, 1, 8);
        jacobiIterations = iJac;
        ImGui::DragFloat("DeltaTime", &cUbo.DT, 0.00005f, 0.0005f, 0.02f, "%.5f");
        ImGui::DragFloat("Rest Density", &cUbo.REST_DENS, 1.0f, 2000.0f, 10000.0f, "%.0f");
        if (cUbo.REST_DENS == 0) cUbo.REST_DENS = 0.1f;
        ImGui::DragFloat("CFM", &cUbo.CFM, 1.0f, 300.0f, 5000.0f, "%.4f");
        if (cUbo.CFM == 0) cUbo.CFM = 0.1;
        ImGui::DragFloat("Artificial Pressure", &cUbo.ART_PRESSURE_COEF, 0.00001f, 0.00001f, 0.001f, "%.5f");

        ImGui::NewLine();
        ImGui::Checkbox("Activate viscosity", &activateVisc);
        ImGui::DragFloat("Viscosity", &cUbo.VISC, 0.00001f, 0.00001f, 0.001f, "%.5f");
        ImGui::Checkbox("Activate vorticity", &activateVorticity);
        cUbo.activateVort = activateVorticity;
        ImGui::DragFloat("Vorticity Coef", &cUbo.VORTICITY_COEF, 0.000001f, 0.000001f, 0.1f, "%.6f");
    }
    ImGui::NewLine();

    if (ImGui::CollapsingHeader("Waves")) {
        ImGui::Checkbox("Activate waves", &activateWaves);
        ImGui::DragFloat("Wall forward speed", &wallForwardSpeed, 0.001f);
        ImGui::DragFloat("Wall backward speed", &wallBackwardSpeed, 0.001f);
        ImGui::DragFloat("Wave limit", &wallLimit, 0.001f);
    }
    ImGui::NewLine();

    if (ImGui::CollapsingHeader("Rendering")) {
        ImGui::DragFloat("Particle Radius", &gUbo.radius, 0.0001f, 0.0001f, 0.2f);
        ImGui::DragFloat("Transparency", &gUbo.transparency, 0.001f, 0.001f, 4.0f);
        ImGui::Checkbox("Render SkyBox", &renderSkybox);

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

        ImGui::SetCursorPosX(10.0f);
        if (ImGui::CollapsingHeader("Blur Options")) {
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
    showParticles = gUbo.renderType == 0;
    ImGui::NewLine();

    if (singleStep) {
        pausedSimulation = true;
        singleStep = false;
    }

    if ((ImGui::Button("Step") || glfwIsKeyJustPressed<GLFW_KEY_P>()) && !controlMode) {
        singleStep = true;
        pausedSimulation = false;
    }

    if (ImGui::Button("Reset") || glfwIsKeyJustPressed<GLFW_KEY_R>()) {
        initializeObjects(true);
        controlMode = false;
//        pausedSimulation = false;
    }

    if (hardResetFrame > 2) {
        hardResetFrame = 2;
        enableEmergencyExit();
    }
    if (ImGui::Button("Hard Reset") || hardReset) {
        hardResetFrame = 0;
        disableEmergencyExit();
        onCreate();
        controlMode = false;
        pausedSimulation = false;
    }
    hardResetFrame++;

    if (ImGui::Button("Reset and enter control mode")) {
        pausedSimulation = true;
        controlMode = true;
    }

    if (ImGui::Button("Save Configurations")) {
        pausedSimulation = true;
        isSaveWindowOpen = true;
        saveFileName = "Enter file name";
        disableKeyboardControl = true;
    }

    ImGui::Text("Using %s", device.getPhysicalDeviceName().c_str());
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                ImGui::GetIO().Framerate);
    ImGui::End();

    if (controlMode) {
        ImGui::Begin("Control Mode");

        auto getParticleShapeSize = [this]() -> glm::vec3 {
            return {float(numParticlesXZ.x) * particleSpacing + cUbo.H,
                    float(NUM_PARTICLES) / float(numParticlesXZ.x * numParticlesXZ.y) * particleVerticalSpacing + cUbo.H,
                    float(numParticlesXZ.y) * particleSpacing + cUbo.H
            };
        };

        int temp = (int) NUM_PARTICLES;
        ImGui::SliderInt("Num Particles", &temp, 16, MAX_PARTICLES);

        ImGui::DragFloat("Spacing", &particleSpacing, 0.01f*cUbo.H, 0.2f*cUbo.H, 2*cUbo.H);
        ImGui::DragFloat("Spacing vert", &particleVerticalSpacing, 0.01f*cUbo.H, 0.2f*cUbo.H, 2*cUbo.H);

        auto particleShapeSize = getParticleShapeSize();

        if (temp != NUM_PARTICLES) {
            NUM_PARTICLES = (uint32_t) temp;

            numParticlesXZ = glm::ivec2(int(std::cbrt(float(temp))));
            particleShapeSize = getParticleShapeSize();
            for (int i = 0; i < 3; i++)
                if (cUbo.BOUNDARY_SIZE[i] < particleShapeSize[i] + 2*cUbo.EPS)
                    cUbo.BOUNDARY_SIZE[i] = particleShapeSize[i] + 2*cUbo.EPS;
        }


        glm::vec3 newBoundSize = cUbo.BOUNDARY_SIZE;
        auto maxBound = glm::vec3(MAX_BOUND);

        ImGui::CDragFloatRanged3("Boundary Size", &newBoundSize[0], 0.5f*cUbo.H, &particleShapeSize[0], &maxBound[0]);
        for (int i = 0; i < 3; i++){
            if (newBoundSize[i] != cUbo.BOUNDARY_SIZE[i] && newBoundSize[i] >= particleShapeSize[i] + 2*cUbo.EPS) {
                cUbo.BOUNDARY_SIZE[i] = newBoundSize[i];
            }
        }

        auto minPos = glm::vec3(cUbo.EPS);
        auto maxPos = glm::vec3(cUbo.BOUNDARY_SIZE.x - particleShapeSize.x, std::max(cUbo.BOUNDARY_SIZE.y - particleShapeSize.y - 3*cUbo.EPS, cUbo.EPS), cUbo.BOUNDARY_SIZE.z - particleShapeSize.z);
        ImGui::CSliderFloatRanged3("Initial Pos", &initialPos[0], &minPos[0], &maxPos[0]);

        auto numParticleMin = glm::ivec2(2);
        auto numParticleMax = glm::ivec2(int(cUbo.BOUNDARY_SIZE.x/particleSpacing - cUbo.H), int(cUbo.BOUNDARY_SIZE.z/particleSpacing - cUbo.H));
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

        ImGui::NewLine();
        if (ImGui::CollapsingHeader("Rigid Objects", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (!rigidObjects.empty()) {
                glm::vec3 rigidObjTranslation = rigidObjects[selectedRigidObj].getTranslation();
                glm::vec3 minObjPos = -2.0f*cUbo.BOUNDARY_SIZE;
                glm::vec3 maxObjPos = 2.0f*cUbo.BOUNDARY_SIZE;
                ImGui::SetCursorPosX(10.0f);
                ImGui::CDragFloatRanged3("Translation", &rigidObjTranslation[0], 0.25f*cUbo.H, &minObjPos[0], &maxObjPos[0]);
                rigidObjects[selectedRigidObj].translate(rigidObjTranslation - rigidObjects[selectedRigidObj].getTranslation());

                std::string curItem = rigidObjectsNames[selectedRigidObj];
                if (ImGui::BeginCombo("##combo", curItem.c_str())) {
                    for (uint32_t i = 0; i < rigidObjectsNames.size(); i++){
                        bool isSelected = (curItem == rigidObjectsNames[i]);
                        if (ImGui::Selectable(rigidObjectsNames[i].c_str(), isSelected)) {
                            selectedRigidObj = i;
                        }
                        if (isSelected)
                            ImGui::SetItemDefaultFocus();
                    }
                    ImGui::EndCombo();
                }
                if (ImGui::Button("Remove this object")) {
                    vkDeviceWaitIdle(device.device());
                    NUM_PARTICLES -= rigidObjects[selectedRigidObj].numParticles();
                    rigidObjects.erase(rigidObjects.begin() + selectedRigidObj);
                    rigidObjectsNames.erase(rigidObjectsNames.begin() + selectedRigidObj);
                    selectedRigidObj = 0;
                }
            }

            ImGui::SetCursorPosX(10.0f);
            if(ImGui::Button("Add rigid object")) {
                isAddWindowOpen = true;
            }
        }

        if (ImGui::Button("Launch and close")){
            initializeObjects(true);
            controlMode = false;
            pausedSimulation = false;
        }
        ImGui::End();
    }

    static uint32_t selectedType = 0;
    if (isAddWindowOpen) {
        ImGui::Begin("Add Rigid Object");

        std::string curItem = rigidObjectTypes[selectedType].second;
        if (ImGui::BeginCombo("##combo", curItem.c_str())) {
            for (uint32_t i = 0; i < rigidObjectTypes.size(); i++){
                bool isSelected = (curItem == rigidObjectTypes[i].second);
                if (ImGui::Selectable(rigidObjectTypes[i].second.c_str(), isSelected)) {
                    selectedType = i;
                }
                if (isSelected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        if (ImGui::Button("Add")) {
            addRigidObject(selectedType);
            isAddWindowOpen = false;
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) {
            isAddWindowOpen = false;
        }
        ImGui::End();
    }

    if (isSaveWindowOpen) {
        ImGui::Begin("Save Configuration");

        ImGui::InputText("File Name", &saveFileName);

        if (ImGui::Button("Save") && saveFileName != "Enter file name") {
            PBFSceneManager::saveScene(*this, saveFileName + ".json");
            isSaveWindowOpen = false;
            pausedSimulation = controlMode;
            disableKeyboardControl = false;
            disableEmergencyExit();
            hardResetFrame = 0;
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) {
            isSaveWindowOpen = false;
            pausedSimulation = controlMode;
            disableKeyboardControl = false;
        }

        ImGui::End();
    }

    if (isLoadWindowOpen) {
        ImGui::Begin("Load Preset");

        ImGui::Text("Choose preset file");

        if (ImGui::BeginCombo("##combo", curFile.data())) {
            for (const std::string_view preset : presets){
                const std::string_view name = preset.substr(preset.find_last_of('/')+1);
                bool isSelected = (curFile == name);
                if (ImGui::Selectable(name.data(), isSelected)) {
                    curFile = name;
                }
                if (isSelected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        if (ImGui::Button("Load")) {
            PBFSceneManager::loadScene(*this, PRESET_DIR + curFile.data());
            isLoadWindowOpen = false;
            pausedSimulation = controlMode;
            if (!pausedSimulation) {
                initializeObjects(true);
            }
            disableKeyboardControl = false;
            disableEmergencyExit();
            hardResetFrame = 0;
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel")) {
            isLoadWindowOpen = false;
            pausedSimulation = controlMode;
            disableKeyboardControl = false;
        }

        ImGui::End();
    }

}

void PBFGPU3DSim::addRigidObject(uint32_t type) {
    rigidObjectsNames.emplace_back("city");
//    rigidObjects.emplace_back(device, "../Models/"+rigidObjectTypes[type].second+".obj", rockTex, 0.1f, cUbo.H/2);
    rigidObjects.emplace_back(device, "../Models/city.obj", rockTex, 0.0005f, cUbo.H/2);
//    rigidObjects.emplace_back(device, "../Models/bunny.obj", rockTex, 20.0f, cUbo.H/2);
    selectedRigidObj = rigidObjects.size() - 1;
    NUM_PARTICLES += rigidObjects[rigidObjects.size()-1].numParticles();
    NUM_RIGID_PARTICLES += rigidObjects[rigidObjects.size()-1].numParticles();
    hardResetFrame = 0;
    disableEmergencyExit();
}

void PBFGPU3DSim::onResize(int width, int height) {
    gUbo.screenHeight = (float) height;
    gUbo.screenWidth = (float) width;
    depthPass.changeImageSize(renderer.getSwapChainExtent());
    thicknessPass.changeImageSize(renderer.getSwapChainExtent());
    scenePass.changeImageSize(renderer.getSwapChainExtent());
    smoothPass.changeImageSize(renderer.getSwapChainExtent());
    auto debugDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
            .addSameTypeBindings(1, 6, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, VK_SHADER_STAGE_FRAGMENT_BIT)
            .build();

    shadingDescriptorSets[0] = createDescriptorSets(
            debugDescriptorLayout,
            {graphicsUniformBuffers[0]->descriptorInfo()},
            {depthPass.descriptorInfo(), thicknessPass.descriptorInfo(), scenePass.descriptorInfo(), depthPass.descriptorInfo(),
             plane.textureInfo(), skybox.descriptorInfo()}
    );

    shadingDescriptorSets[1] = createDescriptorSets(
            debugDescriptorLayout,
            {graphicsUniformBuffers[0]->descriptorInfo()},
            {depthPass.descriptorInfo(), thicknessPass.descriptorInfo(), scenePass.descriptorInfo(), smoothPass.descriptorInfo(),
             plane.textureInfo(), skybox.descriptorInfo()}
    );

    shadingDescriptorSets[2] = createDescriptorSets(
            debugDescriptorLayout,
            {graphicsUniformBuffers[0]->descriptorInfo()},
            {depthPass.descriptorInfo(), thicknessPass.descriptorInfo(), scenePass.descriptorInfo(), smoothPass.additionalImageDescriptorInfo(),
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


