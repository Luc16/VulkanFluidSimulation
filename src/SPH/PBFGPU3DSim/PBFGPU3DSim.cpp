//
// Created by luc on 01/06/23.
//

#include "PBFGPU3DSim.h"

void PBFGPU3DSim::onCreate() {
    camera.m_translation = {-174.012f, 194.745f, 193.005f};
    camera.m_rotation = {0.569391f, 1.95856f, 3.14159f};
    camera.updateView();
    auto extent = window.extent();
    camera.setOrthographicProjection(0.0f, (float) extent.width, (float) extent.height, 0.0f, 0.1f, 1000.f);
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
            info.attributeDescription.clear();
            info.attributeDescription.push_back({0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0});
        });
    }

    createComputeDescriptorSets();

    predictPositionKernel.createPipeline();
    densityKernel.createPipeline();
    lambdaKernel.createPipeline();
    posCorrectionKernel.createPipeline();
    correctPositionsKernel.createPipeline();
    updateVelocitiesKernel.createPipeline();
    applyViscAndComputeVortKernel.createPipeline();
    applyVortAndUpdatePos.createPipeline();

    gridHandler.createSystems();
}

void PBFGPU3DSim::createComputeDescriptorSets() {
    for (uint32_t i = 0; i < positionBuffers.size(); i++) {
        predictPositionKernel.descSets[i] = vkb::DescriptorWriter::createSingleDescriptorSet(globalDescriptorPool, predictPositionKernel.layout, {
                {computeUniformBuffer->descriptorInfo()},
                {positionBuffers[i]->descriptorInfo()},
                {velocityBuffers[i]->descriptorInfo()},
                {predPosBuffers[i]->descriptorInfo()},
        });

        densityKernel.descSets[i] = vkb::DescriptorWriter::createSingleDescriptorSet(globalDescriptorPool, densityKernel.layout, {
                {computeUniformBuffer->descriptorInfo()},
                {gridHandler.gridDescriptorInfo()},
                {predPosBuffers[(i + 1) % predPosBuffers.size()]->descriptorInfo()},
                {densityBuffer->descriptorInfo()},
                {gridIdxBuffer->descriptorInfo()}
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

        correctPositionsKernel.descSets[i] = vkb::DescriptorWriter::createSingleDescriptorSet(globalDescriptorPool, correctPositionsKernel.layout, {
                {computeUniformBuffer->descriptorInfo()},
                {predPosBuffers[(i + 1) % predPosBuffers.size()]->descriptorInfo()},
                {correctionBuffer->descriptorInfo()},
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
    gUbo.viewProj = camera.getProjection()*camera.getView();
    gUbo.cameraPos = camera.m_translation;
    gUbo.radius = cUbo.H;
    cUbo.numParticles = INSTANCE_COUNT;
    cUbo.GRID_SIZE = uint32_t(cUbo.BOUNDARY_SIZE.x/cUbo.H)*uint32_t(cUbo.BOUNDARY_SIZE.y/cUbo.H)*uint32_t(cUbo.BOUNDARY_SIZE.z/cUbo.H) + 1;

    vkDeviceWaitIdle(device.device());

    particles.resize(INSTANCE_COUNT);

    plane.setScale(cUbo.BOUNDARY_SIZE);

    auto accPos = initialPos;
    particleSpacing = cUbo.H + 0.2f;

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
                accPos.y += particleSpacing;
                accPos.z = initialPos.z;
            }
        }
    }

    computeFrameIdx = 0;

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
    densityBuffer = makeBuffer(particles.density.size()*sizeof(float));
    lambdaBuffer = makeBuffer(particles.lambda.size()*sizeof(float));
    gridIdxBuffer = makeBuffer(particles.idx.size() * sizeof(uint32_t));

    for (uint32_t i = 0; i < particleBarrierData.size(); i++){
        particleBarrierData[i] = {
                positionBuffers[i]->getBarrierData(),
                velocityBuffers[i]->getBarrierData(),
                correctionBuffer->getBarrierData(),
                vorticityBuffer->getBarrierData(),
                densityBuffer->getBarrierData(),
                lambdaBuffer->getBarrierData(),
        };
    }


    if (activateRandomOffsets){
        gridHandler.createDescriptorsAndBuffers(
                globalDescriptorPool,
                cUbo.GRID_SIZE,
                {cUbo.numParticles, cUbo.H, cUbo.BOUNDARY_SIZE},
                gridIdxBuffer,
                positionBuffers,
                velocityBuffers,
                predPosBuffers);
    }

    if (objectsInitialized) globalDescriptorPool->freeDescriptors({
            predictPositionKernel.descSets[0],
            predictPositionKernel.descSets[1],
            densityKernel.descSets[0],
            densityKernel.descSets[1],
            lambdaKernel.descSets[0],
            lambdaKernel.descSets[1],
            posCorrectionKernel.descSets[0],
            posCorrectionKernel.descSets[1],
            correctPositionsKernel.descSets[0],
            correctPositionsKernel.descSets[1],
            updateVelocitiesKernel.descSets[0],
            updateVelocitiesKernel.descSets[1],
            applyViscAndComputeVortKernel.descSets[0],
            applyViscAndComputeVortKernel.descSets[1],
            applyVortAndUpdatePos.descSets[0],
            applyVortAndUpdatePos.descSets[1],
    });
    createComputeDescriptorSets();

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

    if (!controlMode) {
        updateSimulation();
        if (activateTimer) {
            auto time = std::chrono::high_resolution_clock::now();
            computeTime = std::chrono::duration<float, std::chrono::milliseconds::period>(time - currentTime).count();
            currentTime = time;
        }

        renderer.runFrame([this](VkCommandBuffer commandBuffer){
            renderObjects();
        }, computeHandler.currentSemaphore(renderer.currentFrame()), vkb::ComputeShaderHandler::waitStages());
    } else {
        initializeObjects(false);
        renderer.runFrame([this](VkCommandBuffer commandBuffer){
            renderObjects();
        });
    }

    if (activateTimer) drawTime = std::chrono::duration<float, std::chrono::milliseconds::period>(std::chrono::high_resolution_clock::now() - currentTime).count();
}

void PBFGPU3DSim::renderObjects() {
    showImGui();

    renderer.runRenderPass([this](VkCommandBuffer& commandBuffer){

        defaultSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);
        plane.render(defaultSystem, commandBuffer);

        particleSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);
        VkBuffer vb = positionBuffers[computeFrameIdx]->getBuffer();
        VkDeviceSize offsets[] = {0};
        vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
        vkCmdDraw(commandBuffer, INSTANCE_COUNT, 1, 0, 0);

    });
}

void PBFGPU3DSim::updateSimulation() {

    computeHandler.runCompute(renderer.currentFrame(), [this](VkCommandBuffer computeCommandBuffer){
        uint32_t blockSize = INSTANCE_COUNT/256 + (1 - (INSTANCE_COUNT%256 == 0));

        predictPositionKernel.bindAndDispatch(computeCommandBuffer, computeFrameIdx, blockSize, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);

        gridHandler.createGrid(computeCommandBuffer, computeFrameIdx);

        vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);

        for (uint32_t i = 0; i < jacobiIterations; i++){
            densityKernel.bindAndDispatch(computeCommandBuffer,computeFrameIdx, blockSize, 1, 1);

            vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);

            lambdaKernel.bindAndDispatch(computeCommandBuffer, computeFrameIdx, blockSize, 1, 1);

            vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);

            posCorrectionKernel.bindAndDispatch(computeCommandBuffer, computeFrameIdx, blockSize, 1, 1);

            vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);

            correctPositionsKernel.bindAndDispatch(computeCommandBuffer, computeFrameIdx, blockSize, 1, 1);

            vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);
        }

        updateVelocitiesKernel.bindAndDispatch(computeCommandBuffer, computeFrameIdx, blockSize, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);
//
        applyViscAndComputeVortKernel.bindAndDispatch(computeCommandBuffer, computeFrameIdx, blockSize, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);

        applyVortAndUpdatePos.bindAndDispatch(computeCommandBuffer, computeFrameIdx, blockSize, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);

    });

    computeFrameIdx = (computeFrameIdx + 1) % positionBuffers.size();
}

void PBFGPU3DSim::updateUniformBuffers(uint32_t frameIndex, float deltaTime){
    camera.setPerspectiveProjection(glm::radians(50.f), renderer.getSwapChainAspectRatio(), 0.1f, 1000.f);
    gUbo.viewProj = camera.getProjection()*camera.getView();
    gUbo.cameraPos = camera.m_translation;
    graphicsUniformBuffers[frameIndex]->write(&gUbo);

    cUbo.planeY = plane.m_translation.y;
    cUbo.G = gravityFactor*glm::vec3(0.0f, -10.0f, 0.0f);
    computeUniformBuffer->write(&cUbo);
}

void PBFGPU3DSim::showImGui(){
    ImGui::Begin("Control Panel");

    ImGui::Text("Rendering %d instances", INSTANCE_COUNT);
    ImGui::Text("Grid size: %d", cUbo.GRID_SIZE);
    ImGui::Checkbox("Display time", &activateTimer);
    if(activateTimer){
        ImGui::Text("Cpu time: %f ms", cpuTime);
        ImGui::Text("Compute time: %f ms", computeTime);
        ImGui::Text("Draw time: %f ms", drawTime);
    }

    if (ImGui::Button("Reset and enter control mode")) controlMode = true;

    ImGui::SliderFloat("Gravity", &gravityFactor, 1.f, 1000.f);
    ImGui::DragFloat("Viscosity", &cUbo.VISC, 0.001f, 0.001f, 5.0f);
    ImGui::DragFloat("DeltaTime", &cUbo.DT, 0.00005f, 0.0005f, 0.02f, "%.5f");
    ImGui::DragFloat("Rest Density", &cUbo.REST_DENS, 0.1f, 0.1f, 1000.0f, "%.4f");
    if (cUbo.REST_DENS == 0) cUbo.REST_DENS = 0.1f;
    ImGui::DragFloat("Mass", &cUbo.MASS, 0.001f, 0.001f, 40.0f, "%.1f");
    ImGui::DragFloat("CFM", &cUbo.CFM, 1.0f, 1.0f, 1000.0f, "%.1f");
    ImGui::DragFloat("ARTIFICIAL PRESSURE", &cUbo.ART_PRESSURE_COEF, 0.01f, 0.01f, 10.0f, "%.2f");
    ImGui::DragFloat("VORTICITY COEF", &cUbo.VORTICITY_COEF, 0.000001f, 0.000001f, 0.1f, "%.6f");

    ImGui::SliderFloat("Plane Y", &plane.m_translation.y, -100.0f, 100.0f);

    if (ImGui::Button("Reset")) initializeObjects(true);

    ImGui::Text("Using %s", device.getPhysicalDeviceName().c_str());
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                ImGui::GetIO().Framerate);
    ImGui::End();


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
        }
        ImGui::End();
    }

}

void PBFGPU3DSim::compileShaders() {
    for (uint32_t i = 0; i < shaders.size(); i++) {
        std::string command{"glslc "};
        if (i < 4) {
            command += RENDER_SHADER_DIR;
        } else if (i < 9) {
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