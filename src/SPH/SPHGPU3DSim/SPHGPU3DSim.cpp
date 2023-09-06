//
// Created by luc on 01/06/23.
//

#include "SPHGPU3DSim.h"

void SPHGPU3DSim::onCreate() {
    camera.m_translation = {-174.012f, 194.745f, 193.005f};
    camera.m_rotation = {0.569391f, 1.95856f, 3.14159f};
    camera.updateView();
    auto extent = window.extent();
    camera.setOrthographicProjection(0.0f, (float) extent.width, (float) extent.height, 0.0f, 0.1f, 1000.f);
    createUniformBuffers();
    initializeObjects(true);

    auto defaultDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
            .build();
    defaultDescriptorSets = createDescriptorSets(defaultDescriptorLayout,
                                                 {graphicsUniformBuffers[0]->descriptorInfo()});
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
    calculateForcesComputeSystem.createPipelineWithLayout(forcesLayout.descriptorSetLayout());
    integrateComputeSystem.createPipelineWithLayout(integrateLayout.descriptorSetLayout());
    calculateDensityPressureComputeSystem.createPipelineWithLayout(densityPressureLayout.descriptorSetLayout());

    gridHandler.createSystems();
}

void SPHGPU3DSim::createComputeDescriptorSets() {
    for (uint32_t i = 0; i < densityPressureSets.size(); i++) {
        densityPressureSets[i] = vkb::DescriptorWriter::createSingleDescriptorSet(globalDescriptorPool, densityPressureLayout, {
                {computeUniformBuffer->descriptorInfo()},
                {gridHandler.gridDescriptorInfo()},
                {positionBuffers[(i + 1) % positionBuffers.size()]->descriptorInfo()},
                {densityBuffer->descriptorInfo()},
                {pressureBuffer->descriptorInfo()},
                {gridIdxBuffer->descriptorInfo()}
        });

        forcesSets[i] = vkb::DescriptorWriter::createSingleDescriptorSet(globalDescriptorPool, forcesLayout, {
                {computeUniformBuffer->descriptorInfo()},
                {gridHandler.gridDescriptorInfo()},
                {positionBuffers[(i + 1) % positionBuffers.size()]->descriptorInfo()},
                {velocityBuffers[(i + 1) % velocityBuffers.size()]->descriptorInfo()},
                {forceBuffer->descriptorInfo()},
                {densityBuffer->descriptorInfo()},
                {pressureBuffer->descriptorInfo()},
                {gridIdxBuffer->descriptorInfo()}
        });

        integrateSets[i] = vkb::DescriptorWriter::createSingleDescriptorSet(globalDescriptorPool, integrateLayout, {
                {computeUniformBuffer->descriptorInfo()},
                {positionBuffers[(i + 1) % positionBuffers.size()]->descriptorInfo()},
                {velocityBuffers[(i + 1) % velocityBuffers.size()]->descriptorInfo()},
                {forceBuffer->descriptorInfo()},
                {densityBuffer->descriptorInfo()},
        });
    }
}

void SPHGPU3DSim::initializeObjects(bool activateRandomOffsets) {
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
        if (activateRandomOffsets) particles.position[i] += glm::vec4(randomFloat(-cUbo.H/5, cUbo.H/5), randomFloat(-cUbo.H/5, cUbo.H/5), randomFloat(-cUbo.H/5, cUbo.H/5), 0.0f);
        particles.velocity[i] = glm::vec4(0.0f);
        particles.force[i] = glm::vec4(0.0f);
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
        if (i == 0) {
            vkb::Buffer::writeVectorToBuffer(device, positionBuffers[i], particles.position);
            vkb::Buffer::writeVectorToBuffer(device, velocityBuffers[i], particles.velocity);
        }

    }

    forceBuffer = makeBuffer(particles.force.size()*sizeof(glm::vec4));
    densityBuffer = makeBuffer(particles.density.size()*sizeof(float));
    pressureBuffer = makeBuffer(particles.pressure.size()*sizeof(float));
    gridIdxBuffer = makeBuffer(particles.idx.size() * sizeof(uint32_t));

    for (uint32_t i = 0; i < particleBarrierData.size(); i++){
        particleBarrierData[i] = {
                positionBuffers[i]->getBarrierData(),
                velocityBuffers[i]->getBarrierData(),
                forceBuffer->getBarrierData(),
                densityBuffer->getBarrierData(),
                pressureBuffer->getBarrierData(),
        };
    }


    if (activateRandomOffsets){
        gridHandler.createDescriptorsAndBuffers(
                globalDescriptorPool,
                cUbo.GRID_SIZE,
                {cUbo.numParticles, cUbo.H, cUbo.BOUNDARY_SIZE},
                gridIdxBuffer,
                positionBuffers,
                velocityBuffers);
    }

    if (objectsInitialized) globalDescriptorPool->freeDescriptors({
        densityPressureSets[0],
        densityPressureSets[1],
        forcesSets[0],
        forcesSets[1],
        integrateSets[0],
        integrateSets[1]
    });
    createComputeDescriptorSets();

    objectsInitialized = true;

}

void SPHGPU3DSim::createUniformBuffers() {
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

void SPHGPU3DSim::mainLoop(float deltaTime) {
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

void SPHGPU3DSim::renderObjects() {
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

void SPHGPU3DSim::updateSimulation() {
    /* Compute Pipeline:
     *  1. reset grid
     *  2. insert particle in grid (atomic_add on the grid pos)
     *  3. prefix sum
     *  4. sort particles based on grid position
     *  5. perform SPH using the nn search with grid
    */


    computeHandler.runCompute(renderer.currentFrame(), [this](VkCommandBuffer computeCommandBuffer){
        uint32_t blockSize = INSTANCE_COUNT/256 + (1 - (INSTANCE_COUNT%256 == 0));

        gridHandler.resetGrid(computeCommandBuffer);

        gridHandler.gridBarrier(computeCommandBuffer);

        gridHandler.insertParticles(computeFrameIdx, computeCommandBuffer);

        gridHandler.gridBarrier(computeCommandBuffer);

        gridHandler.prefixSum(computeCommandBuffer);

        gridHandler.gridBarrier(computeCommandBuffer);

        gridHandler.countingSort(computeFrameIdx, computeCommandBuffer);

        vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);

        calculateDensityPressureComputeSystem.bindAndDispatch(computeCommandBuffer,
                                                              &densityPressureSets[computeFrameIdx],
                                                              blockSize, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);

        calculateForcesComputeSystem.bindAndDispatch(computeCommandBuffer,
                                                     &forcesSets[computeFrameIdx],
                                                     blockSize, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, particleBarrierData[computeFrameIdx]);

        integrateComputeSystem.bindAndDispatch(computeCommandBuffer,
                                               &integrateSets[computeFrameIdx],
                                               blockSize, 1, 1);

    });

    computeFrameIdx = (computeFrameIdx + 1) % positionBuffers.size();
}

void SPHGPU3DSim::updateUniformBuffers(uint32_t frameIndex, float deltaTime){
    camera.setPerspectiveProjection(glm::radians(50.f), renderer.getSwapChainAspectRatio(), 0.1f, 1000.f);
    gUbo.viewProj = camera.getProjection()*camera.getView();
    gUbo.cameraPos = camera.m_translation;
    graphicsUniformBuffers[frameIndex]->write(&gUbo);

    cUbo.deltaTime = deltaTime;
    cUbo.planeY = plane.m_translation.y;
    cUbo.G = gravityFactor*glm::vec3(0.0f, -10.0f, 0.0f);
    computeUniformBuffer->write(&cUbo);
}

void SPHGPU3DSim::showImGui(){
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

    ImGui::DragFloat("Gravity", &gravityFactor, 1.f, 1.f, 1000.f);
    ImGui::DragFloat("Viscosity", &cUbo.VISC, 1.f, 10.0f, 500.f);
    ImGui::DragFloat("DeltaTime", &cUbo.DT, 0.00005f, 0.0005f, 0.002f, "%.5f");

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

void SPHGPU3DSim::compileShaders() {
    for (auto& shaderName : shaders) {
        std::string command{"glslc "};
        command += SHADER_DIR;
        command += shaderName;
        command += " --target-env=vulkan1.1 ";
        command += " -o ";
        command += COMPILED_SHADER_DIR;
        command += shaderName;
        command += ".spv";
        int status = system(command.c_str());
        if (status != 0) {
            throw std::runtime_error("Error compiling shader " + shaderName);
        }
    }
}