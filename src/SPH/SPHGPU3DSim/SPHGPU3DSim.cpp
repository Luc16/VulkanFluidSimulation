//
// Created by luc on 01/06/23.
//

#include "SPHGPU3DSim.h"

void SPHGPU3DSim::onCreate() {
    camera.m_translation = {-56.9685f, 51.9174f, 65.334};
    camera.m_rotation = {0.416948f, 1.95856f, 3.14159};
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
        particleSystem.createPipeline(renderer.renderPass(), particleShaderPaths, [this](vkb::GraphicsPipeline::PipelineConfigInfo& info) {
            info.inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
            info.bindingDescription.clear();
            info.bindingDescription.push_back({0, sizeof(Particle), VK_VERTEX_INPUT_RATE_VERTEX});
            info.attributeDescription.clear();
            info.attributeDescription.push_back({0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Particle, position)});
            info.attributeDescription.push_back({1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Particle, color)});
        });
    }

    auto computeDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .addBinding({2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .build();

    createComputeDescriptorSets(computeDescriptorLayout);
    calculateForcesComputeSystem.createPipelineWithLayout(computeDescriptorLayout.descriptorSetLayout());
    integrateComputeSystem.createPipelineWithLayout(computeDescriptorLayout.descriptorSetLayout());
    calculateDensityPressureComputeSystem.createPipelineWithLayout(computeDescriptorLayout.descriptorSetLayout());

    gridHandler.createSystems();
}

void SPHGPU3DSim::createComputeDescriptorSets(vkb::DescriptorSetLayout &layout) {
    VkDescriptorSetLayout vkLayout = layout.descriptorSetLayout();
    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = globalDescriptorPool->descriptorPool();
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &vkLayout;

    for (uint32_t i = 0; i < computeDescriptorSets.size(); i++) {

        if (vkAllocateDescriptorSets(device.device(), &allocInfo, &computeDescriptorSets[i]) != VK_SUCCESS) {
            throw std::runtime_error("failed to allocate descriptor sets!");
        }

        auto writer = vkb::DescriptorWriter(layout, *globalDescriptorPool);

        auto uniformBufferInfo = computeUniformBuffer->descriptorInfo();
        writer.writeBuffer(0, &uniformBufferInfo);

        auto gridBufferInfo = gridHandler.gridDescriptorInfo();
        writer.writeBuffer(1, &gridBufferInfo);

        auto particleBufferInfo = particleBuffers[(i + 1) % particleBuffers.size()]->descriptorInfo();
        writer.writeBuffer(2, &particleBufferInfo);

        writer.build(computeDescriptorSets[i], false);

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
    auto spherePerSide = (uint32_t) std::cbrt(INSTANCE_COUNT);
    float step = cUbo.H + 0.01f;

    uint32_t count = 0;
    for (uint32_t i = 0; i < particles.size(); i++) {
        auto& sphere = particles[i];
        sphere.color = glm::vec3(0.2f, 0.6f, 1.0f);
        sphere.position = accPos;
        if (activateRandomOffsets) sphere.position += glm::vec3(randomFloat(-cUbo.H/5, cUbo.H/5), randomFloat(-cUbo.H/5, cUbo.H/5), randomFloat(-cUbo.H/5, cUbo.H/5));
        sphere.velocity = glm::vec3(0.0f);
        sphere.force = glm::vec3(0.0f);
        accPos.x += step;

        if (i % numParticlesXZ.x == numParticlesXZ.x - 1) {
            count++;
            accPos.z += step;
            accPos.x = initialPos.x;
            if (count == numParticlesXZ.y) {
                count = 0;
                accPos.y += step;
                accPos.z = initialPos.z;
            }
        }
    }

    computeFrameIdx = 0;
    for (uint32_t i = 0; i < particleBuffers.size(); i++){
        particleBuffers[i] = std::make_unique<vkb::Buffer>(device, particles.size() * sizeof(Particle),
                                                       VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                                       VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        if (i == 0) vkb::Buffer::writeVectorToBuffer(device, particleBuffers[i], particles);
    }


    if (activateRandomOffsets){
        gridHandler.createDescriptorsAndBuffers(
                globalDescriptorPool,
                cUbo.GRID_SIZE,
                {cUbo.numParticles, cUbo.H, cUbo.BOUNDARY_SIZE},
                particleBuffers);
    }

    auto computeDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .addBinding({2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .build();

    if (computeDescriptorSets[0]) globalDescriptorPool->freeDescriptors({computeDescriptorSets[0], computeDescriptorSets[1]});
    createComputeDescriptorSets(computeDescriptorLayout);


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
        VkBuffer vb = particleBuffers[computeFrameIdx]->getBuffer();
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

        vkb::ComputeShaderHandler::computeBarrier(computeCommandBuffer, particleBuffers[computeFrameIdx]);

        calculateDensityPressureComputeSystem.bindAndDispatch(computeCommandBuffer,
                                                              &computeDescriptorSets[computeFrameIdx],
                                                              blockSize, 1, 1);

        vkb::ComputeShaderHandler::computeBarrier(computeCommandBuffer, particleBuffers[computeFrameIdx]);

        calculateForcesComputeSystem.bindAndDispatch(computeCommandBuffer,
                                                     &computeDescriptorSets[computeFrameIdx],
                                                     blockSize, 1, 1);

        vkb::ComputeShaderHandler::computeBarrier(computeCommandBuffer, particleBuffers[computeFrameIdx]);

        integrateComputeSystem.bindAndDispatch(computeCommandBuffer,
                                               &computeDescriptorSets[computeFrameIdx],
                                               blockSize, 1, 1);

    });

    computeFrameIdx = (computeFrameIdx + 1) % particleBuffers.size();
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
            return {float(numParticlesXZ.x) * cUbo.H + cUbo.H,
                    float(INSTANCE_COUNT) / float(numParticlesXZ.x * numParticlesXZ.y) * cUbo.H + cUbo.H,
                    float(numParticlesXZ.y) * cUbo.H + cUbo.H
            };
        };

        int temp = (int) INSTANCE_COUNT;
        ImGui::SliderInt("Num Particles", &temp, 16, 500000);


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
        auto maxBound = glm::vec3(1000.0f);

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
