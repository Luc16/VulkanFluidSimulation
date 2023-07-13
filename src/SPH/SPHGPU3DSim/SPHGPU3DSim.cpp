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
        instanceSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), 0);
        instanceSystem.createPipeline(renderer.renderPass(), instanceShaderPaths,
                                      [this](vkb::GraphicsPipeline::PipelineConfigInfo &info) {
                                          info.bindingDescription.push_back(instancedSpheres.getBindingDescription());
                                          info.attributeDescription.push_back(
                                                  {4, 1, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Particle, position)});
                                          info.attributeDescription.push_back(
                                                  {5, 1, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Particle, color)});
                                          info.attributeDescription.push_back(
                                                  {6, 1, VK_FORMAT_R32_SFLOAT, offsetof(Particle, scale)});
                                      });
    }

    auto computeDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .addBinding({2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .addBinding({3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .build();

    createComputeDescriptorSets(computeDescriptorLayout);
    calculateForcesComputeSystem.createPipelineLayout(computeDescriptorLayout.descriptorSetLayout());
    calculateForcesComputeSystem.createPipeline(calculateForcesShaderPath);

    integrateComputeSystem.createPipelineLayout(computeDescriptorLayout.descriptorSetLayout());
    integrateComputeSystem.createPipeline(integrateShaderPath);

    calculateDensityPressureComputeSystem.createPipelineLayout(computeDescriptorLayout.descriptorSetLayout());
    calculateDensityPressureComputeSystem.createPipeline(calculateDensityPressureShaderPath);

    insertParticlesComputeSystem.createPipelineLayout(computeDescriptorLayout.descriptorSetLayout());
    insertParticlesComputeSystem.createPipeline(insertParticlesShaderPath);

    scanComputeSystem.createPipelineLayout(computeDescriptorLayout.descriptorSetLayout());
    scanComputeSystem.createPipeline(scanShaderPath);

}

void SPHGPU3DSim::createComputeDescriptorSets(vkb::DescriptorSetLayout &layout) {
    VkDescriptorSetLayout vkLayout = layout.descriptorSetLayout();
    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = globalDescriptorPool->descriptorPool();
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &vkLayout;

    if (vkAllocateDescriptorSets(device.device(), &allocInfo, &computeDescriptorSet) != VK_SUCCESS) {
        throw std::runtime_error("failed to allocate descriptor sets!");
    }

    auto writer = vkb::DescriptorWriter(layout, *globalDescriptorPool);

    auto uniformBufferInfo = computeUniformBuffer->descriptorInfo();
    writer.writeBuffer(0, &uniformBufferInfo);

    auto particleBufferInfo = instancedSpheres.descriptorInfo();
    writer.writeBuffer(1, &particleBufferInfo);

    auto gridBufferInfo = gridBuffer->descriptorInfo();
    writer.writeBuffer(2, &gridBufferInfo);

    auto gridOutBufferInfo = gridOutBuffer->descriptorInfo();
    writer.writeBuffer(3, &gridOutBufferInfo);

    writer.build(computeDescriptorSet, false);


}

void SPHGPU3DSim::initializeObjects(bool activateRandomOffsets) {
    gUbo.view = camera.getView();
    gUbo.proj = camera.getProjection();
    cUbo.numParticles = INSTANCE_COUNT;
    cUbo.GRID_SIZE = uint32_t((cUbo.BOUNDARY_SIZE/cUbo.H)*(cUbo.BOUNDARY_SIZE/cUbo.H)*(cUbo.BOUNDARY_SIZE/cUbo.H));

    vkDeviceWaitIdle(device.device());

    instancedSpheres.resizeBuffer(INSTANCE_COUNT);
    grid.resize(INSTANCE_COUNT);

    plane.setScale(cUbo.BOUNDARY_SIZE);

    auto accPos = initialPos;
    auto spherePerSide = (uint32_t) std::cbrt(INSTANCE_COUNT);
    float step = cUbo.H + 0.01f;

    for (uint32_t i = 0; i < instancedSpheres.size(); i++) {
        auto& sphere = instancedSpheres[i];
        sphere.color = glm::vec3(0.2f, 0.6f, 1.0f);
        sphere.scale = 0.8f*cUbo.H;
        sphere.position = accPos;
        if (activateRandomOffsets) sphere.position += glm::vec3(randomFloat(-H/5, H/5), randomFloat(-H/5, H/5), randomFloat(-H/5, H/5));
        sphere.velocity = glm::vec3(0.0f);
        sphere.force = glm::vec3(0.0f);
        accPos.x += step;

        if (i % spherePerSide == spherePerSide - 1) {
            accPos.z += step;
            accPos.x = initialPos.x;
            if (i % (spherePerSide * spherePerSide) == (spherePerSide * spherePerSide) - 1) {
                accPos.y += step;
                accPos.z = initialPos.z;
            }
        }
    }
    instancedSpheres.updateBuffer();

    for (int i = 0; i < 10; i++){
        grid[i] = i % 5;
    }
    grid[0] = 4;

    std::cout << "Grid: ";
    for (int i = 0; i < 16; i++){
        std::cout << grid[i] << ", ";
    }
    std::cout << "\n";

    VkDeviceSize bufferSize = sizeof(uint32_t) * cUbo.GRID_SIZE;
    vkb::Buffer stagingBuffer(device, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    stagingBuffer.singleWrite(grid.data());
    gridBuffer = std::make_unique<vkb::Buffer>(device, bufferSize,
                                               VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                               VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    gridOutBuffer = std::make_unique<vkb::Buffer>(device, bufferSize,
                                                  VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                                  VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    device.copyBuffer(stagingBuffer.getBuffer(), gridBuffer->getBuffer(), bufferSize);
    device.copyBuffer(stagingBuffer.getBuffer(), gridOutBuffer->getBuffer(), bufferSize);

    auto computeDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .addBinding({2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .addBinding({3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .build();

    if (computeDescriptorSet) globalDescriptorPool->freeDescriptors({computeDescriptorSet});
    createComputeDescriptorSets(computeDescriptorLayout);

    barrierBuffers = {
            instancedSpheres.getBarrierData(),
            {gridBuffer->getBuffer(), gridBuffer->getSize()}
    };

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

        instanceSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);
        instancedSpheres.render(instanceSystem, commandBuffer);

    });
}

void SPHGPU3DSim::updateSimulation() {
    computeHandler.runCompute(renderer.currentFrame(), [this](VkCommandBuffer computeCommandBuffer){
        uint32_t blockSize = INSTANCE_COUNT/256 + (1 - (INSTANCE_COUNT%256 == 0));

        insertParticlesComputeSystem.bindAndDispatch(computeCommandBuffer,
                                                              &computeDescriptorSet,
                                                              blockSize, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, barrierBuffers);

        calculateDensityPressureComputeSystem.bindAndDispatch(computeCommandBuffer,
                                                              &computeDescriptorSet,
                                                              blockSize, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, barrierBuffers);

        calculateForcesComputeSystem.bindAndDispatch(computeCommandBuffer,
                                                     &computeDescriptorSet,
                                                     blockSize, 1, 1);


        // Add memory barrier to ensure that the computer shader has finished writing to the buffer
        vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, barrierBuffers);

        integrateComputeSystem.bindAndDispatch(computeCommandBuffer,
                                               &computeDescriptorSet,
                                               blockSize, 1, 1);

    });
}

void SPHGPU3DSim::updateUniformBuffers(uint32_t frameIndex, float deltaTime){
    camera.setPerspectiveProjection(glm::radians(50.f), renderer.getSwapChainAspectRatio(), 0.1f, 1000.f);
    gUbo.view = camera.getView();
    gUbo.proj = camera.getProjection();
    graphicsUniformBuffers[frameIndex]->write(&gUbo);

    cUbo.deltaTime = deltaTime;
    cUbo.planeY = plane.m_translation.y;
    cUbo.G = gravityFactor*glm::vec3(0.0f, -10.0f, 0.0f);
    computeUniformBuffer->write(&cUbo);
}

void SPHGPU3DSim::showImGui(){
    ImGui::Begin("Control Panel");

    ImGui::Text("Rendering %d instances", INSTANCE_COUNT);
    ImGui::Checkbox("Display time", &activateTimer);
    if(activateTimer){
        ImGui::Text("Cpu time: %f ms", cpuTime);
        ImGui::Text("Compute time: %f ms", computeTime);
        ImGui::Text("Draw time: %f ms", drawTime);
    }

    if (ImGui::Button("Reset and enter control mode")) controlMode = true;

    ImGui::DragFloat("Gravity", &gravityFactor, 1.f, 1.f, 1000.f);

    ImGui::SliderFloat("Plane Y", &plane.m_translation.y, -100.0f, 100.0f);

    if (ImGui::Button("Reset")) initializeObjects(true);


    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                ImGui::GetIO().Framerate);
    ImGui::End();


    if (controlMode) {
        ImGui::Begin("Control Mode");

        float particleCubeSize = std::cbrt(float(INSTANCE_COUNT))*cUbo.H + cUbo.EPS;

        int temp = (int) INSTANCE_COUNT;
        ImGui::SliderInt("Num Particles", &temp, 16, 100000);
        if (temp != INSTANCE_COUNT) {
            if (cUbo.BOUNDARY_SIZE < particleCubeSize) cUbo.BOUNDARY_SIZE = particleCubeSize;
        }
        INSTANCE_COUNT = (uint32_t) temp;


        float newBoundSize = cUbo.BOUNDARY_SIZE;
        ImGui::DragFloat("Boundary Size", &newBoundSize, 1, particleCubeSize, 1000);
        if (newBoundSize != cUbo.BOUNDARY_SIZE) {
            if (newBoundSize >= particleCubeSize) cUbo.BOUNDARY_SIZE = newBoundSize;
            if (initialPos.x > cUbo.BOUNDARY_SIZE - particleCubeSize + cUbo.EPS) initialPos.x = cUbo.BOUNDARY_SIZE - particleCubeSize  + cUbo.EPS;
            if (initialPos.z > cUbo.BOUNDARY_SIZE - particleCubeSize  + cUbo.EPS) initialPos.z = cUbo.BOUNDARY_SIZE - particleCubeSize  + cUbo.EPS;
        }

        ImGui::SliderFloat("Initial Pos X", &initialPos.x, cUbo.EPS, cUbo.BOUNDARY_SIZE - particleCubeSize);
        ImGui::SliderFloat("Initial Pos Y", &initialPos.y, cUbo.EPS, cUbo.BOUNDARY_SIZE - particleCubeSize);
        ImGui::SliderFloat("Initial Pos Z", &initialPos.z, cUbo.EPS, cUbo.BOUNDARY_SIZE - particleCubeSize);

        if (ImGui::Button("Launch and close")){
            initializeObjects(true);
            controlMode = false;
        }
        ImGui::End();
    }

}

void SPHGPU3DSim::compileShaders() {
    for (auto& shaderPath : shaders) {
        std::string command{"glslc "};
        command += shaderPath;
        command += " --target-env=vulkan1.1 ";
        command += " -o ";
        command += shaderPath;
        command += ".spv";
        int status = system(command.c_str());
        if (status != 0) {
            throw std::runtime_error("Error compiling shader " + shaderPath);
        }
    }
}
