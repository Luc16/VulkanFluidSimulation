//
// Created by luc on 01/06/23.
//

#include "SPHGPU3DSim.h"

void SPHGPU3DSim::onCreate() {
    initializeObjects();
    createUniformBuffers();

    auto defaultDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
            .build();
    defaultDescriptorSets = createDescriptorSets(defaultDescriptorLayout,{graphicsUniformBuffers[0]->descriptorInfo()});
    {
        defaultSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), sizeof(vkb::DrawableObject::PushConstantData));
        defaultSystem.createPipeline(renderer.renderPass(), shaderPaths);
    }

    {
        instanceSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), 0);
        instanceSystem.createPipeline(renderer.renderPass(), instanceShaderPaths, [this](vkb::GraphicsPipeline::PipelineConfigInfo& info) {
            info.bindingDescription.push_back(instancedSpheres.getBindingDescription());
            info.attributeDescription.push_back({4, 1, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Particle, position)});
            info.attributeDescription.push_back({5, 1, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Particle, color)});
            info.attributeDescription.push_back({6, 1, VK_FORMAT_R32_SFLOAT, offsetof(Particle, scale)});
        });
    }

    auto computeDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .addBinding({2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .build();

    createComputeDescriptorSets(computeDescriptorLayout);
    calculateForcesComputeSystem.createPipelineLayout(computeDescriptorLayout.descriptorSetLayout());
    calculateForcesComputeSystem.createPipeline(calculateForcesShaderPath);

    integrateComputeSystem.createPipelineLayout(computeDescriptorLayout.descriptorSetLayout());
    integrateComputeSystem.createPipeline(integrateShaderPath);

    calculateDensityPressureComputeSystem.createPipelineLayout(computeDescriptorLayout.descriptorSetLayout());
    calculateDensityPressureComputeSystem.createPipeline(calculateDensityPressureShaderPath);
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

    auto storageBufferInfo = instancedSpheres.descriptorInfo();
    writer.writeBuffer(1, &storageBufferInfo);

    writer.build(computeDescriptorSet, false);


}

void SPHGPU3DSim::initializeObjects() {
    camera.setViewTarget({0.0f, 10.0f, 10.0f}, {12.0f, -1.0f, -12.0f }, {0.0f, 1.0f, 0.0f});
    camera.m_rotation = {0, glm::radians(180.0f), glm::radians(180.0f)};    camera.updateView();
    auto extent = window.extent();
    camera.setOrthographicProjection(0.0f, (float) extent.width, (float) extent.height, 0.0f, 0.1f, 1000.f);
    gUbo.view = camera.getView();
    gUbo.proj = camera.getProjection();

    vkDeviceWaitIdle(device.device());

    instancedSpheres.resizeBuffer(INSTANCE_COUNT);

    plane.setScale(BOUNDARY_SIZE);

    auto accPos = glm::vec3(cUbo.EPS, cUbo.EPS, cUbo.EPS);
    auto spherePerSide = (uint32_t) std::cbrt(INSTANCE_COUNT);
    float step = cUbo.H + 0.01f;

    for (uint32_t i = 0; i < instancedSpheres.size(); i++) {
        auto& sphere = instancedSpheres[i];
        sphere.color = glm::vec3(0.2f, 0.6f, 1.0f);
        sphere.scale = 0.8f*cUbo.H;
        sphere.position = accPos + glm::vec3(randomFloat(-cUbo.H/5, cUbo.H/5), randomFloat(-cUbo.H/5, cUbo.H/5), randomFloat(-cUbo.H/5, cUbo.H/5));
        accPos.x += step;

        if (i % spherePerSide == spherePerSide - 1) {
            accPos.z += step;
            accPos.x = cUbo.EPS;
            if (i % (spherePerSide * spherePerSide) == (spherePerSide * spherePerSide) - 1) {
                accPos.y += step;
                accPos.z = cUbo.EPS;
            }
        }
    }
    instancedSpheres.updateBuffer();

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

    cUbo.width = float(window.width());
    cUbo.height = float(window.height());
}

void SPHGPU3DSim::mainLoop(float deltaTime) {
    auto currentTime = std::chrono::high_resolution_clock::now();

    cameraController.moveCamera(window.window(), deltaTime, camera);
    updateBuffers(renderer.currentFrame(), deltaTime);

    if (activateTimer) {
        auto time = std::chrono::high_resolution_clock::now();
        cpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(time - currentTime).count();
        currentTime = time;
    }

    computeHandler.runCompute(renderer.currentFrame(), [this](VkCommandBuffer computeCommandBuffer){
        calculateDensityPressureComputeSystem.bindAndDispatch(computeCommandBuffer,
                                                     &computeDescriptorSet,
                                                     INSTANCE_COUNT/256, 1, 1);

        vkb::ComputeShaderHandler::computeBarrier(computeCommandBuffer, instancedSpheres.getBuffer());

        calculateForcesComputeSystem.bindAndDispatch(computeCommandBuffer,
                                                     &computeDescriptorSet,
                                                     INSTANCE_COUNT/256, 1, 1);


        // Add memory barrier to ensure that the computer shader has finished writing to the buffer
        vkb::ComputeShaderHandler::computeBarrier(computeCommandBuffer, instancedSpheres.getBuffer());


        integrateComputeSystem.bindAndDispatch(computeCommandBuffer,
                                                   &computeDescriptorSet,
                                                   INSTANCE_COUNT/256, 1, 1);

    });

    if (activateTimer) {
        auto time = std::chrono::high_resolution_clock::now();
        computeTime = std::chrono::duration<float, std::chrono::milliseconds::period>(time - currentTime).count();
        currentTime = time;
    }

    renderer.runFrame([this](VkCommandBuffer commandBuffer){
        showImGui();

        renderer.runRenderPass([this](VkCommandBuffer& commandBuffer){

            defaultSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);
            plane.render(defaultSystem, commandBuffer);

            instanceSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);
            instancedSpheres.render(instanceSystem, commandBuffer);

        });
    }, computeHandler.currentSemaphore(renderer.currentFrame()), vkb::ComputeShaderHandler::waitStages());
//    });
    if (activateTimer) drawTime = std::chrono::duration<float, std::chrono::milliseconds::period>(std::chrono::high_resolution_clock::now() - currentTime).count();
}


void SPHGPU3DSim::updateBuffers(uint32_t frameIndex, float deltaTime){
    camera.setPerspectiveProjection(glm::radians(50.f), renderer.getSwapChainAspectRatio(), 0.1f, 1000.f);
    gUbo.view = camera.getView();
    gUbo.proj = camera.getProjection();
    graphicsUniformBuffers[frameIndex]->write(&gUbo);

    cUbo.deltaTime = deltaTime;
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


    if (ImGui::Button("Reset")) onCreate();

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                ImGui::GetIO().Framerate);
    ImGui::End();

}


