//
// Created by luc on 11/03/23.
//

#include "SPHGPU2DSim.h"

void SPHGPU2DSim::onCreate() {
    initializeObjects();
    createUniformBuffers();

    // Default render system
    auto defaultDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
            .build();
    defaultDescriptorSets = createDescriptorSets(defaultDescriptorLayout,{graphicsUniformBuffers[0]->descriptorInfo()});
    defaultSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), 0);
    defaultSystem.createPipeline(renderer.renderPass(), shaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& configInfo){
        configInfo.colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        configInfo.inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;

        configInfo.attributeDescription = Particle::getAttributeDescriptions();
        configInfo.bindingDescription = {Particle::getBindingDescription()};
        configInfo.enableAlphaBlending();
    });



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

void SPHGPU2DSim::createComputeDescriptorSets(vkb::DescriptorSetLayout &layout) {
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

    auto storageBufferInfo = particleBuffer->descriptorInfo();
    writer.writeBuffer(1, &storageBufferInfo);

    writer.build(computeDescriptorSet, false);


}

void SPHGPU2DSim::initializeObjects() {
    camera.setViewTarget({0.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 0.0f }, {0.0f, 1.0f, 0.0f});
    camera.updateView();
    auto extent = window.extent();
    camera.setOrthographicProjection(0.0f, (float) extent.width, (float) extent.height, 0.0f, 0.1f, 1000.f);
    gUbo.view = camera.getView();
    gUbo.proj = camera.getProjection();
    gUbo.radius = cUbo.H/1.5f;

    vkDeviceWaitIdle(device.device());

    auto accPos = glm::vec3(3*float(window.width())/8, 100, 0);
    std::vector<Particle> particles{PARTICLE_COUNT};

    for (uint32_t i = 0; i < PARTICLE_COUNT; i++) {
        auto& particle = particles[i];
        particle.position = accPos;
        particle.velocity = glm::vec3(0.0f);
        particle.color = glm::vec4(0.2f, 0.6f, 1.0f, 1.0f);
        particle.pressure = -cUbo.GAS_CONST * cUbo.REST_DENS;

        accPos.x += cUbo.H*0.95f;

        if (accPos.x > (float) 5*float(window.width())/8) {
            accPos.y += cUbo.H*0.95f;
            accPos.x = 3*float(window.width())/8;
        }

    }

    VkDeviceSize bufferSize = PARTICLE_COUNT * sizeof(Particle);
    vkb::Buffer stagingBuffer(device, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    stagingBuffer.singleWrite(particles.data());


    particleBuffer = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
                                                                       VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    device.copyBuffer(stagingBuffer.getBuffer(), particleBuffer->getBuffer(), bufferSize);


}

void SPHGPU2DSim::createUniformBuffers() {
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

void SPHGPU2DSim::mainLoop(float deltaTime) {
    auto currentTime = std::chrono::high_resolution_clock::now();

    updateBuffers(renderer.currentFrame(), deltaTime);

    if (activateTimer) {
        auto time = std::chrono::high_resolution_clock::now();
        cpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(time - currentTime).count();
        currentTime = time;
    }

    computeHandler.runCompute(renderer.currentFrame(), [this](VkCommandBuffer computeCommandBuffer){
        calculateDensityPressureComputeSystem.bindAndDispatch(computeCommandBuffer,
                                                     &computeDescriptorSet,
                                                     PARTICLE_COUNT/32, PARTICLE_COUNT/32, 1);

        vkb::ComputeShaderHandler::computeBarrier(computeCommandBuffer, particleBuffer);

        calculateForcesComputeSystem.bindAndDispatch(computeCommandBuffer,
                                                     &computeDescriptorSet,
                                                     PARTICLE_COUNT/32, PARTICLE_COUNT/32, 1);


        // Add memory barrier to ensure that the computer shader has finished writing to the buffer
        vkb::ComputeShaderHandler::computeBarrier(computeCommandBuffer, particleBuffer);


        integrateComputeSystem.bindAndDispatch(computeCommandBuffer,
                                                   &computeDescriptorSet,
                                                   PARTICLE_COUNT/256, 1, 1);

    });

    renderer.runFrame([this](VkCommandBuffer commandBuffer){
        showImGui();

        renderer.runRenderPass([this](VkCommandBuffer& commandBuffer){

            defaultSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);

            VkBuffer vb = particleBuffer->getBuffer();
            VkDeviceSize offsets[] = {0};
            vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
            vkCmdDraw(commandBuffer, PARTICLE_COUNT, 1, 0, 0);

        });
    }, computeHandler.currentSemaphore(renderer.currentFrame()), vkb::ComputeShaderHandler::waitStages());
    if (activateTimer) gpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(std::chrono::high_resolution_clock::now() - currentTime).count();
}


void SPHGPU2DSim::updateBuffers(uint32_t frameIndex, float deltaTime){
    cUbo.deltaTime = deltaTime;
    computeUniformBuffer->write(&cUbo);
    graphicsUniformBuffers[frameIndex]->write(&gUbo);
}

void SPHGPU2DSim::showImGui(){
    ImGui::Begin("Control Panel");

    ImGui::Text("Rendering %d instances", PARTICLE_COUNT);
    ImGui::Checkbox("Display time", &activateTimer);
    if(activateTimer){
        ImGui::Text("Gpu time: %f ms", gpuTime);
        ImGui::Text("Cpu time: %f ms", cpuTime);
    }


    if (ImGui::Button("Reset")) onCreate();

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                ImGui::GetIO().Framerate);
    ImGui::End();

}


