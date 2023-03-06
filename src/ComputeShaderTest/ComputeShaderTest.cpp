//
// Created by luc on 13/12/22.
//

#include "ComputeShaderTest.h"

void ComputeShaderTest::onCreate() {
    initializeObjects();
    createUniformBuffers();

    // Default render system
    defaultSystem.createPipelineLayout(nullptr, 0);
    defaultSystem.createPipeline(renderer.renderPass(), shaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& configInfo){
        configInfo.colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        configInfo.inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;

        configInfo.attributeDescription = Particle::getAttributeDescriptions();
        configInfo.bindingDescription = {Particle::getBindingDescription()};
        configInfo.enableAlphaBlending();
    });

    calculateForcesComputeSystem.destroyPipeline();
    moveParticlesComputeSystem.destroyPipeline();
    auto computeDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .addBinding({2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .build();

    createComputeDescriptorSets(computeDescriptorLayout);
    calculateForcesComputeSystem.createPipelineLayout(computeDescriptorLayout.descriptorSetLayout());
    calculateForcesComputeSystem.createPipeline(calculateForcesShaderPath);

    moveParticlesComputeSystem.createPipelineLayout(computeDescriptorLayout.descriptorSetLayout());
    moveParticlesComputeSystem.createPipeline(moveParticlesShaderPath);


    for (uint32_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; i++) {
        computeSemaphores[i] = {calculateForcesComputeSystem.currentSemaphore(i)[0]};
    }
    computeWaitStages = {vkb::ComputeSystem::waitStages()[0]};
}

void ComputeShaderTest::createComputeDescriptorSets(vkb::DescriptorSetLayout &layout) {

    std::vector<VkDescriptorSetLayout> layouts(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT, layout.descriptorSetLayout());
    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = globalDescriptorPool->descriptorPool();
    allocInfo.descriptorSetCount = static_cast<uint32_t>(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    allocInfo.pSetLayouts = layouts.data();

    computeDescriptorSets.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    if (vkAllocateDescriptorSets(device.device(), &allocInfo, computeDescriptorSets.data()) != VK_SUCCESS) {
        throw std::runtime_error("failed to allocate descriptor sets!");
    }

    for (size_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; i++) {
        auto writer = vkb::DescriptorWriter(layout, *globalDescriptorPool);

        auto uniformBufferInfo = uniformBuffers[i]->descriptorInfo();
        writer.writeBuffer(0, &uniformBufferInfo);

        auto storageBufferInfoLastFrame = computeData[(i - 1) % vkb::SwapChain::MAX_FRAMES_IN_FLIGHT]->descriptorInfo();
        writer.writeBuffer(1, &storageBufferInfoLastFrame);

        auto storageBufferInfoCurrentFrame = computeData[i]->descriptorInfo();
        writer.writeBuffer(2, &storageBufferInfoCurrentFrame);

        writer.build(computeDescriptorSets[i], false);
    }

}

void ComputeShaderTest::initializeObjects() {
    vkDeviceWaitIdle(device.device());

    std::vector<Particle> particles(PARTICLE_COUNT);
    for (auto& particle : particles) {
        float r = 0.25f * std::sqrt(randomFloat());
        float theta = randomFloat() * 2 * 3.14159265358979323846f;
        float x = r * std::cos(theta) * float(window.height()) / float(window.width());
        float y = r * std::sin(theta);
//        float x = randomFloat(-1.0f, 1.0f);
//        float y = randomFloat(-1.0f, 1.0f);
        particle.position = glm::vec2(x, y);
        particle.velocity = glm::vec2(0.0f);
        particle.color = glm::vec4(randomFloat(), randomFloat(), randomFloat(), 1.0f);
    }

    VkDeviceSize bufferSize = PARTICLE_COUNT * sizeof(Particle);
    vkb::Buffer stagingBuffer(device, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    stagingBuffer.singleWrite(particles.data());

    computeData.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (uint32_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; i++) {
        computeData[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
                                                                                 VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        device.copyBuffer(stagingBuffer.getBuffer(), computeData[i]->getBuffer(), bufferSize);
    }

}

void ComputeShaderTest::createUniformBuffers() {
    VkDeviceSize bufferSize = sizeof(UniformBufferObject);
    uniformBuffers.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (size_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; ++i) {
        uniformBuffers[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        uniformBuffers[i]->map();
    }
}

void ComputeShaderTest::mainLoop(float deltaTime) {
    auto currentTime = std::chrono::high_resolution_clock::now();

    updateUniformBuffer(renderer.currentFrame(), deltaTime);

    if (activateTimer) {
        auto time = std::chrono::high_resolution_clock::now();
        cpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(time - currentTime).count();
        currentTime = time;
    }

    calculateForcesComputeSystem.runCompute(renderer.currentFrame(), [this](VkCommandBuffer computeCommandBuffer){
        calculateForcesComputeSystem.bind(computeCommandBuffer, &computeDescriptorSets[(renderer.currentFrame() + 1)%vkb::SwapChain::MAX_FRAMES_IN_FLIGHT]);
        calculateForcesComputeSystem.dispatch(computeCommandBuffer, PARTICLE_COUNT/32, PARTICLE_COUNT/32, 1);

        // Add memory barrier to ensure that the computer shader has finished writing to the buffer
        std::array<VkBufferMemoryBarrier, vkb::SwapChain::MAX_FRAMES_IN_FLIGHT> bufferMemoryBarriers{};
        for (uint32_t i = 0; i < computeData.size(); i++){
            bufferMemoryBarriers[i].sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
            bufferMemoryBarriers[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            bufferMemoryBarriers[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
            bufferMemoryBarriers[i].buffer = computeData[i]->getBuffer();
            bufferMemoryBarriers[i].size = computeData[i]->getSize();
            bufferMemoryBarriers[i].srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
            bufferMemoryBarriers[i].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        }

        vkCmdPipelineBarrier(
                computeCommandBuffer,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
                0,
                0, nullptr,
                bufferMemoryBarriers.size(), bufferMemoryBarriers.data(),
                0, nullptr);


        moveParticlesComputeSystem.bind(computeCommandBuffer, &computeDescriptorSets[renderer.currentFrame()]);
        moveParticlesComputeSystem.dispatch(computeCommandBuffer, PARTICLE_COUNT/256, 1, 1);

    });



    renderer.runFrame([this](VkCommandBuffer commandBuffer){
        showImGui();

        renderer.runRenderPass([this](VkCommandBuffer& commandBuffer){

            defaultSystem.bind(commandBuffer, nullptr);

            VkBuffer vb = computeData[renderer.currentFrame()]->getBuffer();
            VkDeviceSize offsets[] = {0};
            vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
            vkCmdDraw(commandBuffer, PARTICLE_COUNT, 1, 0, 0);

        });
    }, computeSemaphores[renderer.currentFrame()], computeWaitStages);
    if (activateTimer) gpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(std::chrono::high_resolution_clock::now() - currentTime).count();
}


void ComputeShaderTest::updateUniformBuffer(uint32_t frameIndex, float deltaTime){
    ubo.deltaTime = deltaTime;
    uniformBuffers[frameIndex]->write(&ubo);
}

void ComputeShaderTest::showImGui(){
    ImGui::Begin("Control Panel");

    ImGui::Checkbox("Display time", &activateTimer);
    if(activateTimer){
        ImGui::Text("Gpu time: %f ms", gpuTime);
        ImGui::Text("Cpu time: %f ms", cpuTime);
    }

    ImGui::DragFloat("Gravitational Constant", &ubo.gravitationalConstant, 0.000002f, 0.0f, 0.002f, "%.7f");

    if (ImGui::Button("Reset")) onCreate();

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                ImGui::GetIO().Framerate);
    ImGui::End();


}