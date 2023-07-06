//
// Created by luc on 13/12/22.
//

#include "ComputeShaderTest.h"

void ComputeShaderTest::onCreate() {
    if (testShader) {
        testComputeShader();
        return;
    }

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

    moveParticlesComputeSystem.createPipelineLayout(computeDescriptorLayout.descriptorSetLayout());
    moveParticlesComputeSystem.createPipeline(moveParticlesShaderPath);

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

        auto uniformBufferInfo = computeUniformBuffers[i]->descriptorInfo();
        writer.writeBuffer(0, &uniformBufferInfo);

        auto storageBufferInfoLastFrame = computeData[(i - 1) % vkb::SwapChain::MAX_FRAMES_IN_FLIGHT]->descriptorInfo();
        writer.writeBuffer(1, &storageBufferInfoLastFrame);

        auto storageBufferInfoCurrentFrame = computeData[i]->descriptorInfo();
        writer.writeBuffer(2, &storageBufferInfoCurrentFrame);

        writer.build(computeDescriptorSets[i], false);
    }

}

void ComputeShaderTest::initializeObjects() {
    camera.setViewTarget({0.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 0.0f }, {0.0f, 1.0f, 0.0f});
    camera.updateView();
    auto extent = window.extent();
    camera.setOrthographicProjection(0.0f, (float) extent.width, (float) extent.height, 0.0f, 0.1f, 1000.f);
    gUbo.view = camera.getView();
    gUbo.proj = camera.getProjection();

    vkDeviceWaitIdle(device.device());

    std::vector<Particle> particles{PARTICLE_COUNT};

    auto middle = glm::vec2(float(window.width())/2,float(window.height())/2);
    for (uint32_t i = 0; i < PARTICLE_COUNT; i++) {
        auto& particle = particles[i];
//        float r = 0.4f * std::sqrt(randomFloat(0.8f, 1.0f)); (anel)
        float r = float(std::min(window.width(), window.height()))/8 * std::sqrt(randomFloat());
        float theta = randomFloat() * 2 * 3.14159265358979323846f;
        float x = r * std::cos(theta) * float(window.height()) / float(window.width());
        float y = r * std::sin(theta);
//        float x = randomFloat(-1.0f, 1.0f);
//        float y = randomFloat(-1.0f, 1.0f);
        particle.position = middle + glm::vec2(x, y);
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

void ComputeShaderTest::testComputeShader() {
    uint32_t gridSize = 1000;
    VkDeviceSize bufferSize = gridSize*sizeof(uint32_t);

    std::vector<uint32_t> data{};
    data.resize(gridSize);

    for (uint32_t &it : data){
        it = 0;
    }

    for (uint32_t i = 0; i < 10; i++){
        data[i] = i % 5;
    }
    data[0] = 4;

    std::cout << "Grid:\t\t";
    for (uint32_t i = 0; i < 16; i++){
        std::cout << data[i] << ", ";
    }
    std::cout << "\n";

    vkb::Buffer stagingBuffer(device, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

    stagingBuffer.singleWrite(data.data());
    vkb::Buffer inBuffer(device, bufferSize,
                         VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    vkb::Buffer outBuffer(device, bufferSize,
                         VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    device.copyBuffer(stagingBuffer.getBuffer(), inBuffer.getBuffer(), bufferSize);

    auto computeDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .build();

    VkDescriptorSetLayout layout = computeDescriptorLayout.descriptorSetLayout();
    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = globalDescriptorPool->descriptorPool();
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &layout;

    VkDescriptorSet set;
    if (vkAllocateDescriptorSets(device.device(), &allocInfo, &set) != VK_SUCCESS) {
        throw std::runtime_error("failed to allocate descriptor sets!");
    }

    auto writer = vkb::DescriptorWriter(computeDescriptorLayout, *globalDescriptorPool);


    auto inBuffInfo = inBuffer.descriptorInfo();
    writer.writeBuffer(0, &inBuffInfo);

    auto outBuffInfo = outBuffer.descriptorInfo();
    writer.writeBuffer(1, &outBuffInfo);

    writer.build(set, false);


    scanComputeSystem.createPipelineLayout(computeDescriptorLayout.descriptorSetLayout());
    scanComputeSystem.createPipeline(scanShaderPath);

    computeHandler.runComputeIsolated(0, [this, &set, gridSize](VkCommandBuffer computeCommandBuffer){
        scanComputeSystem.bindAndDispatch(computeCommandBuffer,
                                                     &set,
                                                     gridSize/256 + 1, 1, 1);
    });

    device.copyBuffer(outBuffer.getBuffer(), stagingBuffer.getBuffer(), bufferSize);

    stagingBuffer.singleRead(data.data());

    std::cout << "New Grid:\t";
    for (uint32_t i = 0; i < 16; i++){
        std::cout << data[i] << ", ";
    }
    std::cout << "\n";

    endProgram();
}


void ComputeShaderTest::createUniformBuffers() {
    VkDeviceSize bufferSize = sizeof(UniformBufferObject);
    computeUniformBuffers.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    graphicsUniformBuffers.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (size_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; ++i) {
        computeUniformBuffers[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        computeUniformBuffers[i]->map();

        graphicsUniformBuffers[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        graphicsUniformBuffers[i]->map();
    }
}

void ComputeShaderTest::mainLoop(float deltaTime) {
    auto currentTime = std::chrono::high_resolution_clock::now();

    updateUniformBuffers(renderer.currentFrame(), deltaTime);

    if (activateTimer) {
        auto time = std::chrono::high_resolution_clock::now();
        cpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(time - currentTime).count();
        currentTime = time;
    }

    computeHandler.runCompute(renderer.currentFrame(), [this](VkCommandBuffer computeCommandBuffer){
        calculateForcesComputeSystem.bindAndDispatch(computeCommandBuffer,
                                                     &computeDescriptorSets[(renderer.currentFrame() + 1)%vkb::SwapChain::MAX_FRAMES_IN_FLIGHT],
                                                     PARTICLE_COUNT/256, 1, 1);


        // Add memory barrier to ensure that the computer shader has finished writing to the buffer
        vkb::ComputeShaderHandler::computeBarrier(computeCommandBuffer, computeData);


        moveParticlesComputeSystem.bindAndDispatch(computeCommandBuffer,
                                                   &computeDescriptorSets[renderer.currentFrame()],
                                                   PARTICLE_COUNT/256, 1, 1);

    });



    renderer.runFrame([this](VkCommandBuffer commandBuffer){
        showImGui();

        renderer.runRenderPass([this](VkCommandBuffer& commandBuffer){

            defaultSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);

            VkBuffer vb = computeData[renderer.currentFrame()]->getBuffer();
            VkDeviceSize offsets[] = {0};
            vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
            vkCmdDraw(commandBuffer, PARTICLE_COUNT, 1, 0, 0);

        });
    }, computeHandler.currentSemaphore(renderer.currentFrame()), vkb::ComputeShaderHandler::waitStages());
    if (activateTimer) gpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(std::chrono::high_resolution_clock::now() - currentTime).count();
}

void ComputeShaderTest::updateUniformBuffers(uint32_t frameIndex, float deltaTime){
    cUbo.deltaTime = deltaTime;
    computeUniformBuffers[frameIndex]->write(&cUbo);
    graphicsUniformBuffers[frameIndex]->write(&gUbo);
}

void ComputeShaderTest::showImGui(){
    ImGui::Begin("Control Panel");

    ImGui::Text("Rendering %d instances", PARTICLE_COUNT);
    ImGui::Checkbox("Display time", &activateTimer);
    if(activateTimer){
        ImGui::Text("Gpu time: %f ms", gpuTime);
        ImGui::Text("Cpu time: %f ms", cpuTime);
    }

    ImGui::DragFloat("Gravitational Constant", &cUbo.gravitationalConstant, 10.0f, 0.0f, 10000.0f, "%.0f");

    if (ImGui::Button("Reset")) onCreate();

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                ImGui::GetIO().Framerate);
    ImGui::End();

}

void ComputeShaderTest::compileShaders() {
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