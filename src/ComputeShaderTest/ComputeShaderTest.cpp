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

VkDescriptorSet ComputeShaderTest::createSingleDescriptorSet(vkb::DescriptorSetLayout &layout, std::vector<VkDescriptorBufferInfo> bufferInfos) {
    VkDescriptorSetLayout setLayout = layout.descriptorSetLayout();
    VkDescriptorSetAllocateInfo allocInfo{};
    allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
    allocInfo.descriptorPool = globalDescriptorPool->descriptorPool();
    allocInfo.descriptorSetCount = 1;
    allocInfo.pSetLayouts = &setLayout;

    VkDescriptorSet set;
    if (vkAllocateDescriptorSets(device.device(), &allocInfo, &set) != VK_SUCCESS) {
        throw std::runtime_error("failed to allocate descriptor sets!");
    }

    auto writer = vkb::DescriptorWriter(layout, *globalDescriptorPool);
    for (uint32_t i = 0; i < bufferInfos.size(); i++){
        writer.writeBuffer(i, &bufferInfos[i]);
    }

    writer.build(set, false);

    return set;
}

void ComputeShaderTest::testComputeShader() {
    auto scanDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .addBinding({2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .build();

    scanComputeSystem.createPipelineLayout(scanDescriptorLayout.descriptorSetLayout());
    scanComputeSystem.createPipeline(scanShaderPath);

    auto scanAddDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .addBinding({2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .build();

    scanAddComputeSystem.createPipelineLayout(scanAddDescriptorLayout.descriptorSetLayout());
    scanAddComputeSystem.createPipeline(scanAddShaderPath);


    uint32_t workGroupSize = 256;
    struct UboData {
        uint32_t size = 50000;
    };

    UboData ubo{};
    uint32_t numShaderCalls = ubo.size/workGroupSize + 1 - (ubo.size%workGroupSize == 0);

    vkb::Buffer uBuffer(device, sizeof(ubo), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                             VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    uBuffer.map();
    uBuffer.write(&ubo);

    VkDeviceSize bufferSize = ubo.size*sizeof(uint32_t);

    std::vector<uint32_t> data{};
    data.resize(ubo.size);

    uint32_t diff = 2;
    for (uint32_t &it : data){
        it = diff;
    }

    for (uint32_t i = 0; i < data.size(); i++){
        data[i] = i;
    }
//    data[0] = 4;

    std::cout << "Grid:\n";
    for (uint32_t i = 0; i < 32; i++){
        std::cout << data[i] << ", ";
    }
    std::cout << "\n";

    vkb::Buffer stagingBuffer(device, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

    stagingBuffer.singleWrite(data.data());
    vkb::Buffer inBuffer(device, bufferSize,
                         VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    device.copyBuffer(stagingBuffer.getBuffer(), inBuffer.getBuffer(), bufferSize);

    std::vector<std::unique_ptr<vkb::Buffer>> partialSums;

    for (uint32_t n = ubo.size; n > 1; n = (n + workGroupSize) / workGroupSize) {
        uint32_t next = (n + workGroupSize) / workGroupSize;
        partialSums.emplace_back(std::make_unique<vkb::Buffer>(device, next * sizeof(uint32_t),
                                                               VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                                               VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT));
    }

    std::vector<VkDescriptorSet> scanSets = {
            createSingleDescriptorSet(scanDescriptorLayout, {
                    uBuffer.descriptorInfo(),
                    inBuffer.descriptorInfo(),
                    partialSums[0]->descriptorInfo()
            })
    };
    for (uint32_t i = 1; i < partialSums.size(); i++){
        scanSets.emplace_back(createSingleDescriptorSet(scanDescriptorLayout, {
                uBuffer.descriptorInfo(),
                partialSums[i - 1]->descriptorInfo(),
                partialSums[i]->descriptorInfo()
        }));
    }
//
//    VkDescriptorSet scanSet = createSingleDescriptorSet(scanDescriptorLayout, {
//            uBuffer.descriptorInfo(),
//            inBuffer.descriptorInfo(),
//            partialSums[0]->descriptorInfo()
//    });
//
//    VkDescriptorSet scanAddSet = createSingleDescriptorSet(scanAddDescriptorLayout, {
//            uBuffer.descriptorInfo(),
//            inBuffer.descriptorInfo(),
//            partialSums[0]->descriptorInfo(),
//    });


    computeHandler.runComputeIsolated(0, [this, &scanSets, &inBuffer, &partialSums, numShaderCalls](VkCommandBuffer computeCommandBuffer){
        for (int i = 0; i < partialSums.size(); i++){
            scanComputeSystem.bindAndDispatch(computeCommandBuffer,
                                              &scanSets[i],
                                              numShaderCalls, 1, 1);
            std::pair<VkBuffer, VkDeviceSize> b1 = (i == 0) ? std::make_pair(inBuffer.getBuffer(), inBuffer.getSize()) :
                    std::make_pair(partialSums[i-1]->getBuffer(), partialSums[i-1]->getSize());
            std::pair<VkBuffer, VkDeviceSize> b2 = std::make_pair(partialSums[i]->getBuffer(), partialSums[i]->getSize());

            vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, {b1, b2});

        }

        for (int i = int(partialSums.size()) - 2; i >= 0; i--){
            scanAddComputeSystem.bindAndDispatch(computeCommandBuffer,
                                                 &scanSets[i],
                                                 numShaderCalls, 1, 1);

            std::pair<VkBuffer, VkDeviceSize> b1 = (i == 0) ? std::make_pair(inBuffer.getBuffer(), inBuffer.getSize()) :
                                                   std::make_pair(partialSums[i-1]->getBuffer(), partialSums[i-1]->getSize());
            std::pair<VkBuffer, VkDeviceSize> b2 = std::make_pair(partialSums[i]->getBuffer(), partialSums[i]->getSize());

            vkb::ComputeShaderHandler::computeBarriers(computeCommandBuffer, {b1, b2});

        }


    });

    device.copyBuffer(inBuffer.getBuffer(), stagingBuffer.getBuffer(), bufferSize);

    stagingBuffer.singleRead(data.data());

    std::cout << "New Grid:\n";
    for (uint32_t i = 0; i < 1000; i++){
        std::cout << data[i] << ", ";
    }
    std::cout << "\n";

    bool error = false;
    for (uint32_t i = 1; i < data.size(); i++){
        if (data[i] - data[i-1] != i){// diff) {
            std::cout << "ERROR at " << i << " !!!!!!\n";
            error = true;
        }
    }
    if (!error){
        std::cout << "All good :)\n";
    }

    uBuffer.unmap();
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