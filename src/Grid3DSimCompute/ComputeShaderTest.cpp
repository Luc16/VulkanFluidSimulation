//
// Created by luc on 13/12/22.
//

#include "ComputeShaderTest.h"

void ComputeShaderTest::onCreate() {
    testCompute();
    initializeObjects();
    createUniformBuffers();

    // Default render system
    auto defaultDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
            .build();
    defaultDescriptorSets = createDescriptorSets(defaultDescriptorLayout,{uniformBuffers[0]->descriptorInfo()});
    {
        defaultSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), sizeof(vkb::DrawableObject::PushConstantData));
        defaultSystem.createPipeline(renderer.renderPass(), shaderPaths);
    }


    {
        instanceSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), 0);
        instanceSystem.createPipeline(renderer.renderPass(), instanceShaderPaths, [this](vkb::Pipeline::PipelineConfigInfo& info) {
            info.bindingDescription.push_back(instancedSpheres.getBindingDescription());
            info.attributeDescription.push_back({4, 1, VK_FORMAT_R32G32B32_SFLOAT, offsetof(InstanceData, position)});
            info.attributeDescription.push_back({5, 1, VK_FORMAT_R32G32B32_SFLOAT, offsetof(InstanceData, color)});
            info.attributeDescription.push_back({6, 1, VK_FORMAT_R32_SFLOAT, offsetof(InstanceData, scale)});
        });
    }
}

void ComputeShaderTest::testCompute() {
    static constexpr uint32_t BUFFER_ELEMENTS = 32;
    std::vector<uint32_t> computeInput(BUFFER_ELEMENTS);
    std::vector<uint32_t> computeOutput(BUFFER_ELEMENTS);

    // Fill input data
    uint32_t n = 0;
    std::generate(computeInput.begin(), computeInput.end(), [&n] { return n++; });

    // populate buffer
    VkDeviceSize bufferSize = sizeof(computeInput[0]) * computeInput.size();

    vkb::Buffer stagingBuffer(device, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    stagingBuffer.singleWrite(computeInput.data());

    stagingBuffer.map();
    VkMappedMemoryRange mappedRange = {
            VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE,
            nullptr,
            stagingBuffer.getMemory(),
            0, VK_WHOLE_SIZE
    };
    vkFlushMappedMemoryRanges(device.device(), 1, &mappedRange);
    stagingBuffer.unmap();

    vkb::Buffer inBuffer(device, bufferSize,
                         VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    device.copyBuffer(stagingBuffer.getBuffer(), inBuffer.getBuffer(), bufferSize);



    // prepare pipeline
    auto computeDescriptorPool = vkb::DescriptorPool::Builder(device)
            .addPoolSize({VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1})
            .build();
    auto computeDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .build();


    VkPipelineLayout pipelineLayout;
    VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo {};
    pipelineLayoutCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
    pipelineLayoutCreateInfo.setLayoutCount = 1;
    auto setLayout = computeDescriptorLayout.descriptorSetLayout();
    pipelineLayoutCreateInfo.pSetLayouts = &setLayout;
    if (vkCreatePipelineLayout(device.device(), &pipelineLayoutCreateInfo, nullptr, &pipelineLayout) != VK_SUCCESS)
        throw std::runtime_error("failed to create compute pipeline layout!");


    VkDescriptorSet set;
    auto writer = vkb::DescriptorWriter(computeDescriptorLayout, *globalDescriptorPool);
    VkDescriptorBufferInfo bufferInfo{inBuffer.getBuffer(), 0, VK_WHOLE_SIZE};
    writer.writeBuffer(0, &bufferInfo);
    writer.build(set);


    VkPipelineCache pipelineCache;
    VkPipelineCacheCreateInfo pipelineCacheCreateInfo = {};
    pipelineCacheCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO;
    if (vkCreatePipelineCache(device.device(), &pipelineCacheCreateInfo, nullptr, &pipelineCache) != VK_SUCCESS)
        throw std::runtime_error("failed to create compute pipeline cache!");


    VkPipeline computePipeline;
    VkComputePipelineCreateInfo computePipelineCreateInfo{};
    computePipelineCreateInfo.sType = VK_STRUCTURE_TYPE_COMPUTE_PIPELINE_CREATE_INFO;
    computePipelineCreateInfo.layout = pipelineLayout;
    computePipelineCreateInfo.flags = 0;

    struct SpecializationData {
        uint32_t BUFFER_ELEMENT_COUNT = BUFFER_ELEMENTS;
    } specializationData;
    VkSpecializationMapEntry specializationMapEntry = {0, 0, sizeof(uint32_t)};
    VkSpecializationInfo specializationInfo = {1, &specializationMapEntry, sizeof(SpecializationData), &specializationData};

    auto compShaderCode = vkb::Pipeline::readFile("../src/Grid3DSimCompute/Shaders/test.comp.spv");
    VkShaderModuleCreateInfo createInfo{};
    createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
    createInfo.codeSize = compShaderCode.size();
    createInfo.pCode = reinterpret_cast<const uint32_t*>(compShaderCode.data());

    VkShaderModule compShaderModule;
    if (vkCreateShaderModule(device.device(), &createInfo, nullptr, &compShaderModule) != VK_SUCCESS){
        throw std::runtime_error("failed to create shader module!");
    }

    VkPipelineShaderStageCreateInfo shaderStage{};
    shaderStage.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shaderStage.stage = VK_SHADER_STAGE_COMPUTE_BIT;
    shaderStage.module = compShaderModule;
    shaderStage.pName = "main";
    shaderStage.pSpecializationInfo = &specializationInfo;

    computePipelineCreateInfo.stage = shaderStage;
    if (vkCreateComputePipelines(device.device(), pipelineCache, 1, &computePipelineCreateInfo, nullptr, &computePipeline) != VK_SUCCESS)
        throw std::runtime_error("failed to create compute pipeline!");


    // submit work

    VkCommandBuffer commandBuffer;
    VkFence fence;
    // Create a command buffer for compute operations
    VkCommandBufferAllocateInfo cmdBufAllocateInfo = {
            VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO,
            nullptr,
            device.graphicsCommandPool(),
            VK_COMMAND_BUFFER_LEVEL_PRIMARY,
            1
    };
    vkAllocateCommandBuffers(device.device(), &cmdBufAllocateInfo, &commandBuffer);

    // Fence for compute CB sync
    VkFenceCreateInfo fenceCreateInfo = {VK_STRUCTURE_TYPE_FENCE_CREATE_INFO, nullptr, VK_FENCE_CREATE_SIGNALED_BIT};
    vkCreateFence(device.device(), &fenceCreateInfo, nullptr, &fence);

    VkCommandBufferBeginInfo beginInfo{};
    beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;

    vkBeginCommandBuffer(commandBuffer, &beginInfo);


    VkBufferMemoryBarrier bufferBarrier {};
    bufferBarrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
    bufferBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    bufferBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    bufferBarrier.buffer = inBuffer.getBuffer();
    bufferBarrier.size = VK_WHOLE_SIZE;
    bufferBarrier.srcAccessMask = VK_ACCESS_HOST_WRITE_BIT;
    bufferBarrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
    bufferBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    bufferBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

    vkCmdPipelineBarrier(
            commandBuffer,
            VK_PIPELINE_STAGE_HOST_BIT,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            0,
            0, nullptr,
            1, &bufferBarrier,
            0, nullptr);

    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, computePipeline);
    vkCmdBindDescriptorSets(commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, pipelineLayout, 0, 1, &set, 0, 0);

    vkCmdDispatch(commandBuffer, BUFFER_ELEMENTS, 1, 1);

    bufferBarrier.srcAccessMask = VK_ACCESS_SHADER_WRITE_BIT;
    bufferBarrier.dstAccessMask = VK_ACCESS_TRANSFER_READ_BIT;
    bufferBarrier.buffer = inBuffer.getBuffer();
    bufferBarrier.size = VK_WHOLE_SIZE;
    bufferBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    bufferBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

    vkCmdPipelineBarrier(
            commandBuffer,
            VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            0,
            0, nullptr,
            1, &bufferBarrier,
            0, nullptr);

    // Read back to host visible buffer
    VkBufferCopy copyRegion = {};
    copyRegion.size = bufferSize;
    vkCmdCopyBuffer(commandBuffer, inBuffer.getBuffer(), stagingBuffer.getBuffer(), 1, &copyRegion);

    // Barrier to ensure that buffer copy is finished before host reading from it
    bufferBarrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    bufferBarrier.dstAccessMask = VK_ACCESS_HOST_READ_BIT;
    bufferBarrier.buffer = stagingBuffer.getBuffer();
    bufferBarrier.size = VK_WHOLE_SIZE;
    bufferBarrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    bufferBarrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;

    vkCmdPipelineBarrier(
            commandBuffer,
            VK_PIPELINE_STAGE_TRANSFER_BIT,
            VK_PIPELINE_STAGE_HOST_BIT,
            0,
            0, nullptr,
            1, &bufferBarrier,
            0, nullptr);

    vkEndCommandBuffer(commandBuffer);

    // Submit compute work
    vkResetFences(device.device(), 1, &fence);
    const VkPipelineStageFlags waitStageMask = VK_PIPELINE_STAGE_TRANSFER_BIT;
    VkSubmitInfo computeSubmitInfo {};
    computeSubmitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    computeSubmitInfo.pWaitDstStageMask = &waitStageMask;
    computeSubmitInfo.commandBufferCount = 1;
    computeSubmitInfo.pCommandBuffers = &commandBuffer;
    vkQueueSubmit(device.graphicsQueue(), 1, &computeSubmitInfo, fence);
    vkWaitForFences(device.device(), 1, &fence, VK_TRUE, UINT64_MAX);

    // Make device writes visible to the host
    void *mapped;
    vkMapMemory(device.device(), stagingBuffer.getMemory(), 0, VK_WHOLE_SIZE, 0, &mapped);
    VkMappedMemoryRange mappedRange2 {};
    mappedRange2.sType = VK_STRUCTURE_TYPE_MAPPED_MEMORY_RANGE;
    mappedRange2.memory = stagingBuffer.getMemory();
    mappedRange2.offset = 0;
    mappedRange2.size = VK_WHOLE_SIZE;
    vkInvalidateMappedMemoryRanges(device.device(), 1, &mappedRange2);

    // Copy to output
    memcpy(computeOutput.data(), mapped, bufferSize);
    vkUnmapMemory(device.device(), stagingBuffer.getMemory());

    vkQueueWaitIdle(device.graphicsQueue());

    // Output buffer contents
    printf("Compute input:\n");
    for (uint32_t i = 0; i < computeInput.size(); ++i) {
        printf("%d -> %d \n", computeInput[i], computeOutput[i]);
    }
    std::cout << std::endl;


    vkDestroyFence(device.device(), fence, nullptr);
    vkDestroyPipelineCache(device.device(), pipelineCache, nullptr);
    vkDestroyPipelineLayout(device.device(), pipelineLayout, nullptr);
    vkDestroyShaderModule(device.device(), compShaderModule, nullptr);
    vkDestroyPipeline(device.device(), computePipeline, nullptr);
}

void ComputeShaderTest::initializeObjects() {
    camera.setViewTarget({0.0f, 10.0f, 10.0f}, {12.0f, -1.0f, -12.0f }, {0.0f, 1.0f, 0.0f});
    camera.m_rotation = {0, glm::radians(180.0f), glm::radians(180.0f)};

    createInstances();
}

void ComputeShaderTest::createInstances() {
    vkDeviceWaitIdle(device.device());

    instancedSpheres.resizeBuffer(INSTANCE_COUNT);
    sphereSpeeds.resize(INSTANCE_COUNT);
    iter.resize(INSTANCE_COUNT);

    auto planeScale = sqrtf((float) INSTANCE_COUNT);
    plane.m_translation = {1.5*planeScale/2, -1.0f, -1.5*planeScale/2};
    plane.setScale(planeScale);

    auto accPos = glm::vec3(0.0f, 0.0f, 0.0f);
    auto spherePerLine = (int) planeScale;

    for (uint32_t i = 0; i < instancedSpheres.size(); i++) {
        auto& sphere = instancedSpheres[i];
        sphere.color = glm::vec3(
                0.2f + randomDouble(0.0f, 0.8f),
                0.2f + randomDouble(0.0f, 0.8f),
                0.2f + randomDouble(0.0f, 0.8f)
        );
        if (i == instancedSpheres.size() - 1) sphere.color = {1, 0, 0};
        sphere.scale = 1.0f;
        sphere.position = accPos + glm::vec3(0.0f, randomDouble(1.0f, 3.0f), 0.0f);
        accPos.x += 1.5f;

        sphereSpeeds[i] = 0.0f;
        iter[i] = i;
        if (i % spherePerLine == spherePerLine - 1) {
            accPos.z -= 1.5f;
            accPos.x = 0.0f;
        }
    }
    instancedSpheres.updateBuffer();
}

void ComputeShaderTest::createUniformBuffers() {
    VkDeviceSize bufferSize = sizeof(UniformBufferObject);
    uniformBuffers.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (size_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; ++i) {
        uniformBuffers[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    }
}

void ComputeShaderTest::mainLoop(float deltaTime) {
    auto currentTime = std::chrono::high_resolution_clock::now();

    cameraController.moveCamera(window.window(), deltaTime, camera);
    updateUniformBuffer(renderer.currentFrame(), deltaTime);

    updateSpheres(deltaTime);
    instancedSpheres.updateBuffer();
    if (activateTimer) {
        auto time = std::chrono::high_resolution_clock::now();
        cpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(time - currentTime).count();
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
    });
    if (activateTimer) gpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(std::chrono::high_resolution_clock::now() - currentTime).count();
}

void ComputeShaderTest::updateSpheres(float deltaTime){
    float a = 5;
    for (size_t i = 0; i < instancedSpheres.size(); i++) {
        auto& sphere = instancedSpheres[i];

        sphere.position.y += deltaTime * sphereSpeeds[i];
        sphereSpeeds[i] -= a*deltaTime;
        if (sphere.position.y < plane.m_translation.y) {
            sphere.position.y = plane.m_translation.y;
            sphereSpeeds[i] *= (damping - 1);
        }
    }

}

void ComputeShaderTest::updateUniformBuffer(uint32_t frameIndex, float deltaTime){

    UniformBufferObject ubo{};
    camera.setPerspectiveProjection(glm::radians(50.f), renderer.getSwapChainAspectRatio(), 0.1f, 1000.f);
    ubo.view = camera.getView();
    ubo.proj = camera.getProjection();
    uniformBuffers[frameIndex]->singleWrite(&ubo);

}

void ComputeShaderTest::showImGui(){

    {

        ImGui::Begin("Control Panel");

        ImGui::Text("Rendering %d instances", INSTANCE_COUNT);
        ImGui::Checkbox("Display time", &activateTimer);
        if(activateTimer){
            ImGui::Text("Gpu time: %f ms", gpuTime);
            ImGui::Text("Cpu time: %f ms", cpuTime);
        }

        if (ImGui::Button("Double instance count")){
            if (INSTANCE_COUNT < 8388608) INSTANCE_COUNT *= 2;
            createInstances();
        }
        if (ImGui::Button("Half instance count")){
            if (INSTANCE_COUNT > 1) INSTANCE_COUNT /= 2;
            createInstances();
        }

        if (ImGui::CollapsingHeader("Plane", ImGuiTreeNodeFlags_DefaultOpen)) {

            ImGui::SliderFloat("y", &plane.m_translation.y, -100.0f, 10.0f);
            if (ImGui::Button("Reset")) createInstances();

        }

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                    ImGui::GetIO().Framerate);
        ImGui::End();
    }

}