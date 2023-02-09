//
// Created by luc on 13/12/22.
//

#include "ComputeShaderTest.h"

void ComputeShaderTest::onCreate() {
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