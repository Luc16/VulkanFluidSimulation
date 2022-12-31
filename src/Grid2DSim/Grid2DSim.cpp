//
// Created by luc on 30/12/22.
//

#include "Grid2DSim.h"
#include <execution>

void Grid2DSim::onCreate() {
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
            info.bindingDescription.push_back(grid.getBindingDescription());
            info.attributeDescription.push_back({4, 1, VK_FORMAT_R32G32B32_SFLOAT, offsetof(InstanceData, position)});
            info.attributeDescription.push_back({5, 1, VK_FORMAT_R32G32B32_SFLOAT, offsetof(InstanceData, color)});
            info.attributeDescription.push_back({6, 1, VK_FORMAT_R32_SFLOAT, offsetof(InstanceData, scale)});
        });
    }
}

void Grid2DSim::initializeObjects() {
    INSTANCE_COUNT = (window.width()/SIZE)*(window.height()/SIZE);
    camera.setViewTarget({0.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 0.0f }, {0.0f, 1.0f, 0.0f});

    createInstances();
}

void Grid2DSim::createInstances() {
    vkDeviceWaitIdle(device.device());

    grid.resizeBuffer(INSTANCE_COUNT);
    iter.resize(INSTANCE_COUNT);

    auto size = (float) SIZE;
    auto accPos = glm::vec3(0.0f, 0.0f, 0.0f);
    auto screenExtent = window.extent();

    for (uint32_t i = 0; i < grid.size(); i++) {
        auto& tile = grid[i];
        tile.color = glm::vec3(
                0.5f + randomDouble(0.0f, 0.5f),
                0.5f + randomDouble(0.0f, 0.5f),
                0.5f + randomDouble(0.0f, 0.5f)
        );
        tile.scale = size;
        tile.position = accPos;
        accPos.x += size;

        iter[i] = i;
        if (accPos.x + size > (float) screenExtent.width) {
            accPos.y += size;
            accPos.x = 0.0f;
        }
    }
    grid.updateBuffer();
}

void Grid2DSim::createUniformBuffers() {
    VkDeviceSize bufferSize = sizeof(UniformBufferObject);
    uniformBuffers.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (size_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; ++i) {
        uniformBuffers[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    }
}

void Grid2DSim::mainLoop(float deltaTime) {
    auto currentTime = std::chrono::high_resolution_clock::now();

    cameraController.moveCamera(window.window(), deltaTime, camera);
    updateUniformBuffer(renderer.currentFrame(), deltaTime);

    grid.updateBuffer();
    if (activateTimer) {
        auto time = std::chrono::high_resolution_clock::now();
        cpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(time - currentTime).count();
        currentTime = time;
    }

    renderer.runFrame([this](VkCommandBuffer commandBuffer){
        showImGui();

        renderer.runRenderPass([this](VkCommandBuffer& commandBuffer){

//            defaultSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);

            instanceSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);
            grid.render(instanceSystem, commandBuffer);
        });
    });
    if (activateTimer) gpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(std::chrono::high_resolution_clock::now() - currentTime).count();
}

void Grid2DSim::onResize(int width, int height) {
    INSTANCE_COUNT = (width/SIZE)*(height/SIZE);
    createInstances();
}

void Grid2DSim::updateUniformBuffer(uint32_t frameIndex, float deltaTime){

    UniformBufferObject ubo{};
    auto extent = window.extent();
    camera.setOrthographicProjection(0.0f, (float) extent.width, (float) extent.height, 0.0f, 0.1f, 1000.f);
    ubo.view = camera.getView();
    ubo.proj = camera.getProjection();
    uniformBuffers[frameIndex]->singleWrite(&ubo);

}

void Grid2DSim::showImGui(){

    {

        ImGui::Begin("Control Panel");

        ImGui::Text("Rendering %d instances", INSTANCE_COUNT);
        ImGui::Checkbox("Display time", &activateTimer);
        if(activateTimer){
            ImGui::Text("Gpu time: %f ms", gpuTime);
            ImGui::Text("Cpu time: %f ms", cpuTime);
        }

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                    ImGui::GetIO().Framerate);
        ImGui::End();
    }


}