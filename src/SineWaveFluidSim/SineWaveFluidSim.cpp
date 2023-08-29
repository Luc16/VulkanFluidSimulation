//
// Created by luc on 13/12/22.
//

#include "SineWaveFluidSim.h"
#include <execution>

void SineWaveFluidSim::onCreate() {
    initializeObjects();
    createUniformBuffers();

    // Default render system
    auto defaultDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
            .build();
    defaultDescriptorSets = createDescriptorSets(defaultDescriptorLayout,{uniformBuffers[0]->descriptorInfo()});
    {
        defaultSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), sizeof(vkb::DrawableObject::PushConstantData));
        defaultSystem.createPipeline(renderer.renderPass(), shaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& info) {
            info.bindingDescription.clear();
            info.bindingDescription.push_back({0, sizeof(glm::vec3), VK_VERTEX_INPUT_RATE_VERTEX});
            info.attributeDescription.clear();
            info.attributeDescription.push_back({0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0});
        });
    }
}

void SineWaveFluidSim::initializeObjects() {
    camera.m_translation = {-59.5092, 56.8393, -3.12228};
    camera.m_rotation = {0.35098, 1.10274, 3.14159};

    createGrid();
}

void SineWaveFluidSim::createGrid() {
    vkDeviceWaitIdle(device.device());

    plane.setScale(scale);

    float fsize = scale/float(vertexPerSide);

    vertices.resize((vertexPerSide+1) * (vertexPerSide+1));
    glm::vec3 accPos = {0.0f, 0.0f, 0.0f};
    for (uint32_t i = 0; i < vertices.size(); i++) {
        vertices[i] = accPos;
        accPos.x += fsize;

        if (accPos.x > scale) {
            accPos.x = 0.0f;
            accPos.z += fsize;
        }

    }

    std::array<uint32_t, 6> defIndices{0, vertexPerSide+1, 1, 1, vertexPerSide+1, vertexPerSide+2};
    indices.resize(vertexPerSide * vertexPerSide * defIndices.size());

    for (uint32_t i = 0, idx = 0; i < vertexPerSide*vertexPerSide; ++i, ++idx){
        if (idx % (vertexPerSide + 1) == vertexPerSide) idx++;
        for (uint32_t j = 0; j < defIndices.size(); ++j){
            indices[defIndices.size()*i + j] = idx + defIndices[j];
        }
    }

    vertexBuffer = std::make_unique<vkb::Buffer>(device, vertices.size() * sizeof(glm::vec3),
                                                      VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                                      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    vkb::Buffer::writeVectorToBuffer(device, vertexBuffer, vertices);

    indexBuffer = std::make_unique<vkb::Buffer>(device, indices.size() * sizeof(uint32_t),
                                                 VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    vkb::Buffer::writeVectorToBuffer(device, indexBuffer, indices);

}

void SineWaveFluidSim::createUniformBuffers() {
    VkDeviceSize bufferSize = sizeof(UniformBufferObject);
    uniformBuffers.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (size_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; ++i) {
        uniformBuffers[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        uniformBuffers[i]->map();
    }
}

void SineWaveFluidSim::mainLoop(float deltaTime) {
    auto currentTime = std::chrono::high_resolution_clock::now();

    cameraController.moveCamera(window.window(), deltaTime, camera);
    updateUniformBuffer(renderer.currentFrame(), deltaTime);

    if (glfwGetKey(window.window(), GLFW_KEY_SPACE) == GLFW_PRESS){
        std::cout << "{" << camera.m_translation.x << ", " << camera.m_translation.y << ", " << camera.m_translation.z << "}\n";
        std::cout << "{" << camera.m_rotation.x << ", " << camera.m_rotation.y << ", " << camera.m_rotation.z << "}\n";
    }

    if (activateTimer) {
        auto time = std::chrono::high_resolution_clock::now();
        cpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(time - currentTime).count();
        currentTime = time;
    }

    renderer.runFrame([this](VkCommandBuffer commandBuffer){
        showImGui();

        renderer.runRenderPass([this](VkCommandBuffer& commandBuffer){

            defaultSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);
            renderGrid(commandBuffer);

        });
    });
    if (activateTimer) gpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(std::chrono::high_resolution_clock::now() - currentTime).count();
}

void SineWaveFluidSim::renderGrid(VkCommandBuffer commandBuffer) {
    VkBuffer vbPos = vertexBuffer->getBuffer();
    VkDeviceSize offsets[] = {0};
    vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vbPos, offsets);
    vkCmdBindIndexBuffer(commandBuffer, indexBuffer->getBuffer(), 0, VK_INDEX_TYPE_UINT32);
    vkCmdDrawIndexed(commandBuffer, indices.size(), 1, 0, 0, 0);
}

void SineWaveFluidSim::updateUniformBuffer(uint32_t frameIndex, float deltaTime){

    camera.setPerspectiveProjection(glm::radians(50.f), renderer.getSwapChainAspectRatio(), 0.1f, 1000.f);
    ubo.viewProj = camera.getProjection()*camera.getView();
    ubo.time += deltaTime;
    uniformBuffers[frameIndex]->write(&ubo);

}

void SineWaveFluidSim::showImGui(){

    {

        ImGui::Begin("Control Panel");

        ImGui::Text("Rendering %zu vertices", vertices.size());
        ImGui::Checkbox("Display time", &activateTimer);
        if(activateTimer){
            ImGui::Text("Gpu time: %f ms", gpuTime);
            ImGui::Text("Cpu time: %f ms", cpuTime);
        }

        ImGui::DragFloat("omega", &ubo.omega, 0.01f);
        ImGui::DragFloat("phi", &ubo.phi, 0.2f);
        ImGui::DragFloat("amp", &ubo.amp, 0.02f);
        int temp = int(ubo.numSines);
        ImGui::DragInt("numSines", &temp, 1);
        ubo.numSines = temp;

        ImGui::Text("Using %s", device.getPhysicalDeviceName().c_str());
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                    ImGui::GetIO().Framerate);
        ImGui::End();
    }


}

void SineWaveFluidSim::compileShaders() {
    for (auto& shaderName : shaders) {
        std::string command{"glslc "};
        command += SHADER_DIR;
        command += shaderName;
        command += " --target-env=vulkan1.1 ";
        command += " -o ";
        command += SHADER_DIR;
        command += shaderName;
        command += ".spv";
        int status = system(command.c_str());
        if (status != 0) {
            throw std::runtime_error("Error compiling shader " + shaderName);
        }
    }
}