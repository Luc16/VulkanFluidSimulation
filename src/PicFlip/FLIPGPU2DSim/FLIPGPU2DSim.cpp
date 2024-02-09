//
// Created by luc on 11/03/23.
//

#include "FLIPGPU2DSim.h"

void FLIPGPU2DSim::onCreate() {
    initializeObjects();
    createBuffers();
    generateGridLines();

    // Default render system
    auto defaultDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
            .build();
    defaultDescriptorSets = createDescriptorSets(defaultDescriptorLayout,{uniformBuffers[0]->descriptorInfo()});
    particleSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), 0);
    particleSystem.createPipeline(renderer.renderPass(), particleShaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& configInfo){
        configInfo.inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;

        configInfo.attributeDescription = Particle::getAttributeDescriptions();
        configInfo.bindingDescription = {Particle::getBindingDescription()};
        configInfo.enableAlphaBlending();
    });
    lineSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), 0);
    lineSystem.createPipeline(renderer.renderPass(), defaultShaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& configInfo){
        configInfo.inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;

        configInfo.attributeDescription = Point::getAttributeDescriptions();
        configInfo.bindingDescription = {Point::getBindingDescription()};
        configInfo.enableAlphaBlending();
    });

    quadSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), 0);
    quadSystem.createPipeline(renderer.renderPass(), defaultShaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& configInfo){
        configInfo.attributeDescription = Point::getAttributeDescriptions();
        configInfo.bindingDescription = {Point::getBindingDescription()};
        configInfo.enableAlphaBlending();
    });

}

void FLIPGPU2DSim::initializeObjects() {
    camera.setViewTarget({0.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 0.0f }, {0.0f, 1.0f, 0.0f});
    camera.updateView();
    auto extent = window.extent();
    camera.setOrthographicProjection(0.0f, (float) extent.width, (float) extent.height, 0.0f, 0.1f, 1000.f);
    ubo.view = camera.getView();
    ubo.proj = camera.getProjection();
    ubo.radius = 2.0f*radius;

    vkDeviceWaitIdle(device.device());

    // initialize particles
    flipSolver.initializeParticles(globalDescriptorPool);
}

void FLIPGPU2DSim::generateGridLines() {
    gridLines.reserve(numTilesX + numTilesY);
    for (uint32_t i = 0; i < numTilesX; i++) {
        auto x = float(i * SIZE);

        gridLines.push_back(Line{
                        {{x, 0.0f, 0.0f}, gridColor},
                        {{x, float(window.height()), 0.0f}, gridColor}
                });
    }

    for (uint32_t i = 0; i < numTilesY; i++) {
        auto y = float(i * SIZE);

        gridLines.push_back(Line{
                        {{0.0f, y, 0.0f}, gridColor},
                        {{float(window.width()), y, 0.0f}, gridColor}
                });
    }

    for (uint32_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; i++) {
        vkb::Buffer::writeVectorToBuffer(device, gridLinesBuffer, gridLines);
    }
}

void FLIPGPU2DSim::createBuffers() {
    particleBuffer = std::make_unique<vkb::Buffer>(device, PARTICLE_COUNT * sizeof(Particle), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
                                                                                   VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
//    vkb::Buffer::writeVectorToBuffer(device, particleBuffer, flipSolver.particles);
    gridLinesBuffer = std::make_unique<vkb::Buffer>(device, (numTilesX + numTilesY) * sizeof(Line), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
                                                                               VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);


    VkDeviceSize bufferSize = sizeof(UniformBufferObject);
    uniformBuffers.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (size_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; ++i) {
        uniformBuffers[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        uniformBuffers[i]->map();
    }
}

void FLIPGPU2DSim::mainLoop(float deltaTime) {
    auto currentTime = std::chrono::high_resolution_clock::now();

    if (!paused) flipSolver.updateSimulation(deltaTime, flipRatio);

    updateBuffers(renderer.currentFrame());

    if (activateTimer) {
        auto time = std::chrono::high_resolution_clock::now();
        cpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(time - currentTime).count();
        currentTime = time;
    }

//    renderObjects();
    endProgram();

    if (activateTimer) gpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(std::chrono::high_resolution_clock::now() - currentTime).count();
}

void FLIPGPU2DSim::renderObjects() {
    renderer.runFrame([this](VkCommandBuffer commandBuffer){
        showImGui();

        renderer.runRenderPass([this](VkCommandBuffer& commandBuffer){

            if (showParticles) {
                particleSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);
                VkBuffer vb = particleBuffer->getBuffer();
                VkDeviceSize offsets[] = {0};
                vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
                vkCmdDraw(commandBuffer, PARTICLE_COUNT, 1, 0, 0);
            }

            if (showGrid) drawGrid(commandBuffer);


        });
    });
}

void FLIPGPU2DSim::drawGrid(VkCommandBuffer commandBuffer) {
    lineSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);

    VkBuffer vb = gridLinesBuffer->getBuffer();
    VkDeviceSize offsets[] = {0};
    vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
    vkCmdDraw(commandBuffer, 2*gridLines.size(), 1, 0, 0);
}

void FLIPGPU2DSim::applyGridLineColor() {
    for (auto& line : gridLines) {
        line.setColor(gridColor);
    }
    for (uint32_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; i++) {
        vkb::Buffer::writeVectorToBuffer(device, gridLinesBuffer, gridLines);
    }
}

void FLIPGPU2DSim::updateBuffers(uint32_t frameIndex) {
    uniformBuffers[frameIndex]->write(&ubo);

//    vkb::Buffer::writeVectorToBuffer(device, particleBuffer, flipSolver.particles);
}


void FLIPGPU2DSim::showImGui(){
    static bool changeGridColorWindowOpen = false;

    ImGui::Begin("Control Panel");

    ImGui::Text("Rendering %d instances", PARTICLE_COUNT);
    ImGui::Checkbox("Display time", &activateTimer);
    if(activateTimer){
        ImGui::Text("Gpu time: %f ms", gpuTime);
        ImGui::Text("Cpu time: %f ms", cpuTime);
    }

    ImGui::Checkbox("Show Particles", &showParticles);
    ImGui::Checkbox("Show Grid", &showGrid);

    if (showGrid) {
        if (ImGui::Button("Change Grid Color")) {
            changeGridColorWindowOpen = true;
        }
    }


    ImGui::DragFloat("Flip ratio", &flipRatio, 0.001f, 0.0001f, 1.0f);

    if (ImGui::Button("Reset") || glfwIsKeyJustPressed<GLFW_KEY_R>()) initializeObjects();
    if (ImGui::Button("Pause") || glfwIsKeyJustPressed<GLFW_KEY_SPACE>()) paused = !paused;

    static bool singleStep = false;
    if (singleStep) {
        paused = true;
        singleStep = false;
    }
    if (ImGui::Button("Step") || glfwIsKeyJustPressed<GLFW_KEY_S>()) {
        singleStep = true;
        paused = false;
    }

    ImGui::Text("Using %s", device.getPhysicalDeviceName().c_str());
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                ImGui::GetIO().Framerate);
    ImGui::End();

    if (changeGridColorWindowOpen) {
        ImGui::Begin("Change Grid Color", &changeGridColorWindowOpen);
        ImGui::ColorPicker3("Grid Color", &gridColor[0]);
        applyGridLineColor();
        if (ImGui::Button("Close")) {
            changeGridColorWindowOpen = false;
        }
        ImGui::End();
    }
}

void FLIPGPU2DSim::compileShaders() {
    for (uint32_t i = 0; i < shaders.size(); i++) {
        std::string command{"glslc "};
        if (i < computeShaderStartIdx) {
            command += RENDER_SHADER_DIR;
        } else {
            command += SIMULATIONS_SHADER_DIR;
        }
        command += shaders[i];
        command += " --target-env=vulkan1.1 ";
        command += " -o ";
        command += COMPILED_SHADER_DIR;
        command += shaders[i];
        command += ".spv";
        int status = system(command.c_str());
        if (status != 0) {
            throw std::runtime_error("Error compiling shader " + shaders[i]);
        }
    }
}






