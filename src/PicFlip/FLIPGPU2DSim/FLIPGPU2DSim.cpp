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

        configInfo.attributeDescription.clear();
        configInfo.attributeDescription.push_back({0, 0, VK_FORMAT_R32G32_SFLOAT, 0});
        configInfo.bindingDescription.clear();
        configInfo.bindingDescription.push_back({0, sizeof(glm::vec2), VK_VERTEX_INPUT_RATE_VERTEX});

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
    vkDeviceWaitIdle(device.device());

    camera.setViewTarget({0.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 0.0f }, {0.0f, 1.0f, 0.0f});
    camera.updateView();
    auto extent = window.extent();
    camera.setOrthographicProjection(0.0f, (float) extent.width, (float) extent.height, 0.0f, 0.1f, 1000.f);
    ubo.view = camera.getView();
    ubo.proj = camera.getProjection();
    ubo.radius = 2.0f*flipSolver.particleRadius();

    flipSolver.initialize(globalDescriptorPool);
}

void FLIPGPU2DSim::generateGridLines() {
    gridLines.reserve(flipSolver.getNumTilesX() + flipSolver.getNumTilesY());
    for (uint32_t i = 0; i < flipSolver.getNumTilesX(); i++) {
        auto x = float(i * flipSolver.getCellSize());

        gridLines.push_back(Line{
                        {{x, 0.0f, 0.0f}, gridColor},
                        {{x, float(window.height()), 0.0f}, gridColor}
                });
    }

    for (uint32_t i = 0; i < flipSolver.getNumTilesY(); i++) {
        auto y = float(i * flipSolver.getCellSize());

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
    gridLinesBuffer = std::make_unique<vkb::Buffer>(device, (flipSolver.getNumTilesX() + flipSolver.getNumTilesY()) * sizeof(Line), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
                                                                               VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    fluidQuadBuffer = std::make_unique<vkb::Buffer>(device, (flipSolver.getNumTilesX() * flipSolver.getNumTilesY()) * sizeof(GridQuad), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
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

    if (!paused) flipSolver.updateSimulation(deltaTime);

    updateBuffers(renderer.currentFrame());

    if (activateTimer) {
        auto time = std::chrono::high_resolution_clock::now();
        cpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(time - currentTime).count();
        currentTime = time;
    }

    renderObjects();

    if (activateTimer) gpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(std::chrono::high_resolution_clock::now() - currentTime).count();
}

void FLIPGPU2DSim::renderObjects() {
    if (paused) {
        renderer.runFrame([this](VkCommandBuffer commandBuffer) {
            showImGui();

            renderer.runRenderPass([this](VkCommandBuffer &commandBuffer) {

                if (showParticles) {
                    particleSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);
                    VkBuffer vb = flipSolver.particleBuffer();
                    VkDeviceSize offsets[] = {0};
                    vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
                    vkCmdDraw(commandBuffer, flipSolver.getParticleCount(), 1, 0, 0);
                }

                if (showGrid) drawGrid(commandBuffer);


            });
        });
        return;
    }
    renderer.runFrame([this](VkCommandBuffer commandBuffer) {
        showImGui();

        renderer.runRenderPass([this](VkCommandBuffer &commandBuffer) {

            if (showParticles) {
                particleSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);
                VkBuffer vb = flipSolver.particleBuffer();
                VkDeviceSize offsets[] = {0};
                vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
                vkCmdDraw(commandBuffer, flipSolver.getParticleCount(), 1, 0, 0);
            }

            if (showGrid) drawGrid(commandBuffer);
            if (showFluidQuads) updateAndDrawFluidQuads(commandBuffer);


        });
    }, flipSolver.computeSemaphore(), vkb::ComputeShaderHandler::waitStages());

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

void FLIPGPU2DSim::updateAndDrawFluidQuads(VkCommandBuffer commandBuffer) {
    fluidQuads.clear();
    auto hasVel = flipSolver.hasVelBuffer();
    fluidQuads.reserve(flipSolver.getNumTilesX()*flipSolver.getNumTilesY());
    for (uint32_t j = 0; j < flipSolver.getNumTilesY(); j++) {
        for (uint32_t i = 0; i < flipSolver.getNumTilesX(); i++) {
//            if (flipSolver.getCellType(i, j) == AIR) continue;
            auto origin = glm::vec3(float(i*flipSolver.getCellSize()), float(j*flipSolver.getCellSize()), 0.0f);

            if (hasVel[i + j*flipSolver.getNumTilesX()] > 0) {
                fluidQuads.emplace_back(origin, flipSolver.getCellSize(), glm::vec3(0.0f, 0.0f, 1.0f));
            }
        }
    }

    if (!fluidQuads.empty()) vkb::Buffer::writeVectorToBuffer(device, fluidQuadBuffer, fluidQuads);

    quadSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);

    VkBuffer vb = fluidQuadBuffer->getBuffer();
    VkDeviceSize offsets[] = {0};
    vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
    vkCmdDraw(commandBuffer, 6*fluidQuads.size(), 1, 0, 0);
}

void FLIPGPU2DSim::updateBuffers(uint32_t frameIndex) {
    uniformBuffers[frameIndex]->write(&ubo);
}


void FLIPGPU2DSim::showImGui(){
    static bool changeGridColorWindowOpen = false;

    ImGui::Begin("Control Panel");

    ImGui::Text("Rendering %d particles", flipSolver.getParticleCount());
    ImGui::Checkbox("Display time", &activateTimer);
    if(activateTimer){
        ImGui::Text("Gpu time: %f ms", gpuTime);
        ImGui::Text("Cpu time: %f ms", cpuTime);
    }

    ImGui::Checkbox("Show Particles", &showParticles);
    ImGui::Checkbox("Show Grid", &showGrid);
    ImGui::Checkbox("Show fluid quads", &showFluidQuads);

    if (showGrid) {
        if (ImGui::Button("Change Grid Color")) {
            changeGridColorWindowOpen = true;
        }
    }


    ImGui::DragFloat("Flip ratio", &flipSolver.flipRatio, 0.001f, 0.0001f, 1.0f);

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
        if (i < pressureSolverStartIdx) {
            command += RENDER_SHADER_DIR;
        } else if (i < computeShaderStartIdx) {
            command += PRESSURE_SOLVER_SHADER_DIR;
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






