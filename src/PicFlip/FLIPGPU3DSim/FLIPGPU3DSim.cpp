//
// Created by luc on 11/03/23.
//

#include "FLIPGPU3DSim.h"

void FLIPGPU3DSim::onCreate() {
    camera.m_translation = {-3.85021f, 6.08832f, 4.48576f};
    camera.m_rotation = {0.72675f, 2.22789f, 3.14159f};
    camera.updateView();
    rocks.emplace_back(device, "../Models/rockA.obj", rockTex, 0.05f);
    rocks[0].translate(glm::vec3(5.0f, 0.75f, 5.0f));
    createBuffers();
    initializeObjects();

    // Default render system
    auto descriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
            .addBinding({1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr})
            .build();
    planeDescriptorSets = createDescriptorSets(descriptorLayout, {uniformBuffers[0]->descriptorInfo()},
                                               {plane.textureInfo()});
    rockDescriptorSets = createDescriptorSets(descriptorLayout, {uniformBuffers[0]->descriptorInfo()},
                                               {rockTex->descriptorInfo()});
    {
        defaultSystem.createPipelineLayout(descriptorLayout.descriptorSetLayout(),
                                           sizeof(vkb::DrawableObject::PushConstantData));
        defaultSystem.createPipeline(renderer.renderPass(), defaultShaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo &info) {
//            info.inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;

        });
    }

    skyboxDescriptorSets = createDescriptorSets(descriptorLayout,
                                                {uniformBuffers[0]->descriptorInfo()}, {skybox.descriptorInfo()});
    skyboxSystem.createPipelineLayout(descriptorLayout.descriptorSetLayout(), 0);
    skyboxSystem.createPipeline(renderer.renderPass(), skyboxShaderPaths,
                                [](vkb::GraphicsPipeline::PipelineConfigInfo &info) {
                                    info.depthStencilInfo.depthTestEnable = VK_FALSE;
                                    info.depthStencilInfo.depthWriteEnable = VK_FALSE;
                                    info.depthStencilInfo.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
                                    info.colorBlendAttachment.blendEnable = VK_FALSE;

                                    info.bindingDescription.clear();
                                    info.bindingDescription.push_back(
                                            {0, sizeof(glm::vec3), VK_VERTEX_INPUT_RATE_VERTEX});
                                    info.attributeDescription.clear();
                                    info.attributeDescription.push_back({0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0});
                                });

    auto lineDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
            .build();
    lineDescriptorSets = createDescriptorSets(lineDescriptorLayout, {uniformBuffers[0]->descriptorInfo()});
    lineSystem.createPipelineLayout(lineDescriptorLayout.descriptorSetLayout(), 0);
    lineSystem.createPipeline(renderer.renderPass(), lineShaderPaths,
                              [](vkb::GraphicsPipeline::PipelineConfigInfo &configInfo) {
                                  configInfo.inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;

                                  configInfo.attributeDescription = Point::getAttributeDescriptions();
                                  configInfo.bindingDescription = {Point::getBindingDescription()};
                                  configInfo.enableAlphaBlending();
                              });
    flipRenderer.initialize(
            *globalDescriptorPool,
            renderer,
            uniformBuffers[0],
            skybox,
            plane
    );
}

void FLIPGPU3DSim::initializeObjects(bool start) {
    vkDeviceWaitIdle(device.device());

    initializeGridLines();


    plane.setScale(dimensions);

    if (start) {
        disableEmergencyExit();
        for (auto& rock : rocks) {
            rock.createSdf(device,flipSolver.getCellSize(), dimensions);
        }
    }

    flipSolver.initialize(globalDescriptorPool, rocks, start);
//    flipSolver.initialize(globalDescriptorPool, {}, start);
}

void FLIPGPU3DSim::initializeGridLines() {
    gridLines.clear();
    gridLines.reserve(
             (flipSolver.getNumTilesX() + 1)*(flipSolver.getNumTilesY() + 1) +
                (flipSolver.getNumTilesX() + 1)*(flipSolver.getNumTilesZ() + 1) +
                (flipSolver.getNumTilesY() + 1)*(flipSolver.getNumTilesZ() + 1)
    );

    for (uint32_t i = 0; i < flipSolver.getNumTilesX() + 1; i++) {
        for (uint32_t j = 0; j < flipSolver.getNumTilesY() + 1; j++) {
            auto x = float(i) * flipSolver.getCellSize();
            auto y = float(j) * flipSolver.getCellSize();

            gridLines.push_back(Line{
                    {{x, y, 0.0f}, gridColor},
                    {{x, y, dimensions.z}, gridColor}
            });
        }
    }

    for (uint32_t i = 0; i < flipSolver.getNumTilesX() + 1; i++) {
        for (uint32_t j = 0; j < flipSolver.getNumTilesZ() + 1; j++) {
            auto x = float(i) * flipSolver.getCellSize();
            auto z = float(j) * flipSolver.getCellSize();

            gridLines.push_back(Line{
                    {{x, 0.0f, z}, gridColor},
                    {{x, dimensions.y, z}, gridColor}
            });
        }
    }

    for (uint32_t i = 0; i < flipSolver.getNumTilesY() + 1; i++) {
        for (uint32_t j = 0; j < flipSolver.getNumTilesZ() + 1; j++) {
            auto y = float(i) * flipSolver.getCellSize();
            auto z = float(j) * flipSolver.getCellSize();

            gridLines.push_back(Line{
                    {{0.0f, y, z}, gridColor},
                    {{dimensions.x, y, z}, gridColor}
            });
        }
    }
    gridLinesBuffer = std::make_unique<vkb::Buffer>(device, gridLines.size() * sizeof(Line), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
                                                                                          VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    vkb::Buffer::writeVectorToBuffer(device, gridLinesBuffer, gridLines);

}

void FLIPGPU3DSim::createBuffers() {
    VkDeviceSize bufferSize = sizeof(UniformBufferObject);
    uniformBuffers.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (size_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; ++i) {
        uniformBuffers[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        uniformBuffers[i]->map();
    }
}

void FLIPGPU3DSim::mainLoop(float deltaTime) {
    auto currentTime = std::chrono::high_resolution_clock::now();

    cameraController.moveCamera(window.window(), deltaTime, camera);

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

void FLIPGPU3DSim::renderObjects() {
    auto render = [this](VkCommandBuffer commandBuffer){
        showImGui();

        if (showParticles && ubo.renderType > 0)
            flipRenderer.runOffscreenPasses(commandBuffer, flipSolver,
                                        renderer.currentFrame(), skybox, plane, rocks, defaultSystem,
                                        planeDescriptorSets, rockDescriptorSets, skyboxDescriptorSets, renderSkybox);
        renderer.runRenderPass([this](VkCommandBuffer &commandBuffer) {
            if (renderSkybox) {
                skyboxSystem.bind(commandBuffer, &skyboxDescriptorSets[renderer.currentFrame()]);
                skybox.bindAndDraw(commandBuffer);
            }

            if (showParticles) flipRenderer.render(commandBuffer, flipSolver, renderer.currentFrame(), ubo.renderType);

            defaultSystem.bind(commandBuffer, &planeDescriptorSets[renderer.currentFrame()]);
            plane.render(defaultSystem, commandBuffer);
            defaultSystem.bind(commandBuffer, &rockDescriptorSets[renderer.currentFrame()]);
            for (auto& rock : rocks)
                rock.render(defaultSystem, commandBuffer);

            if (showGrid) drawGrid(commandBuffer);

        });
    };

    if (paused) {
        renderer.runFrame([&](VkCommandBuffer commandBuffer) {
            render(commandBuffer);
        });
        return;
    }
    renderer.runFrame([&](VkCommandBuffer commandBuffer) {
        render(commandBuffer);
    }, flipSolver.computeSemaphore(), vkb::ComputeShaderHandler::waitStages());

}

void FLIPGPU3DSim::drawGrid(VkCommandBuffer commandBuffer) {
    lineSystem.bind(commandBuffer, &lineDescriptorSets[renderer.currentFrame()]);

    VkBuffer vb = gridLinesBuffer->getBuffer();
    VkDeviceSize offsets[] = {0};
    vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
    vkCmdDraw(commandBuffer, 2*gridLines.size(), 1, 0, 0);
}

void FLIPGPU3DSim::applyGridLineColor() {
    for (auto& line : gridLines) {
        line.setColor(gridColor);
    }
    vkb::Buffer::writeVectorToBuffer(device, gridLinesBuffer, gridLines);

}

void FLIPGPU3DSim::updateBuffers(uint32_t frameIndex) {
    camera.setPerspectiveProjection(glm::radians(50.f), renderer.getSwapChainAspectRatio(), 0.1f, 500.f);
    ubo.view = camera.getView();
    ubo.inverseView = glm::inverse(ubo.view);
    ubo.proj = camera.getProjection();
    uniformBuffers[frameIndex]->write(&ubo);
}

void FLIPGPU3DSim::showImGui(){
    static bool changeGridColorWindowOpen = false;
    ImGui::Begin("Control Panel");

    ImGui::Text("Rendering %d particles", flipSolver.getParticleCount());
    ImGui::Text("Grid size: %d", flipSolver.getCellCount());
    ImGui::Checkbox("Display time", &activateTimer);
    if(activateTimer){
        ImGui::Text("Gpu time: %f ms", gpuTime);
        ImGui::Text("Cpu time: %f ms", cpuTime);
    }

    ImGui::Checkbox("Show Particles", &showParticles);
    ImGui::Checkbox("Show sky box", &renderSkybox);
    ImGui::Checkbox("Show Grid", &showGrid);

    if (showGrid) {
        if (ImGui::Button("Change Grid Color")) {
            changeGridColorWindowOpen = true;
        }
    }

    int ext = int(flipSolver.extensions);
    ImGui::SliderInt("Velocity extensions", &ext, 0, int(flipSolver.maxExtensions));
    flipSolver.extensions = ext;

    if (ImGui::Button("Reset") || glfwIsKeyJustPressed<GLFW_KEY_R>()) initializeObjects();
    if (ImGui::Button("Pause") || glfwIsKeyJustPressed<GLFW_KEY_SPACE>()) {
        if (paused && controlMode) {
            initializeObjects();
            controlMode = false;
        }
        paused = !paused;
    }
    if (ImGui::Button("Reset and enter control mode")) {
        paused = true;
        controlMode = true;
    }
    if (singleStep) {
        paused = true;
        singleStep = false;
    }
    if (ImGui::Button("Step") || (paused && glfwIsKeyJustPressed<GLFW_KEY_S>())) {
        singleStep = true;
        paused = false;
    }

    if (ImGui::CollapsingHeader("Rendering")) {
        ImGui::DragFloat("Particle Radius", &ubo.radius, 0.0001f, 0.0001f, 0.2f);
        ImGui::DragFloat("Transparency", &ubo.transparency, 0.001f, 0.001f, 4.0f);
        ImGui::Checkbox("Render SkyBox", &renderSkybox);

        ImGui::Text("View mode");
        static std::array<std::string, 9> renderTypes = {"Particles", "Depth", "Thickness", "Normals", "Smooth",
                                                         "Reflection", "Refraction", "Fresnel Scale", "Fresnel"};
        std::string curItem = renderTypes[ubo.renderType];
        if (ImGui::BeginCombo("##combo", curItem.c_str())) {
            for (uint32_t i = 0; i < renderTypes.size(); i++){
                bool isSelected = (curItem == renderTypes[i]);
                if (ImGui::Selectable(renderTypes[i].c_str(), isSelected)) {
                    ubo.renderType = i;
                }
                if (isSelected)
                    ImGui::SetItemDefaultFocus();
            }
            ImGui::EndCombo();
        }

        ImGui::SetCursorPosX(10.0f);
        if (ImGui::CollapsingHeader("Blur Options")) {
            ImGui::SetCursorPosX(15.0f);
            ImGui::Text("Blur mode");
            static std::array<std::string, 3> blurTypes = {"Bilateral", "Gaussian", "Bilateral 2"};
            curItem = blurTypes[ubo.blurMode];
            ImGui::SetCursorPosX(15.0f);
            if (ImGui::BeginCombo("##combo2", curItem.c_str())) {
                for (uint32_t i = 0; i < blurTypes.size(); i++){
                    bool isSelected = (curItem == blurTypes[i]);
                    if (ImGui::Selectable(blurTypes[i].c_str(), isSelected)) {
                        ubo.blurMode = i;
                    }
                    if (isSelected)
                        ImGui::SetItemDefaultFocus();
                }
                ImGui::EndCombo();
            }
            ImGui::SetCursorPosX(15.0f);
            ImGui::DragInt("Blur Iterations", &flipRenderer.blurIterations, 1, 0, 20);
            ImGui::SetCursorPosX(15.0f);
            ImGui::DragInt("Smoothing Radius", &ubo.filterRadius, 1, 0, 20);
            ImGui::SetCursorPosX(15.0f);
            ImGui::DragFloat("Blur Scale", &ubo.blurScale, 0.01f, 0.01f, 5.0f, "%.3f");
            ImGui::SetCursorPosX(15.0f);
            ImGui::DragFloat("Blur Fall Off", &ubo.blurDepthFalloff, 10.0f, 100.0f, 10000.0f);
        }
    }

    ImGui::Text("Using %s", device.getPhysicalDeviceName().c_str());
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                ImGui::GetIO().Framerate);
    ImGui::End();


    if (controlMode) {
        ImGui::Begin("Control Mode");

        int numParticles = (int) flipSolver.getParticleCount();
        ImGui::SliderInt("Num Particles", &numParticles, 16, 1000000);

        float cellSize = flipSolver.getCellSize();
        ImGui::DragFloat("Cell Size", &cellSize, 0.05f, 0.001f, 1.0f);

        auto maxBound = glm::vec3(20.0f);
        auto minBound = glm::vec3(2.0f);
        auto prev = dimensions;
        ImGui::CDragFloatRanged3("Box Size", &dimensions[0], 0.01f, &minBound[0], &maxBound[0]);

        auto numParticleMin = glm::ivec3(2);
        auto numParticleMax = glm::ivec3(5);
        ImGui::CSliderIntRanged3("Particles per cell", &flipSolver.particlePerCell[0], &numParticleMin[0], &numParticleMax[0]);

        auto startMin = glm::ivec3(2);
        auto startMax = glm::ivec3(flipSolver.getDimension()) - flipSolver.particleSpan - glm::ivec3(1);
        ImGui::CSliderIntRanged3("Particles start", &flipSolver.particleStart[0], &startMin[0], &startMax[0]);

        auto spanMin = glm::ivec3(4);
        auto spanMax = glm::ivec3(flipSolver.getDimension());
        ImGui::CSliderIntRanged3("Particles size", &flipSolver.particleSpan[0], &spanMin[0], &spanMax[0]);

        auto rockPos = rocks[0].getTranslation();
        auto prevPos = rocks[0].getTranslation();
        auto minRockPos = glm::vec3(0.0f);
        auto maxBoundRock = dimensions;
        ImGui::CSliderFloatRanged3("Rock Position", &rockPos[0], &minRockPos[0], &maxBoundRock[0]);
        rocks[0].translate(rockPos - prevPos);
        // boundary

        if (prev != dimensions || cellSize != flipSolver.getCellSize() || numParticles != (int) flipSolver.getParticleCount()) {
            flipSolver.updateUniformBuffers(
                    numParticles,
                    dimensions,
                    cellSize,
                    -1
            );
        }

        initializeObjects(false);

        ImGui::NewLine();

        if (ImGui::Button("Launch and close")) {
            initializeObjects();
            controlMode = false;
            paused = false;
        }
        ImGui::End();
    }

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

void FLIPGPU3DSim::onResize(int width, int height) {
    ubo.screenHeight = (float) height;
    ubo.screenWidth = (float) width;

    flipRenderer.resize(renderer.getSwapChainExtent(), *globalDescriptorPool, uniformBuffers[0], skybox, plane);
}

void FLIPGPU3DSim::compileShaders() {
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






