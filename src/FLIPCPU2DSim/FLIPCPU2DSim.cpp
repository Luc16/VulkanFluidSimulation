//
// Created by luc on 11/03/23.
//

#include "FLIPCPU2DSim.h"

void FLIPCPU2DSim::onCreate() {
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

void FLIPCPU2DSim::initializeObjects() {
    camera.setViewTarget({0.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 0.0f }, {0.0f, 1.0f, 0.0f});
    camera.updateView();
    auto extent = window.extent();
    camera.setOrthographicProjection(0.0f, (float) extent.width, (float) extent.height, 0.0f, 0.1f, 1000.f);
    ubo.view = camera.getView();
    ubo.proj = camera.getProjection();
    ubo.radius = 2.0f*radius;

    vkDeviceWaitIdle(device.device());

    // initialize particles
    flipSolver.initializeParticles();
    flipSolver.resetGrid(true);
}

void FLIPCPU2DSim::generateGridLines() {
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

void FLIPCPU2DSim::createBuffers() {
    particleBuffer = std::make_unique<vkb::Buffer>(device, PARTICLE_COUNT * sizeof(Particle), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
                                                                                   VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    vkb::Buffer::writeVectorToBuffer(device, particleBuffer, flipSolver.particles);
    gridLinesBuffer = std::make_unique<vkb::Buffer>(device, (numTilesX + numTilesY) * sizeof(Line), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
                                                                               VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    velocityFieldBuffer = std::make_unique<vkb::Buffer>(device, 2*numTilesX*numTilesY * sizeof(Line), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
                                                                            VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    fluidQuadBuffer = std::make_unique<vkb::Buffer>(device, numTilesX*numTilesY * sizeof(GridQuad), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
                                                                                                     VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);


    VkDeviceSize bufferSize = sizeof(UniformBufferObject);
    uniformBuffers.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (size_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; ++i) {
        uniformBuffers[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        uniformBuffers[i]->map();
    }
}

void FLIPCPU2DSim::mainLoop(float deltaTime) {
    auto currentTime = std::chrono::high_resolution_clock::now();

    if (!paused) flipSolver.updateSimulation(deltaTime, flipRatio);

    updateBuffers(renderer.currentFrame());

    if (activateTimer) {
        auto time = std::chrono::high_resolution_clock::now();
        cpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(time - currentTime).count();
        currentTime = time;
    }

    renderObjects();

    if (activateTimer) gpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(std::chrono::high_resolution_clock::now() - currentTime).count();
}

void FLIPCPU2DSim::renderObjects() {
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
            if (showVelField) updateAndDrawVelocityField(commandBuffer);
            if (showFluidQuads) updateAndDrawFluidQuads(commandBuffer);


        });
    });
}

void FLIPCPU2DSim::drawGrid(VkCommandBuffer commandBuffer) {
    lineSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);

    VkBuffer vb = gridLinesBuffer->getBuffer();
    VkDeviceSize offsets[] = {0};
    vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
    vkCmdDraw(commandBuffer, 2*gridLines.size(), 1, 0, 0);
}

void FLIPCPU2DSim::applyGridLineColor() {
    for (auto& line : gridLines) {
        line.setColor(gridColor);
    }
    for (uint32_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; i++) {
        vkb::Buffer::writeVectorToBuffer(device, gridLinesBuffer, gridLines);
    }
}

void FLIPCPU2DSim::updateBuffers(uint32_t frameIndex) {
    uniformBuffers[frameIndex]->write(&ubo);

    vkb::Buffer::writeVectorToBuffer(device, particleBuffer, flipSolver.particles);
}

void FLIPCPU2DSim::updateAndDrawVelocityField(VkCommandBuffer commandBuffer) {

    velocityLines.clear();
    velocityLines.reserve(2*numTilesX*numTilesY);
    for (uint32_t j = 0; j < numTilesY-1; j++) {
        for (uint32_t i = 0; i < numTilesX-1; i++) {
//            if (cellTypes(i, j) != FLUID) continue;
            float velXCenter = (flipSolver.getVelX(i, j) + flipSolver.getVelX(i+1, j))/2;
            float velYCenter = (flipSolver.getVelY(i, j) + flipSolver.getVelY(i, j+1))/2;

            auto origin = glm::vec3(float(i*SIZE + SIZE/2), float(j*SIZE + SIZE/2), 0.0f);
            auto vec = glm::vec3(velXCenter, velYCenter, 0.0f);
            if (glm::length(vec) > float(1.5f*SIZE))  {
                vec = glm::normalize(vec)*float(1.5f*SIZE);
            }

            auto arrow = Line::arrowFromRay(origin, vec,glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f));
            velocityLines.push_back(arrow[0]);
            velocityLines.push_back(arrow[1]);
        }
    }

    if (!velocityLines.empty()) {
        vkb::Buffer::writeVectorToBuffer(device, velocityFieldBuffer, velocityLines);
    }

    lineSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);

    VkBuffer vb = velocityFieldBuffer->getBuffer();
    VkDeviceSize offsets[] = {0};
    vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
    vkCmdDraw(commandBuffer, 2*velocityLines.size(), 1, 0, 0);
}

void FLIPCPU2DSim::updateAndDrawFluidQuads(VkCommandBuffer commandBuffer) {
    fluidQuads.clear();
    fluidQuads.reserve(numTilesX*numTilesY);
    for (uint32_t j = 0; j < numTilesY; j++) {
        for (uint32_t i = 0; i < numTilesX; i++) {
            if (flipSolver.getCellType(i, j) == AIR) continue;
            auto origin = glm::vec3(float(i*SIZE), float(j*SIZE), 0.0f);

            if (flipSolver.getSolidCells(i, j) == 0) {
                fluidQuads.emplace_back(origin, SIZE, glm::vec3(0.0f, 1.0f, 0.0f));
            } else if (flipSolver.getCellType(i, j) == FLUID) {
                fluidQuads.emplace_back(origin, SIZE, glm::vec3(0.0f, 0.0f, 1.0f));
            }

        }
    }

    vkb::Buffer::writeVectorToBuffer(device, fluidQuadBuffer, fluidQuads);

    quadSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);

    VkBuffer vb = fluidQuadBuffer->getBuffer();
    VkDeviceSize offsets[] = {0};
    vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
    vkCmdDraw(commandBuffer, 6*fluidQuads.size(), 1, 0, 0);
}

void FLIPCPU2DSim::showImGui(){
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

    ImGui::Checkbox("Show Velocity Field", &showVelField);
    ImGui::Checkbox("Show Fluid Cells", &showFluidQuads);

    ImGui::DragFloat("Flip ratio", &flipRatio, 0.001f, 0.0001f, 1.0f);


    if (ImGui::Button("Reset") || glfwIsKeyJustPressed(GLFW_KEY_R)) initializeObjects();
    if (ImGui::Button("Pause") || glfwIsKeyJustPressed(GLFW_KEY_SPACE)) paused = !paused;
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






