//
// Created by luc on 11/03/23.
//

#include "FLIPCPU2DSim.h"

void FLIPCPU2DSim::onCreate() {
    initializeObjects();
    velocityLines.resize(2*numTilesX*numTilesY);
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

void FLIPCPU2DSim::resetGrid(bool hardReset) {
    if (hardReset) {
        for (uint32_t j = 0; j < numTilesY; j++) {
            for (uint32_t i = 0; i < numTilesX; i++) {
                solidCells(i, j) = (i == 0 || i == numTilesX-1 || j == 0) ? 0.0f : 1.0f;
            }
        }
    }
    for (uint32_t i = 0; i < CELL_COUNT; i++) {
        if (hardReset) {
            previous.velX[i] = 0.0f;
            previous.velY[i] = 0.0f;
        }
        current.velX[i] = 0.0f;
        current.velY[i] = 0.0f;
        cellTypes[i] = (solidCells[i] == 0.0f) ? SOLID: AIR;
        weightVelX[i] = 0.0f;
        weightVelY[i] = 0.0f;
    }
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
    float startingX = 3*float(window.width())/8;
    auto accPos = glm::vec3(startingX, 100, 0);

    for (uint32_t i = 0; i < PARTICLE_COUNT; i++) {
        auto& particle = particles[i];
        particle.position = accPos;
        particle.velocity = glm::vec3(0.0f, 0.0f, 0.0f);
        particle.color = glm::vec4(0.2f, 0.6f, 1.0f, 1.0f);

        accPos.x += radius*1.5f;

        if (accPos.x > (float) 5*float(window.width())/8) {
            accPos.y += radius*1.5f;
            accPos.x = startingX;
        }

    }

    resetGrid(true);

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
    vkb::Buffer::writeVectorToBuffer(device, particleBuffer, particles);
    gridLinesBuffer = std::make_unique<vkb::Buffer>(device, (numTilesX + numTilesY) * sizeof(Line), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
                                                                               VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    velocityFieldBuffer = std::make_unique<vkb::Buffer>(device, velocityLines.size() * sizeof(Line), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
                                                                            VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    fluidQuadBuffer = std::make_unique<vkb::Buffer>(device, CELL_COUNT * sizeof(GridQuad), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
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

    updateSimulation();

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

    vkb::Buffer::writeVectorToBuffer(device, particleBuffer, particles);
}

void FLIPCPU2DSim::updateAndDrawVelocityField(VkCommandBuffer commandBuffer) {

    uint32_t k = 0;
    for (uint32_t j = 1; j < numTilesY-1; j++) {
        for (uint32_t i = 1; i < numTilesX-1; i++) {
            float velXCenter = (current.velX(i, j) + current.velX(i+1, j))/2;
            float velYCenter = (current.velY(i, j) + current.velY(i, j+1))/2;

            auto origin = glm::vec3(float(i*SIZE + SIZE/2), float(j*SIZE + SIZE/2), 0.0f);

            auto arrow = Line::arrowFromRay(
                    origin, glm::vec3(velXCenter, velYCenter, 0.0f),
                    glm::vec3(1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f)
            );
            velocityLines[k++] = arrow[0];
            velocityLines[k++] = arrow[1];
        }
    }

    vkb::Buffer::writeVectorToBuffer(device, velocityFieldBuffer, velocityLines);

    lineSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);

    VkBuffer vb = velocityFieldBuffer->getBuffer();
    VkDeviceSize offsets[] = {0};
    vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
    vkCmdDraw(commandBuffer, 2*velocityLines.size(), 1, 0, 0);
}

void FLIPCPU2DSim::updateAndDrawFluidQuads(VkCommandBuffer commandBuffer) {
    fluidQuads.clear();
    fluidQuads.reserve(CELL_COUNT);
    for (uint32_t j = 1; j < numTilesY-1; j++) {
        for (uint32_t i = 1; i < numTilesX-1; i++) {
            if (cellTypes(i, j) != FLUID) continue;

            auto origin = glm::vec3(float(i*SIZE), float(j*SIZE), 0.0f);

            fluidQuads.emplace_back(origin, SIZE, glm::vec3(0.0f, 0.0f, 1.0f));
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



    if (ImGui::Button("Reset")) initializeObjects();
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

void FLIPCPU2DSim::updateSimulation() {
    // how it should be:

    // advect particles
    // transfer particles velocities to grid
    // update grid (gravity)
    // project velocities
    // transfer grid velocities to particles
    advectParticles();
    transferParticlesVelocitiesToGrid();
//    projectVelocities();
    transferGridVelocitiesToParticles();
}

void FLIPCPU2DSim::advectParticles() {
    static constexpr float BOUND_DAMPING = 0.f;
    for (auto &particle : particles) {
        particle.position += dt * particle.velocity;

        if (particle.position.x - radius < 0.f) {
            particle.velocity.x = BOUND_DAMPING;
            particle.position.x = radius;
        }
        if (particle.position.x + radius > float(window.width())) {
            particle.velocity.x *= BOUND_DAMPING;
            particle.position.x = float(window.width()) - radius;
        }
        if (particle.position.y - radius < 0.f) {
            particle.velocity.y *= BOUND_DAMPING;
            particle.position.y = radius;
        }
        if (particle.position.y + radius > float(window.height())) {
            particle.velocity.y *= BOUND_DAMPING;
            particle.position.y = float(window.height()) - radius;
        }

    }
}

std::tuple<float, float, float, float> FLIPCPU2DSim::particleGridWeights(const Particle& particle, glm::ivec2 gridPos, uint32_t xShift, uint32_t yShift){
    auto fSize = float(SIZE);

    float dx = particle.position.x - float(SIZE * gridPos.x + xShift);
    float dy = particle.position.y - float(SIZE * gridPos.y + yShift);

    return {
        (1.0f - dx / fSize) * (1.0f - dy / fSize), // down left
        (dx / fSize) * (1.0f - dy / fSize), // down right
        (1.0f - dx / fSize) * (dy / fSize), // up left
        (dx / fSize) * (dy / fSize), // up right
    };
}

void FLIPCPU2DSim::applyWeightedValuesToMatrix(Matrix<float, CELL_COUNT, numTilesX> &matrix, glm::ivec2 gridPos,
                                               std::tuple<float, float, float, float> weights, float value) {
    matrix(gridPos) += std::get<0>(weights)*value;
    matrix(gridPos.x + 1, gridPos.y) +=std::get<1>(weights)*value;
    matrix(gridPos.x, gridPos.y + 1) += std::get<2>(weights)*value;
    matrix(gridPos.x + 1, gridPos.y + 1) += std::get<3>(weights)*value;
}

void FLIPCPU2DSim::transferParticlesVelocitiesToGrid() {

    previous.velX.swap(current.velX);
    previous.velY.swap(current.velY);
    // reset all cells (to air, and velocities and weights to 0)
    resetGrid();
    // set all cells with particles to fluid
    for (auto& particle : particles) {
        cellTypes(particle.gridPos()) = FLUID;
    }
    auto addVelComponent = [this](Matrix<float, CELL_COUNT, numTilesX>& velComponent, Matrix<float, CELL_COUNT, numTilesX>& weightComponent, bool isX) {
        for (auto& particle: particles) {
            float vel = particle.velocity.x;
            uint32_t xShift = 0, yShift = SIZE/2;
            if (!isX) {
                vel = particle.velocity.y;
                xShift = SIZE/2;
                yShift = 0;
            }
            glm::ivec2 gridPos = particle.gridPos(xShift, yShift);
            auto weights = particleGridWeights(particle, gridPos, xShift, yShift);
            applyWeightedValuesToMatrix(velComponent, gridPos, weights, vel);
            applyWeightedValuesToMatrix(weightComponent, gridPos, weights, 1.0f);
        }

        for (uint32_t i = 0; i < CELL_COUNT; i++) {
            if (weightComponent[i] > 0) {
                velComponent[i] /= weightComponent[i];
            }
//            // apply gravity
            if (!isX) {
                velComponent[i] -= 9.8f*dt;
            }
        }
    };

    addVelComponent(current.velX, weightVelX, true);
    addVelComponent(current.velY, weightVelY, false);
}

void FLIPCPU2DSim::projectVelocities() {
    for (uint32_t i = 0; i < CELL_COUNT; i++) {
        previous.velX[i] = current.velX[i];
        previous.velY[i] = current.velY[i];
    }

    for (uint32_t iter = 0; iter < numIterations; iter++){
        for (uint32_t j = 1; j < numTilesY - 1; j++) {
            for (uint32_t i = 1; i < numTilesX - 1; i++) {
                if (cellTypes(i, j) != FLUID) continue;

                float sum = solidCells(i - 1, j) + solidCells(i + 1, j) + solidCells(i, j - 1) + solidCells(i, j + 1);
                if (sum == 0.0f) continue;

                float divergent = current.velX(i + 1, j) - current.velX(i, j) + current.velY(i, j + 1) - current.velY(i, j);

                float pressureDiff = overRelaxation*divergent/sum;

                current.velX(i, j) += pressureDiff*solidCells(i - 1, j);
                current.velX(i + 1, j) -= pressureDiff*solidCells(i + 1, j);
                current.velY(i, j) += pressureDiff*solidCells(i, j - 1);
                current.velY(i, j + 1) -= pressureDiff*solidCells(i, j + 1);

            }
        }
    }
}

void FLIPCPU2DSim::transferGridVelocitiesToParticles() {

    auto addVelComponent = [this](Matrix<float, CELL_COUNT, numTilesX>& velComponent, Matrix<float, CELL_COUNT, numTilesX>& prevVelComponent, bool isX) {
        for (auto& particle: particles) {
            float vel = particle.velocity.x;
            uint32_t xShift = 0, yShift = SIZE/2;
            if (!isX) {
                vel = particle.velocity.y;
                xShift = SIZE/2;
                yShift = 0;
            }
            glm::ivec2 gridPos = particle.gridPos(xShift, yShift);
            auto [wdl, wdr, wul, wur] = particleGridWeights(particle, gridPos, xShift, yShift);

            // TODO olhar celulas no offset tbm
            uint32_t offsetX = isX ? 1 : 0;
            uint32_t offsetY = 1 - offsetX;
            float validDL = (cellTypes(gridPos) == AIR || cellTypes(gridPos.x + offsetX, gridPos.y + offsetY) == AIR) ? 0.0f : 1.0f;
            float validDR = (cellTypes(gridPos.x + 1, gridPos.y) == AIR || cellTypes(gridPos.x + offsetX, gridPos.y + offsetY) == AIR) ? 0.0f : 1.0f;
            float validUL = (cellTypes(gridPos.x, gridPos.y + 1) == AIR || cellTypes(gridPos.x + offsetX, gridPos.y + offsetY) == AIR) ? 0.0f : 1.0f;
            float validUR = (cellTypes(gridPos.x + 1, gridPos.y + 1) == AIR || cellTypes(gridPos.x + offsetX, gridPos.y + offsetY) == AIR) ? 0.0f : 1.0f;

            float totalWeight = wdl*validDL + wdr*validDR + wul*validUL + wur*validUR;

            if (totalWeight > 0) {
                float picContribution = (validDL*wdl*velComponent(gridPos) +
                        validDR*wdr*velComponent(gridPos.x + 1, gridPos.y) +
                        validUL*wul*velComponent(gridPos.x, gridPos.y + 1) +
                        validUR*wur*velComponent(gridPos.x + 1, gridPos.y + 1))/totalWeight;
                float correction = (validDL*wdl*(velComponent(gridPos) - prevVelComponent(gridPos)) +
                                    validDR*wdr*(velComponent(gridPos.x + 1, gridPos.y) - prevVelComponent(gridPos.x + 1, gridPos.y)) +
                                    validUL*wul*(velComponent(gridPos.x, gridPos.y + 1) - prevVelComponent(gridPos.x, gridPos.y + 1)) +
                                    validUR*wur*(velComponent(gridPos.x + 1, gridPos.y + 1) - prevVelComponent(gridPos.x + 1, gridPos.y + 1)))/totalWeight;
                float flipContribution = vel + correction;
                if (isX){
                    particle.velocity.x = (1 - flipRatio) * picContribution + flipRatio * flipContribution;
                }
                else {
                    particle.velocity.y = (1 - flipRatio) * picContribution + flipRatio * flipContribution;
                }
            }

        }

    };
    addVelComponent(current.velX, previous.velX, true);
    addVelComponent(current.velY, previous.velY, false);

}



