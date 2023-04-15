//
// Created by luc on 11/03/23.
//

#include "FLIPCPU2DSim.h"

void FLIPCPU2DSim::onCreate() {
    initializeObjects();
    createUniformBuffers();

    // Default render system
    auto defaultDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
            .build();
    defaultDescriptorSets = createDescriptorSets(defaultDescriptorLayout,{uniformBuffers[0]->descriptorInfo()});
    defaultSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), 0);
    defaultSystem.createPipeline(renderer.renderPass(), shaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& configInfo){
        configInfo.colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        configInfo.inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;

        configInfo.attributeDescription = Particle::getAttributeDescriptions();
        configInfo.bindingDescription = {Particle::getBindingDescription()};
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
            densities[i] = 0.0f;
        }
        current.velX[i] = 0.0f;
        current.velY[i] = 0.0f;
        cellTypes[i] = (solidCells[i] > 0.0f) ? SOLID: AIR;
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
    auto accPos = glm::vec3(3*float(window.width())/8, 100, 0);

    for (uint32_t i = 0; i < PARTICLE_COUNT; i++) {
        auto& particle = particles[i];
        particle.position = accPos;
        particle.velocity = glm::vec3(0.0f, 1.0f, 0.0f);
        particle.color = glm::vec4(0.2f, 0.6f, 1.0f, 1.0f);

        accPos.x += radius*1.5f;

        if (accPos.x > (float) 5*float(window.width())/8) {
            accPos.y += radius*1.5f;
            accPos.x = 3*float(window.width())/8;
        }

    }

    VkDeviceSize bufferSize = PARTICLE_COUNT * sizeof(Particle);
    vkb::Buffer stagingBuffer(device, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    stagingBuffer.singleWrite(particles.data());


    particleData.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (uint32_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; i++) {
        particleData[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
                                                                           VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        device.copyBuffer(stagingBuffer.getBuffer(), particleData[i]->getBuffer(), bufferSize);
    }

    resetGrid(true);

}

void FLIPCPU2DSim::createUniformBuffers() {
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

    renderer.runFrame([this](VkCommandBuffer commandBuffer){
        showImGui();

        renderer.runRenderPass([this](VkCommandBuffer& commandBuffer){

            defaultSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);

            VkBuffer vb = particleData[renderer.currentFrame()]->getBuffer();
            VkDeviceSize offsets[] = {0};
            vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
            vkCmdDraw(commandBuffer, PARTICLE_COUNT, 1, 0, 0);

        });
    });
    if (activateTimer) gpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(std::chrono::high_resolution_clock::now() - currentTime).count();
}

void FLIPCPU2DSim::updateBuffers(uint32_t frameIndex) {
    uniformBuffers[frameIndex]->write(&ubo);


    VkDeviceSize bufferSize = PARTICLE_COUNT * sizeof(Particle);
    vkb::Buffer stagingBuffer(device, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    stagingBuffer.singleWrite(particles.data());


    particleData.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (uint32_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; i++) {
        device.copyBuffer(stagingBuffer.getBuffer(), particleData[i]->getBuffer(), bufferSize);
    }
}

void FLIPCPU2DSim::showImGui(){
    ImGui::Begin("Control Panel");

    ImGui::Text("Rendering %d instances", PARTICLE_COUNT);
    ImGui::Checkbox("Display time", &activateTimer);
    if(activateTimer){
        ImGui::Text("Gpu time: %f ms", gpuTime);
        ImGui::Text("Cpu time: %f ms", cpuTime);
    }


    if (ImGui::Button("Reset")) onCreate();

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                ImGui::GetIO().Framerate);
    ImGui::End();

}

void FLIPCPU2DSim::updateSimulation() {
    integrateParticles();
    particlesCollideParticles();
    particlesCollideObjects();
    transferParticlesVelocitiesToGrid();
    updateDensities();
    projectVelocities();
    transferGridVelocitiesToParticles();
}

void FLIPCPU2DSim::integrateParticles() {
    for (auto &particle : particles) {
        particle.velocity += 10 * dt * gravity;
        particle.position += dt * particle.velocity;
    }
}

void FLIPCPU2DSim::particlesCollideParticles() {
    // TODO
}

void FLIPCPU2DSim::particlesCollideObjects() {
    static constexpr float BOUND_DAMPING = 0.f;
    for (auto &particle : particles) {
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

void FLIPCPU2DSim::updateDensities() {

    for (uint32_t i = 0; i < CELL_COUNT; i++) {
        densities[i] = 0.0f;
    }
    for (auto& particle: particles) {
        uint32_t xShift = SIZE/2, yShift = SIZE/2;
        glm::ivec2 gridPos = particle.gridPos(xShift, yShift);
        auto [wdl, wdr, wul, wur] = particleGridWeights(particle, gridPos, xShift, yShift);

        if (gridPos.x < numTilesX && gridPos.y < numTilesY) densities(gridPos) += wdl;
        if (gridPos.x + 1 < numTilesX && gridPos.y < numTilesY) densities(gridPos.x + 1, gridPos.y) += wdr;
        if (gridPos.x < numTilesX && gridPos.y + 1 < numTilesY) densities(gridPos.x, + gridPos.y) += wul;
        if (gridPos.x + 1 < numTilesX && gridPos.y + 1 < numTilesY) densities(gridPos.x + 1,  + gridPos.y + 1) += wur;

    }

    if (restDensity == 0.0f) {
        float sum = 0.0f;
        float numFluidCells = 0.0f;
        for (uint32_t i = 0; i < CELL_COUNT; i++) {
            if (cellTypes[i] == FLUID) {
                sum += densities[i];
                numFluidCells++;
            }
        }
        if (numFluidCells > 0.0f)
            restDensity = sum / numFluidCells;
    }

}

std::tuple<float, float, float, float> FLIPCPU2DSim::particleGridWeights(const Particle& particle, glm::ivec2 gridPos, uint32_t xShift, uint32_t yShift){
    auto fSize = float(SIZE);

    float dx = particle.position.x - float(SIZE * gridPos.x + xShift);
    float dy = particle.position.y - float(SIZE * gridPos.y + yShift);

    // TODO isso pode ser mais eficiente?
    return {
        (1.0f - dx / fSize) * (1.0f - dy / fSize), // down left
        (dx / fSize) * (1.0f - dy / fSize), // down right
        (1.0f - dx / fSize) * (dy / fSize), // up left
        (dx / fSize) * (dy / fSize), // up right
    };
}

void FLIPCPU2DSim::transferParticlesVelocitiesToGrid() {

    previous.velX.swap(current.velX);
    previous.velY.swap(current.velY);
    resetGrid();
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
            auto [wdl, wdr, wul, wur] = particleGridWeights(particle, gridPos, xShift, yShift);

            velComponent(gridPos) += wdl*vel;
            velComponent(gridPos.x + 1, gridPos.y) += wdr*vel;
            velComponent(gridPos.x, gridPos.y + 1) += wul*vel;
            velComponent(gridPos.x + 1, gridPos.y + 1) += wur*vel;

            weightComponent(gridPos) += wdl;
            weightComponent(gridPos.x + 1, gridPos.y) += wdr;
            weightComponent(gridPos.x, gridPos.y + 1) += wul;
            weightComponent(gridPos.x + 1, gridPos.y + 1) += wur;

        }

        for (uint32_t i = 0; i < CELL_COUNT; i++) {
            if (weightComponent[i] > 0) {
                velComponent[i] /= weightComponent[i];
            }


        }

        // TODO deal with solid cells

    };

    addVelComponent(current.velX, weightVelX, true);
    addVelComponent(current.velY, weightVelY, false);



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

                if (restDensity > 0.0f && densities(i, j) > restDensity) {
                    divergent -= gasCoefficient*(densities(i, j) - restDensity);
                }

                float pressureDiff = overRelaxation*divergent/sum;

                current.velX(i, j) += pressureDiff*solidCells(i - 1, j);
                current.velX(i + 1, j) -= pressureDiff*solidCells(i + 1, j);
                current.velY(i, j) += pressureDiff*solidCells(i, j - 1);
                current.velY(i, j + 1) -= pressureDiff*solidCells(i, j + 1);

            }
        }
    }
}



