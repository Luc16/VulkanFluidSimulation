//
// Created by luc on 13/12/22.
//

#include "SPHCPU3DSim.h"

void SPHCPU3DSim::onCreate() {
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
        particleSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), 0);
        particleSystem.createPipeline(renderer.renderPass(), particleShaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& info) {
            info.inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
            info.bindingDescription.clear();
            info.bindingDescription.push_back({0, sizeof(glm::vec3), VK_VERTEX_INPUT_RATE_VERTEX});
            info.bindingDescription.push_back({1, sizeof(glm::vec3), VK_VERTEX_INPUT_RATE_VERTEX});
            info.attributeDescription.clear();
            info.attributeDescription.push_back({0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0});
            info.attributeDescription.push_back({1, 1, VK_FORMAT_R32G32B32_SFLOAT, 0});
        });
    }
}

void SPHCPU3DSim::initializeObjects() {
    camera.m_translation = {-56.9685f, 51.9174f, 65.334};
    camera.m_rotation = {0.416948f, 1.95856f, 3.14159};

    createInstances(true);
}

void SPHCPU3DSim::createInstances(bool activateRandomOffsets) {
    vkDeviceWaitIdle(device.device());

    particles.resize(INSTANCE_COUNT);
    sortedParticles.resize(INSTANCE_COUNT);

    plane.setScale(BOUNDARY_SIZE);

    grid = vkb::SpatialGrid(H, BOUNDARY_SIZE);

    auto accPos = initialPos;
    float step = H + 0.1f;

    uint32_t count = 0;
    for (uint32_t i = 0; i < INSTANCE_COUNT; i++) {
        particles.color[i] = glm::vec3(0.2f, 0.6f, 1.0f);
        particles.position[i] = accPos;
        if (activateRandomOffsets) particles.position[i] += glm::vec3(randomFloat(-H/5, H/5), randomFloat(-H/5, H/5), randomFloat(-H/5, H/5));
        particles.velocity[i] = glm::vec3(0.0f);
        particles.force[i] = glm::vec3(0.0f);
        accPos.x += step;

        if (i % numParticlesXZ.x == numParticlesXZ.x - 1) {
            count++;
            accPos.z += step;
            accPos.x = initialPos.x;
            if (count == numParticlesXZ.y) {
                count = 0;
                accPos.y += step;
                accPos.z = initialPos.z;
            }
        }
    }
    particlePosBuffer = std::make_unique<vkb::Buffer>(device, particles.position.size() * sizeof(glm::vec3),
                                                 VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    vkb::Buffer::writeVectorToBuffer(device, particlePosBuffer, particles.position);

    particleColBuffer = std::make_unique<vkb::Buffer>(device, particles.color.size() * sizeof(glm::vec3),
                                                      VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                                      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    vkb::Buffer::writeVectorToBuffer(device, particleColBuffer, particles.color);

    std::vector<uint32_t> idxs(INSTANCE_COUNT);
    std::iota(std::begin(idxs), std::end(idxs), 0);
    idxBuffer = std::make_unique<vkb::Buffer>(device, idxs.size() * sizeof(uint32_t),
                                                      VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                                      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    vkb::Buffer::writeVectorToBuffer(device, idxBuffer, idxs);
}

void SPHCPU3DSim::createUniformBuffers() {
    VkDeviceSize bufferSize = sizeof(UniformBufferObject);
    uniformBuffers.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (size_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; ++i) {
        uniformBuffers[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    }
}

void SPHCPU3DSim::mainLoop(float deltaTime) {
    auto currentTime = std::chrono::high_resolution_clock::now();

    cameraController.moveCamera(window.window(), deltaTime, camera);
    updateUniformBuffer(renderer.currentFrame());

    if (!controlMode) {
        updateParticles(deltaTime);
        vkb::Buffer::writeVectorToBuffer(device, particlePosBuffer, particles.position);
        vkb::Buffer::writeVectorToBuffer(device, particleColBuffer, particles.color);
    } else {
        createInstances(false);
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
            plane.render(defaultSystem, commandBuffer);

            particleSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);
            VkBuffer vbPos = particlePosBuffer->getBuffer();
            VkBuffer vbCol = particleColBuffer->getBuffer();
            VkDeviceSize offsets[] = {0};
            vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vbPos, offsets);
            vkCmdBindVertexBuffers(commandBuffer, 1, 1, &vbCol, offsets);
            vkCmdBindIndexBuffer(commandBuffer, idxBuffer->getBuffer(), 0, VK_INDEX_TYPE_UINT32);
            vkCmdDrawIndexed(commandBuffer, INSTANCE_COUNT, 1, 0, 0, 0);

        });
    });
    if (activateTimer) gpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(std::chrono::high_resolution_clock::now() - currentTime).count();
}

void SPHCPU3DSim::updateUniformBuffer(uint32_t frameIndex) {

    UniformBufferObject ubo{};
    camera.setPerspectiveProjection(glm::radians(50.f), renderer.getSwapChainAspectRatio(), 0.1f, 1000.f);
    ubo.viewProj = camera.getProjection()*camera.getView();
    ubo.cameraPos = camera.m_translation;
    uniformBuffers[frameIndex]->singleWrite(&ubo);

}

void SPHCPU3DSim::showImGui(){

    ImGui::Begin("Control Panel");

    ImGui::Text("Rendering %d particles", INSTANCE_COUNT);
    ImGui::Text("Grid size: %d", grid.size());

    ImGui::Checkbox("Display time", &activateTimer);
    if(activateTimer){
        ImGui::Text("Gpu time: %f ms", gpuTime);
        ImGui::Text("Cpu time: %f ms", cpuTime);
    }

    ImGui::SliderFloat("Gravity", &gravityFactor, 1.f, 1000.f);
    ImGui::DragFloat("Viscosity", &VISC, 1.f, 10.0f, 500.f);
    ImGui::DragFloat("DeltaTime", &DT, 0.00005f, 0.0005f, 0.002f, "%.5f");
//    ImGui::DragFloat("Color upate", &colorUpdate, 0.0005f, 0.0f, 1.0f);
//    ImGui::DragFloat("Color thresh", &densColorThreshold, 0.005f, 0.5f, 10.0f);

    ImGui::SliderFloat("Plane Y", &plane.m_translation.y, -100.0f, 100.0f);
//    if (ImGui::SliderFloat("Bound X", &BOUNDARY_SIZE.x, 20.0f, 75.0f))
//        plane.setScale(BOUNDARY_SIZE);

    if (ImGui::Button("Reset and enter control mode")) controlMode = true;
    if (ImGui::Button("Reset")) createInstances(true);


    ImGui::Text("Using %s", device.getPhysicalDeviceName().c_str());
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                ImGui::GetIO().Framerate);
    ImGui::End();

    if (controlMode) {
        ImGui::Begin("Control Mode");

        auto getParticleShapeSize = [this]() -> glm::vec3 {
            return {float(numParticlesXZ.x) * H + H,
                    float(INSTANCE_COUNT) / float(numParticlesXZ.x * numParticlesXZ.y) * H + H,
                    float(numParticlesXZ.y) * H + H
            };
        };

        int temp = (int) INSTANCE_COUNT;
        ImGui::SliderInt("Num Particles", &temp, 16, 500000);


        auto particleShapeSize = getParticleShapeSize();

        if (temp != INSTANCE_COUNT) {
            particlesPerThread = INSTANCE_COUNT/numThreads;
            INSTANCE_COUNT = (uint32_t) temp;

            numParticlesXZ = glm::ivec2(int(std::cbrt(float(temp))));
            particleShapeSize = getParticleShapeSize();
            for (int i = 0; i < 3; i++)
                if (BOUNDARY_SIZE[i] < particleShapeSize[i] + 2*EPS)
                    BOUNDARY_SIZE[i] = particleShapeSize[i] + 2*EPS;
        }


        glm::vec3 newBoundSize = BOUNDARY_SIZE;
        auto maxBound = glm::vec3(1000.0f);

        ImGui::CDragFloatRanged3("Boundary Size", &newBoundSize[0], 1, &particleShapeSize[0], &maxBound[0]);
        for (int i = 0; i < 3; i++){
            if (newBoundSize[i] != BOUNDARY_SIZE[i] && newBoundSize[i] >= particleShapeSize[i] + 2*EPS) {
                BOUNDARY_SIZE[i] = newBoundSize[i];
            }
        }

        auto minPos = glm::vec3(EPS);
        auto maxPos = glm::vec3(BOUNDARY_SIZE.x - particleShapeSize.x, std::max(BOUNDARY_SIZE.y - particleShapeSize.y - 3*EPS, EPS), BOUNDARY_SIZE.z - particleShapeSize.z);
        ImGui::CSliderFloatRanged3("Initial Pos", &initialPos[0], &minPos[0], &maxPos[0]);

        auto numParticleMin = glm::ivec2(2);
        auto numParticleMax = glm::ivec2(int(BOUNDARY_SIZE.x/H - H), int(BOUNDARY_SIZE.z/H - H));
        ImGui::CSliderIntRanged2("Num Particles XZ", &numParticlesXZ[0], &numParticleMin[0], &numParticleMax[0]);

        particleShapeSize = getParticleShapeSize();

        for (int i = 0; i < 3; i++){
            float newEps = (i == 1) ? 3*EPS : EPS;
            if (initialPos[i] > BOUNDARY_SIZE[i] - particleShapeSize[i] - newEps) {
                if (BOUNDARY_SIZE[i] - initialPos[i] - particleShapeSize[i] < newEps)
                    BOUNDARY_SIZE[i] = initialPos[i] + particleShapeSize[i] + newEps;
                else
                    initialPos[i] = BOUNDARY_SIZE[i] - particleShapeSize[i]  + newEps;
            }
        }

        if (ImGui::Button("Launch and close")){
            createInstances(true);
            controlMode = false;
        }
        ImGui::End();
    }

}

void SPHCPU3DSim::updateParticles(float deltaTime){

    grid.createAndSortVec(particles, sortedParticles);

    particles.swap(sortedParticles);

    auto computeDensityPressureThreaded = [this](uint32_t start, uint32_t end) {
        for (uint32_t idx = start; idx < end; idx++) {
            particles.density[idx] = 0.0f;
            grid.query(particles.position[idx], H, [this, idx](uint32_t otherIdx) {
                auto vec = particles.position[idx] - particles.position[otherIdx];
                auto dist2 = glm::dot(vec, vec);
                if (dist2 < HSQ) {
                    float partialDensity = MASS * POLY6 * std::pow(HSQ - dist2, 3.f);
                    particles.density[idx] += partialDensity;
                }
            });
            particles.pressure[idx] = GAS_CONST * (particles.density[idx] - REST_DENS);
        }
    };

    auto computeForcesThreaded = [this](uint32_t start, uint32_t end) {
        for (uint32_t idx = start; idx < end; idx++) {
            glm::vec3 fPress{0.0f}, fVisc{0.0f};
            grid.query(particles.position[idx], H, [this, idx, &fVisc, &fPress](uint32_t otherIdx) {

                if (otherIdx == idx) return;

                auto vec = particles.position[otherIdx] - particles.position[idx];
                auto dist = glm::length(vec);

                if (dist == 0){
                    particles.position[idx] += glm::vec3(0.001);
                    return;
                }

                if (dist < H) {
                    fPress += (vec / dist) * MASS * (particles.pressure[idx] + particles.pressure[otherIdx]) /
                              (2.f * particles.density[otherIdx]) * SPIKY_GRAD * std::pow(H - dist, 2.f);

                    fVisc += VISC * MASS * (particles.velocity[otherIdx] - particles.velocity[idx]) /
                             particles.density[otherIdx] * VISC_LAP * (H - dist);
                }

            });
            particles.force[idx] = fPress + fVisc + gravityFactor*G*MASS/particles.density[idx];
        }
    };

    auto integrateThreaded = [this](uint32_t start, uint32_t end) {
        for (uint32_t idx = start; idx < end; idx++) {

//            particles.color[idx] = glm::vec3(
//                    std::clamp(particles.color[idx].r - colorUpdate, 0.2f, 1.0f),
//                    std::clamp(particles.color[idx].g - colorUpdate, 0.4f, 1.0f),
//                    std::clamp(particles.color[idx].b + colorUpdate, 0.0f, 1.0f)
//            );
//
//            if (particles.density[idx]/MIN_DENS < densColorThreshold){
//                particles.color[idx] = glm::vec3(0.8f, 0.8f, 1.0f);
//            }

            // forward Euler integration
            particles.velocity[idx] += DT * particles.force[idx] / particles.density[idx];
            particles.position[idx] += DT * particles.velocity[idx];

            // enforce boundary conditions
            if (particles.position[idx].x - EPS < 0.f) {
                particles.velocity[idx].x *= BOUND_DAMPING;
                particles.position[idx].x = EPS;
            } else if (particles.position[idx].x + EPS > BOUNDARY_SIZE.x) {
                particles.velocity[idx].x *= BOUND_DAMPING;
                particles.position[idx].x = BOUNDARY_SIZE.x - EPS;
            }
            if (particles.position[idx].z - EPS < 0.f) {
                particles.velocity[idx].z *= BOUND_DAMPING;
                particles.position[idx].z = EPS;
            } else if (particles.position[idx].z + EPS > BOUNDARY_SIZE.z) {
                particles.velocity[idx].z *= BOUND_DAMPING;
                particles.position[idx].z = BOUNDARY_SIZE.z - EPS;
            }
            if (particles.position[idx].y - EPS < plane.m_translation.y) {
                particles.velocity[idx].y *= BOUND_DAMPING;
                particles.position[idx].y = plane.m_translation.y + EPS;
            } else if (particles.position[idx].y + EPS > BOUNDARY_SIZE.y) {
                particles.velocity[idx].y *= BOUND_DAMPING;
                particles.position[idx].y = BOUNDARY_SIZE.y - EPS;
            }

        }
    };

    std::array<std::function<void(uint32_t, uint32_t)>, 3> threadFunctions = {
            computeDensityPressureThreaded,
            computeForcesThreaded,
            integrateThreaded
    };

    for (const auto& function : threadFunctions) {
        for (uint32_t i = 0; i < numThreads - 1; i++){
            threads[i] = std::jthread(function, i * particlesPerThread, (i+1) * particlesPerThread);
        }
        threads[numThreads - 1] = std::jthread(function, (numThreads - 1)*particlesPerThread, INSTANCE_COUNT);
        for (auto& thread : threads) thread.join();
    }
}



