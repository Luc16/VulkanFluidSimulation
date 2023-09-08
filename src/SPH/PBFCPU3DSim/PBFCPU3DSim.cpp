//
// Created by luc on 13/12/22.
//

#include "PBFCPU3DSim.h"

void PBFCPU3DSim::onCreate() {
    createFunctions();
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
        particleSystem.createPipeline(renderer.renderPass(), particleShaderPaths, [this](vkb::GraphicsPipeline::PipelineConfigInfo& info) {
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

void PBFCPU3DSim::initializeObjects() {
    camera.m_translation = {-56.9685f, 51.9174f, 65.334};
    camera.m_rotation = {0.416948f, 1.95856f, 3.14159};

    createInstances(true);
}

void PBFCPU3DSim::createInstances(bool activateRandomOffsets) {
    vkDeviceWaitIdle(device.device());

    particles.resize(INSTANCE_COUNT);
    sortedParticles.resize(INSTANCE_COUNT);

    plane.setScale(BOUNDARY_SIZE);

    grid = vkb::SpatialGrid(H, BOUNDARY_SIZE);

    auto accPos = initialPos;
    float step = H - 0.01f;

    uint32_t count = 0;
    for (uint32_t i = 0; i < particles.position.size(); i++) {
        particles.color[i] = glm::vec3(0.2f, 0.6f, 1.0f);
        particles.position[i] = accPos;
        if (activateRandomOffsets) particles.position[i] += glm::vec3(
                randomFloat((accPos.x != EPS) ? -H/5 : 0.0f, H/5),
                randomFloat((accPos.y != EPS) ? -H/5 : 0.0f, H/5),
                randomFloat((accPos.z != EPS) ? -H/5 : 0.0f, H/5));
        particles.velocity[i] = glm::vec3(0.0f);
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

void PBFCPU3DSim::createUniformBuffers() {
    VkDeviceSize bufferSize = sizeof(UniformBufferObject);
    uniformBuffers.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (size_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; ++i) {
        uniformBuffers[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    }
}

void PBFCPU3DSim::mainLoop(float deltaTime) {
    DT = std::clamp(deltaTime, 0.005f, 0.016f);

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

void PBFCPU3DSim::updateUniformBuffer(uint32_t frameIndex) {

    UniformBufferObject ubo{};
    camera.setPerspectiveProjection(glm::radians(50.f), renderer.getSwapChainAspectRatio(), 0.1f, 1000.f);
    ubo.viewProj = camera.getProjection()*camera.getView();
    ubo.cameraPos = camera.m_translation;
    uniformBuffers[frameIndex]->singleWrite(&ubo);

}

void PBFCPU3DSim::showImGui(){

    ImGui::Begin("Control Panel");

    ImGui::Text("Rendering %d particles", INSTANCE_COUNT);
    ImGui::Text("Grid size: %d", grid.size());

    ImGui::Checkbox("Display time", &activateTimer);
    if(activateTimer){
        ImGui::Text("Gpu time: %f ms", gpuTime);
        ImGui::Text("Cpu time: %f ms", cpuTime);
    }

    ImGui::SliderFloat("Gravity", &gravityFactor, 1.f, 1000.f);
    ImGui::DragFloat("Viscosity", &VISC, 0.001f, 0.001f, 5.0f);
    ImGui::DragFloat("DeltaTime", &DT, 0.00005f, 0.0005f, 0.02f, "%.5f");
    ImGui::DragFloat("Rest Density", &REST_DENS, 1.0f, 4.0f, 1000.0f, "%.0f");
    ImGui::DragFloat("Mass", &MASS, 0.5f, 1.0f, 40.0f, "%.1f");
    ImGui::DragFloat("CFM", &CFM, 1.0f, 1.0f, 1000.0f, "%.1f");
    ImGui::DragFloat("ARTIFICIAL PRESSURE", &ART_PRESSURE_COEF, 0.01f, 0.01f, 10.0f, "%.2f");
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

void PBFCPU3DSim::threadedCall(const std::function<void(uint32_t, uint32_t)> &func) {
    for (uint32_t i = 0; i < numThreads - 1; i++){
        threads[i] = std::jthread(func, i * particlesPerThread, (i+1) * particlesPerThread);
    }
    threads[numThreads - 1] = std::jthread(func, (numThreads - 1)*particlesPerThread, INSTANCE_COUNT);
    for (auto& thread : threads) thread.join();
}

void PBFCPU3DSim::updateParticles(float deltaTime){

    // predict positions
    threadedCall(predictPositionsThreaded)
;
    // find neighbor particles
    grid.createAndSortVec(particles, sortedParticles);
    particles.swap(sortedParticles);

    for (uint32_t i = 0; i < jacobiIterations; i++) {
        threadedCall(computeDensityThreaded);
        threadedCall(computeLambdaThreaded);

        threadedCall(computePositionCorrectionThreaded);

        threadedCall(correctPositionThreaded);
    }

    threadedCall(updateVelocitiesThreaded);
    threadedCall(applyXsphViscosityAndComputeVorticity);
    threadedCall(applyVorticity);
}

void PBFCPU3DSim::createFunctions() {
    predictPositionsThreaded = [this](uint32_t start, uint32_t end) {
        for (uint32_t idx = start; idx < end; idx++) {
            particles.velocity[idx] += DT*G;
            particles.predPos[idx] = particles.position[idx] + DT*particles.velocity[idx];

            // enforce boundary conditions
            float nearEPS = EPS + 0.02;
            if (particles.predPos[idx].x - EPS < 0.0f) {
                particles.predPos[idx].x = nearEPS;
            } else if (particles.predPos[idx].x + EPS > BOUNDARY_SIZE.x) {
                particles.predPos[idx].x = BOUNDARY_SIZE.x - nearEPS;
            }
            if (particles.predPos[idx].z - EPS < 0.0f) {
                particles.predPos[idx].z = nearEPS;
            } else if (particles.predPos[idx].z + EPS > BOUNDARY_SIZE.z) {
                particles.predPos[idx].z = BOUNDARY_SIZE.z - nearEPS;
            }
            if (particles.predPos[idx].y - EPS < plane.m_translation.y) {
                particles.predPos[idx].y = plane.m_translation.y + nearEPS;
            } else if (particles.predPos[idx].y + EPS > BOUNDARY_SIZE.y) {
                particles.predPos[idx].y = BOUNDARY_SIZE.y - nearEPS;
            }
        }
    };

    computeDensityThreaded = [this](uint32_t start, uint32_t end) {
        for (uint32_t idx = start; idx < end; idx++) {

            particles.density[idx] = 0.0f;
            grid.query(particles.predPos[idx], H, [this, idx](uint32_t otherIdx) {
                auto vec = particles.predPos[idx] - particles.predPos[otherIdx];
                auto dist2 = glm::dot(vec, vec);
                if (dist2 < HSQ) {
                    particles.density[idx] += MASS * POLY6 * std::pow(HSQ - dist2, 3.f);
                }
            });
        }
    };

    computeLambdaThreaded = [this](uint32_t start, uint32_t end) {
        for (uint32_t idx = start; idx < end; idx++) {
            auto gradCiSum = glm::vec3(0.0f);
            float gradCsqSum = 0.0f;

            grid.query(particles.predPos[idx], H, [this, idx, &gradCiSum, &gradCsqSum](uint32_t otherIdx) {

                auto vec = particles.predPos[idx] - particles.predPos[otherIdx];
                auto dist2 = glm::dot(vec, vec);

                if (idx == otherIdx || dist2 < 0.0000001f) return;

                if (dist2 < HSQ) {
                    float dist = std::sqrt(dist2);
                    float delta = H - dist;
                    glm::vec3 gradCj = (vec/dist) * SPIKY_GRAD * delta * delta;

                    // contribution from particle
                    gradCiSum += gradCj;

                    // contribution from neighbors
                    gradCsqSum += glm::dot(gradCj, gradCj);

                }
            });

            float gradConstraintSqSum = (gradCsqSum + glm::dot(gradCiSum, gradCiSum))/(REST_DENS * REST_DENS);

            float constraint = particles.density[idx]/REST_DENS - 1;

            particles.lambda[idx] = - constraint / (gradConstraintSqSum + CFM);

        }
    };

    computePositionCorrectionThreaded = [this](uint32_t start, uint32_t end) {
        for (uint32_t idx = start; idx < end; idx++) {
            particles.posCorrection[idx] = glm::vec3(0.0f);

            grid.query(particles.predPos[idx], H, [this, idx](uint32_t otherIdx) {

                auto vec = particles.predPos[idx] - particles.predPos[otherIdx];
                auto dist2 = glm::dot(vec, vec);

                if (idx == otherIdx || dist2 < 0.0000001f) return;

                if (dist2 < HSQ) {
                    auto dist = glm::length(vec);
                    float sCorr = - ART_PRESSURE_COEF * std::pow(std::pow(HSQ - dist2, 3.f) / std::pow(HSQ*(1.0f - 0.09f), 3.f), 4.0f);

                    particles.posCorrection[idx] += (vec / dist) * (particles.lambda[idx] + particles.lambda[otherIdx] + sCorr) * SPIKY_GRAD * (H - dist)*(H - dist);
                }
            });

            particles.posCorrection[idx] /= REST_DENS;
        }
    };

    correctPositionThreaded = [this](uint32_t start, uint32_t end) {
        for (uint32_t idx = start; idx < end; idx++) {
            particles.predPos[idx] += particles.posCorrection[idx];

            // enforce boundary conditions
            float nearEPS = EPS + 0.02;
            if (particles.predPos[idx].x - EPS < 0.0f) {
                particles.predPos[idx].x = nearEPS;
            } else if (particles.predPos[idx].x + EPS > BOUNDARY_SIZE.x) {
                particles.predPos[idx].x = BOUNDARY_SIZE.x - nearEPS;
            }
            if (particles.predPos[idx].z - EPS < 0.0f) {
                particles.predPos[idx].z = nearEPS;
            } else if (particles.predPos[idx].z + EPS > BOUNDARY_SIZE.z) {
                particles.predPos[idx].z = BOUNDARY_SIZE.z - nearEPS;
            }
            if (particles.predPos[idx].y - EPS < plane.m_translation.y) {
                particles.predPos[idx].y = plane.m_translation.y + nearEPS;
            } else if (particles.predPos[idx].y + EPS > BOUNDARY_SIZE.y) {
                particles.predPos[idx].y = BOUNDARY_SIZE.y - nearEPS;
            }
        }
    };

    updateVelocitiesThreaded = [this](uint32_t start, uint32_t end) {
        for (uint32_t idx = start; idx < end; idx++) {

            particles.velocity[idx] = (particles.predPos[idx] - particles.position[idx])/DT;

            if (particles.density[idx] >= REST_DENS) {
                particles.color[idx] = {1.0f, 0.0f, 0.0f};
            } else {
                particles.color[idx] = glm::vec3(0.2f, 0.6f, 1.0f);
            }

        }
    };

    applyXsphViscosityAndComputeVorticity = [this](uint32_t start, uint32_t end) {
        for (uint32_t idx = start; idx < end; idx++) {

            auto viscTerm = glm::vec3(0.0f);
            particles.vorticity[idx] = glm::vec3(0.0f);


            grid.query(particles.predPos[idx], H, [this, idx, &viscTerm](uint32_t otherIdx) {

                auto vec = particles.predPos[idx] - particles.predPos[otherIdx];
                auto dist2 = glm::dot(vec, vec);

                if (idx == otherIdx || dist2 < 0.0000001f) return;

                if (dist2 < HSQ) {
                    auto vij = (particles.velocity[otherIdx] - particles.velocity[idx]);
                    viscTerm += vij * POLY6 * std::pow(HSQ - dist2, 3.f);

                    float dist = std::sqrt(dist2);
                    particles.vorticity[idx] += glm::cross(vij, (vec/dist) * SPIKY_GRAD * (H - dist) * (H - dist));
                }
            });

            particles.posCorrection[idx] = viscTerm * VISC;
        }
    };

    applyVorticity = [this](uint32_t start, uint32_t end) {
        for (uint32_t idx = start; idx < end; idx++) {
            auto locationVector = glm::vec3(0.0f);

            grid.query(particles.predPos[idx], H, [this, idx, &locationVector](uint32_t otherIdx) {

                auto vec = particles.predPos[idx] - particles.predPos[otherIdx];
                auto dist2 = glm::dot(vec, vec);

                if (idx == otherIdx || dist2 < 0.0000001f) return;

                if (dist2 < HSQ) {
                    float dist = std::sqrt(dist2);
                    float lengthVort = glm::length(particles.vorticity[idx]);
                    if (lengthVort > 0.0f){
                        locationVector += (particles.vorticity[idx]/lengthVort)*(vec/dist) * SPIKY_GRAD * (H - dist) * (H - dist);
                    }
                }
            });

            particles.velocity[idx] += VORTICITY_COEF*glm::cross(locationVector, particles.vorticity[idx])*DT/MASS + particles.posCorrection[idx];
            particles.position[idx] = particles.predPos[idx];

        }
    };

}



