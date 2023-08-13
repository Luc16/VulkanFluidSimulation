//
// Created by luc on 13/12/22.
//

#include "PBFCPU3DSim.h"

void PBFCPU3DSim::onCreate() {
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
            info.bindingDescription.push_back({0, sizeof(Particle), VK_VERTEX_INPUT_RATE_VERTEX});
            info.attributeDescription.clear();
            info.attributeDescription.push_back({0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Particle, position)});
            info.attributeDescription.push_back({1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Particle, color)});
        });
    }

    createFunctions();
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
    float step = H + 0.01f;

    uint32_t count = 0;
    for (uint32_t i = 0; i < particles.size(); i++) {
        auto& sphere = particles[i];
        sphere.color = glm::vec3(0.2f, 0.6f, 1.0f);
        sphere.position = accPos;
        if (activateRandomOffsets) sphere.position += glm::vec3(
                randomFloat((accPos.x != EPS) ? -H/5 : 0.0f, H/5),
                randomFloat((accPos.y != EPS) ? -H/5 : 0.0f, H/5),
                randomFloat((accPos.z != EPS) ? -H/5 : 0.0f, H/5));
        sphere.velocity = glm::vec3(0.0f);
        sphere.predPos = glm::vec3(0.0f);
        sphere.posCorrection = glm::vec3(0.0f);
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
    particleBuffer = std::make_unique<vkb::Buffer>(device, particles.size() * sizeof(Particle),
                                                 VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    vkb::Buffer::writeVectorToBuffer(device, particleBuffer, particles);
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
    auto currentTime = std::chrono::high_resolution_clock::now();

    cameraController.moveCamera(window.window(), deltaTime, camera);
    updateUniformBuffer(renderer.currentFrame());

    if (!controlMode) {
        updateParticles(deltaTime);
        vkb::Buffer::writeVectorToBuffer(device, particleBuffer, particles);
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
            plane.m_translation.y += BOUNDARY_SIZE.y;
            plane.render(defaultSystem, commandBuffer);
            plane.m_translation.y -= BOUNDARY_SIZE.y;

            particleSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);
            VkBuffer vb = particleBuffer->getBuffer();
            VkDeviceSize offsets[] = {0};
            vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
            vkCmdDraw(commandBuffer, INSTANCE_COUNT, 1, 0, 0);

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
    ImGui::DragFloat("Viscosity", &VISC, 1.f, 10.0f, 500.f);
    ImGui::DragFloat("DeltaTime", &DT, 0.00005f, 0.0005f, 0.02f, "%.5f");
    ImGui::DragFloat("Rest Density", &REST_DENS, 1.0f, 4.0f, 1000.0f, "%.0f");
    ImGui::DragFloat("Mass", &MASS, 0.5f, 1.0f, 40.0f, "%.1f");
    ImGui::DragFloat("CFM", &CFM, 1.0f, 1.0f, 1000.0f, "%.1f");
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
    grid.createAndSort(particles, sortedParticles);
    particles.swap(sortedParticles);

    for (uint32_t i = 0; i < 4; i++) {
        threadedCall(computeDensityThreaded);
        threadedCall(computeLambdaThreaded);

        threadedCall(computePositionCorrectionThreaded);

        threadedCall(correctPositionThreaded);
    }

    threadedCall(updateVelocitiesThreaded);
    //    apply vorticity and XSPH viscosity
    threadedCall(updatePositionThreaded);
}

void PBFCPU3DSim::createFunctions() {
    predictPositionsThreaded = [this](uint32_t start, uint32_t end) {
        for (uint32_t idx = start; idx < end; idx++) {
            auto& particle = particles[idx];

            particle.velocity += DT*G;
            particle.predPos = particle.position + DT*particle.velocity;
        }
    };

    computeDensityThreaded = [this](uint32_t start, uint32_t end) {
        for (uint32_t idx = start; idx < end; idx++) {
            auto& particle = particles[idx];
            particle.density = 0.0f;
            grid.query(particle.predPos, H, [this, &particle](uint32_t otherIdx) {
                const auto& other = particles[otherIdx];
                auto vec = particle.predPos - other.predPos;
                auto dist2 = glm::dot(vec, vec);
                if (dist2 < HSQ) {
                    float partialDensity = MASS * POLY6 * std::pow(HSQ - dist2, 3.f);
                    particle.density += partialDensity;
                }
            });
        }
    };

    computeLambdaThreaded = [this](uint32_t start, uint32_t end) {
        for (uint32_t idx = start; idx < end; idx++) {
            auto& particle = particles[idx];
            auto gradCiSum = glm::vec3(0.0f);
            float gradCsqSum = 0.0f;

            grid.query(particle.predPos, H, [this, &particle, &gradCiSum, &gradCsqSum](uint32_t otherIdx) {
                const auto& other = particles[otherIdx];

                auto vec = particle.predPos - other.predPos;
                auto dist2 = glm::dot(vec, vec);

                if (&particle == &other || dist2 < 0.0000001f) return;


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

            float constraint = particle.density/REST_DENS - 1;

            particle.lambda = - constraint / (gradConstraintSqSum + CFM);
        }
    };

    computePositionCorrectionThreaded = [this](uint32_t start, uint32_t end) {
        for (uint32_t idx = start; idx < end; idx++) {
            auto& particle = particles[idx];
            particle.posCorrection = glm::vec3(0.0f);

            grid.query(particle.predPos, H, [this, &particle](uint32_t otherIdx) {
                const auto& other = particles[otherIdx];

                auto vec = particle.predPos - other.predPos;
                auto dist2 = glm::dot(vec, vec);

                if (&other == &particle || dist2 < 0.0000001f) return;

                if (dist2 < HSQ) {
                    auto dist = glm::length(vec);
                    float sCorr = -0.1f * std::pow(std::pow(HSQ - dist2, 3.f) / std::pow(HSQ*(1.0f - 0.09f), 3.f), 4.0f);

                    particle.posCorrection += (vec / dist) * (particle.lambda + other.lambda + sCorr) * SPIKY_GRAD * (H - dist)*(H - dist);
                }
            });



            particle.posCorrection /= REST_DENS;
        }
    };

    correctPositionThreaded = [this](uint32_t start, uint32_t end) {
        for (uint32_t idx = start; idx < end; idx++) {
            auto& particle = particles[idx];

            particle.predPos += particle.posCorrection;

            // enforce boundary conditions
            float nearEPS = EPS + 0.02;
            if (particle.predPos.x - EPS < 0.0f) {
                particle.predPos.x = nearEPS;
            } else if (particle.predPos.x + EPS > BOUNDARY_SIZE.x) {
                particle.predPos.x = BOUNDARY_SIZE.x - nearEPS;
            }
            if (particle.predPos.z - EPS < 0.0f) {
                particle.predPos.z = nearEPS;
            } else if (particle.predPos.z + EPS > BOUNDARY_SIZE.z) {
                particle.predPos.z = BOUNDARY_SIZE.z - nearEPS;
            }
            if (particle.predPos.y - EPS < plane.m_translation.y) {
                particle.predPos.y = plane.m_translation.y + nearEPS;
            } else if (particle.predPos.y + EPS > BOUNDARY_SIZE.y) {
                particle.predPos.y = BOUNDARY_SIZE.y - nearEPS;
            }


        }
    };

    updateVelocitiesThreaded = [this](uint32_t start, uint32_t end) {
        for (uint32_t idx = start; idx < end; idx++) {
            auto& particle = particles[idx];

            particle.velocity = (particle.predPos - particle.position)/DT;

            if (particle.density >= REST_DENS) {
                particle.color = {1.0f, 0.0f, 0.0f};
            } else {
                particle.color = glm::vec3(0.2f, 0.6f, 1.0f);
            }

        }
    };

    updatePositionThreaded = [this](uint32_t start, uint32_t end) {
        for (uint32_t idx = start; idx < end; idx++) {
            auto& particle = particles[idx];

            particle.position = particle.predPos;
        }
    };

}



