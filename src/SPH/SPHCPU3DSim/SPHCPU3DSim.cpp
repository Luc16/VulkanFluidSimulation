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
        instanceSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), 0);
        instanceSystem.createPipeline(renderer.renderPass(), instanceShaderPaths, [this](vkb::GraphicsPipeline::PipelineConfigInfo& info) {
            info.bindingDescription.push_back(instancedSpheres.getBindingDescription());
            info.attributeDescription.push_back({4, 1, VK_FORMAT_R32G32B32_SFLOAT, offsetof(InstanceData, position)});
            info.attributeDescription.push_back({5, 1, VK_FORMAT_R32G32B32_SFLOAT, offsetof(InstanceData, color)});
            info.attributeDescription.push_back({6, 1, VK_FORMAT_R32_SFLOAT, offsetof(InstanceData, scale)});
        });
    }
}

void SPHCPU3DSim::initializeObjects() {
    camera.m_translation = {-56.9685f, 51.9174f, 65.334};
    camera.m_rotation = {0.416948f, 1.95856f, 3.14159};

    createInstances();
}

void SPHCPU3DSim::createInstances() {
    vkDeviceWaitIdle(device.device());

    instancedSpheres.resizeBuffer(INSTANCE_COUNT);

    plane.setScale(BOUNDARY_SIZE);

    auto accPos = glm::vec3(EPS, EPS, EPS);
    auto spherePerSide = (uint32_t) std::cbrt(INSTANCE_COUNT);
    float step = H + 0.01f;

    for (uint32_t i = 0; i < instancedSpheres.size(); i++) {
        auto& sphere = instancedSpheres[i];
        sphere.color = glm::vec3(0.2f, 0.6f, 1.0f);
        sphere.scale = 0.8f*H;
        sphere.position = accPos + glm::vec3(randomFloat(-H/5, H/5), randomFloat(-H/5, H/5), randomFloat(-H/5, H/5));
        accPos.x += step;

        if (i % spherePerSide == spherePerSide - 1) {
            accPos.z += step;
            accPos.x = EPS;
            if (i % (spherePerSide * spherePerSide) == (spherePerSide * spherePerSide) - 1) {
                accPos.y += step;
                accPos.z = EPS;
            }
        }
    }
    instancedSpheres.updateBuffer();
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

    updateSpheres(deltaTime);
    instancedSpheres.updateBuffer();
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

            instanceSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);
            instancedSpheres.render(instanceSystem, commandBuffer);
        });
    });
    if (activateTimer) gpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(std::chrono::high_resolution_clock::now() - currentTime).count();
}

void SPHCPU3DSim::updateUniformBuffer(uint32_t frameIndex) {

    UniformBufferObject ubo{};
    camera.setPerspectiveProjection(glm::radians(50.f), renderer.getSwapChainAspectRatio(), 0.1f, 1000.f);
    ubo.view = camera.getView();
    ubo.proj = camera.getProjection();
    uniformBuffers[frameIndex]->singleWrite(&ubo);

}

void SPHCPU3DSim::showImGui(){

    ImGui::Begin("Control Panel");

    ImGui::Text("Rendering %d instances", INSTANCE_COUNT);
    ImGui::Checkbox("Display time", &activateTimer);
    if(activateTimer){
        ImGui::Text("Gpu time: %f ms", gpuTime);
        ImGui::Text("Cpu time: %f ms", cpuTime);
    }

    ImGui::SliderFloat("Gravity", &gravityFactor, 1.f, 1000.f);
    ImGui::DragFloat("Color upate", &colorUpdate, 0.0005f, 0.0f, 1.0f);
    ImGui::DragFloat("Color thresh", &densColorThreshold, 0.005f, 0.5f, 10.0f);

    if (ImGui::CollapsingHeader("Plane", ImGuiTreeNodeFlags_DefaultOpen)) {

        ImGui::SliderFloat("y", &plane.m_translation.y, -100.0f, 100.0f);
        if (ImGui::Button("Reset")) createInstances();

    }

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                ImGui::GetIO().Framerate);
    ImGui::End();

}

void SPHCPU3DSim::computeDensityPressure() {
    for (auto & particle : instancedSpheres) {
        particle.density = 0.0f;
//        particleHash.query(particle.position, H, [this, &particle](uint32_t otherIdx) {
        particleHash.query2(particle.position, H);
        for (uint32_t i = 0; i < particleHash.queryCount; i++) {
            const auto& other = instancedSpheres[particleHash.queryRes[i]];
            auto vec = particle.position - other.position;
            auto dist2 = glm::dot(vec, vec);
            if (dist2 < HSQ) {
                float partialDensity = MASS * POLY6 * std::pow(HSQ - dist2, 3.f);
                particle.density += partialDensity;
            }

        }
//        });
        particle.pressure = GAS_CONST * (particle.density - REST_DENS);
    }
}

void SPHCPU3DSim::computeForces() {
    for (auto& particle: instancedSpheres) {
        glm::vec3 fPress{0.0f}, fVisc{0.0f};
        particleHash.query2(particle.position, H);
        for (uint32_t i = 0; i < particleHash.queryCount; i++) {
//        particleHash.query(particle.position, H, [this, &particle, &fVisc, &fPress](uint32_t otherIdx) {
            const auto& other = instancedSpheres[particleHash.queryRes[i]];
            if (&other == &particle) continue;

            auto vec = other.position - particle.position;
            auto dist = glm::length(vec);

            if (dist < H) {
                fPress += -(vec / dist) * MASS * (particle.pressure + other.pressure) /
                          (2.f * other.density) * SPIKY_GRAD * std::pow(H - dist, 3.f);
                fVisc += VISC * MASS * (other.velocity - particle.velocity) /
                         other.density * VISC_LAP * (H - dist);
            }

//        });
        }
        particle.force = fPress + fVisc + gravityFactor*G*MASS/particle.density;
    }
}

void SPHCPU3DSim::integrate() {
    for (auto &particle : instancedSpheres) {

        particle.color = glm::vec3(
                std::clamp(particle.color.r - colorUpdate, 0.2f, 1.0f),
                std::clamp(particle.color.g - colorUpdate, 0.4f, 1.0f),
                std::clamp(particle.color.b + colorUpdate, 0.0f, 1.0f)
        );

        if (particle.density/MIN_DENS < densColorThreshold){
            particle.color = glm::vec3(0.8f, 0.8f, 1.0f);
        }

        // forward Euler integration
        particle.velocity += DT * particle.force / particle.density;
        particle.position += DT * particle.velocity;

        // enforce boundary conditions
        if (particle.position.x - EPS < 0.f) {
            particle.velocity.x *= BOUND_DAMPING;
            particle.position.x = EPS;
        } else if (particle.position.x + EPS > BOUNDARY_SIZE) {
            particle.velocity.x *= BOUND_DAMPING;
            particle.position.x = BOUNDARY_SIZE - EPS;
        }
        if (particle.position.z - EPS < 0.f) {
            particle.velocity.z *= BOUND_DAMPING;
            particle.position.z = EPS;
        } else if (particle.position.z + EPS > BOUNDARY_SIZE) {
            particle.velocity.z *= BOUND_DAMPING;
            particle.position.z = BOUNDARY_SIZE - EPS;
        }
        if (particle.position.y - EPS < plane.m_translation.y) {
            particle.velocity.y *= BOUND_DAMPING;
            particle.position.y = plane.m_translation.y + EPS;
        }
//        else if (particle.position.y + EPS > 25.f) {
//            particle.velocity.y *= BOUND_DAMPING;
//            particle.position.y = 25.f - EPS;
//        }

    }
}

void SPHCPU3DSim::updateSpheres(float deltaTime){

    particleHash.create(instancedSpheres);

    auto computeDensityPressureThreaded = [this](uint32_t start, uint32_t end) {
        for (uint32_t idx = start; idx < end; idx++) {
            auto& particle = instancedSpheres[idx];
            particle.density = 0.0f;
            particleHash.query(particle.position, H, [this, &particle](uint32_t otherIdx) {
                const auto& other = instancedSpheres[otherIdx];
                auto vec = particle.position - other.position;
                auto dist2 = glm::dot(vec, vec);
                if (dist2 < HSQ) {
                    float partialDensity = MASS * POLY6 * std::pow(HSQ - dist2, 3.f);
                    particle.density += partialDensity;
                }
            });
            particle.pressure = GAS_CONST * (particle.density - REST_DENS);
        }
    };

    auto computeForcesThreaded = [this](uint32_t start, uint32_t end) {
        for (uint32_t idx = start; idx < end; idx++) {
            auto& particle = instancedSpheres[idx];
            glm::vec3 fPress{0.0f}, fVisc{0.0f};
            particleHash.query(particle.position, H, [this, &particle, &fVisc, &fPress](uint32_t otherIdx) {
                const auto& other = instancedSpheres[otherIdx];
                if (&other == &particle) return ;

                auto vec = other.position - particle.position;
                auto dist = glm::length(vec);

                if (dist < H) {
                    fPress += -(vec / dist) * MASS * (particle.pressure + other.pressure) /
                              (2.f * other.density) * SPIKY_GRAD * std::pow(H - dist, 3.f);
                    fVisc += VISC * MASS * (other.velocity - particle.velocity) /
                             other.density * VISC_LAP * (H - dist);
                }

            });
            particle.force = fPress + fVisc + gravityFactor*G*MASS/particle.density;
        }
    };

    auto integrateThreaded = [this](uint32_t start, uint32_t end) {
        for (uint32_t idx = start; idx < end; idx++) {
            auto& particle = instancedSpheres[idx];

            particle.color = glm::vec3(
                    std::clamp(particle.color.r - colorUpdate, 0.2f, 1.0f),
                    std::clamp(particle.color.g - colorUpdate, 0.4f, 1.0f),
                    std::clamp(particle.color.b + colorUpdate, 0.0f, 1.0f)
            );

            if (particle.density/MIN_DENS < densColorThreshold){
                particle.color = glm::vec3(0.8f, 0.8f, 1.0f);
            }

            // forward Euler integration
            particle.velocity += DT * particle.force / particle.density;
            particle.position += DT * particle.velocity;

            // enforce boundary conditions
            if (particle.position.x - EPS < 0.f) {
                particle.velocity.x *= BOUND_DAMPING;
                particle.position.x = EPS;
            } else if (particle.position.x + EPS > BOUNDARY_SIZE) {
                particle.velocity.x *= BOUND_DAMPING;
                particle.position.x = BOUNDARY_SIZE - EPS;
            }
            if (particle.position.z - EPS < 0.f) {
                particle.velocity.z *= BOUND_DAMPING;
                particle.position.z = EPS;
            } else if (particle.position.z + EPS > BOUNDARY_SIZE) {
                particle.velocity.z *= BOUND_DAMPING;
                particle.position.z = BOUNDARY_SIZE - EPS;
            }
            if (particle.position.y - EPS < plane.m_translation.y) {
                particle.velocity.y *= BOUND_DAMPING;
                particle.position.y = plane.m_translation.y + EPS;
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



