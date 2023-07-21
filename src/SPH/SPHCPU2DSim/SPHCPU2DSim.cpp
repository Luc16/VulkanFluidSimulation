//
// Created by luc on 11/03/23.
//

#include "SPHCPU2DSim.h"

void SPHCPU2DSim::onCreate() {
    initializeObjects();
    createUniformBuffers();

    // Default render system
    auto defaultDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
            .build();
    defaultDescriptorSets = createDescriptorSets(defaultDescriptorLayout,{uniformBuffers[0]->descriptorInfo()});
    obstacleDescriptorSets = createDescriptorSets(defaultDescriptorLayout,{obstacleUniformBuffers[0]->descriptorInfo()});
    defaultSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), 0);
    defaultSystem.createPipeline(renderer.renderPass(), shaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& configInfo){
        configInfo.colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        configInfo.inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;

        configInfo.attributeDescription = Particle::getAttributeDescriptions();
        configInfo.bindingDescription = {Particle::getBindingDescription()};
        configInfo.enableAlphaBlending();
    });

}

void SPHCPU2DSim::initializeObjects() {
    camera.setViewTarget({0.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 0.0f }, {0.0f, 1.0f, 0.0f});
    camera.updateView();
    auto extent = window.extent();
    camera.setOrthographicProjection(0.0f, (float) extent.width, (float) extent.height, 0.0f, 0.1f, 1000.f);
    ubo.view = camera.getView();
    ubo.proj = camera.getProjection();
    ubo.radius = H/1.0f;
    particleRadius = ubo.radius/4;

    obstacleUbo.view = camera.getView();
    obstacleUbo.proj = camera.getProjection();
    obstacleUbo.radius = 8*H;
    obstacleRadius = obstacleUbo.radius/4;

    vkDeviceWaitIdle(device.device());

    auto accPos = glm::vec3(3*float(window.width())/8, 100, 0);

    for (uint32_t i = 0; i < PARTICLE_COUNT; i++) {
        auto& particle = particles[i];
        particle.position = accPos;
        particle.velocity = glm::vec3(0.0f);
        particle.color = glm::vec4(0.2f, 0.6f, 1.0f, 1.0f);

        accPos.x += H*0.95f;

        if (accPos.x > (float) 5*float(window.width())/8) {
            accPos.y += H*0.95f;
            accPos.x = 3*float(window.width())/8;
        }

    }
    VkDeviceSize bufferSize = PARTICLE_COUNT * sizeof(Particle);

    particleData.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (uint32_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; i++) {
        particleData[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
                                                                            VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        vkb::Buffer::writeVectorToBuffer(device, particleData[i], particles);
    }

    obstacle.color = {1.0f, 0.0f, 0.0f, 1.0f};
    obstacle.position = {window.width()/2, 4*window.height()/5, 0};

    bufferSize = sizeof(Particle);
    vkb::Buffer stagingBufferObstacle(device, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    stagingBufferObstacle.singleWrite(&obstacle);


    obstacleData.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (uint32_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; i++) {
        obstacleData[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
                                                                            VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        device.copyBuffer(stagingBufferObstacle.getBuffer(), obstacleData[i]->getBuffer(), bufferSize);
    }

}

void SPHCPU2DSim::createUniformBuffers() {
    VkDeviceSize bufferSize = sizeof(UniformBufferObject);
    uniformBuffers.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    obstacleUniformBuffers.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (size_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; ++i) {
        uniformBuffers[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        uniformBuffers[i]->map();
        obstacleUniformBuffers[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        obstacleUniformBuffers[i]->map();
    }
}

void SPHCPU2DSim::mainLoop(float deltaTime) {
    auto currentTime = std::chrono::high_resolution_clock::now();

    updateParticles();
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

            defaultSystem.bind(commandBuffer, &obstacleDescriptorSets[renderer.currentFrame()]);
            VkBuffer vb2 = obstacleData[renderer.currentFrame()]->getBuffer();
            vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb2, offsets);
            vkCmdDraw(commandBuffer, 1, 1, 0, 0);


        });
    });
    if (activateTimer) gpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(std::chrono::high_resolution_clock::now() - currentTime).count();
}

void SPHCPU2DSim::updateBuffers(uint32_t frameIndex) {
    uniformBuffers[frameIndex]->write(&ubo);

    VkDeviceSize bufferSize = PARTICLE_COUNT * sizeof(Particle);
    vkb::Buffer stagingBuffer(device, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    stagingBuffer.singleWrite(particles.data());


    device.copyBuffer(stagingBuffer.getBuffer(), particleData[frameIndex]->getBuffer(), bufferSize);


    obstacleUniformBuffers[frameIndex]->write(&obstacleUbo);
    bufferSize = sizeof(Particle);
    vkb::Buffer stagingBufferObstacle(device, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                      VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    stagingBufferObstacle.singleWrite(&obstacle);

    device.copyBuffer(stagingBufferObstacle.getBuffer(), obstacleData[frameIndex]->getBuffer(), bufferSize);
}

void SPHCPU2DSim::showImGui(){
    ImGui::Begin("Control Panel");

    ImGui::Text("Rendering %d instances", PARTICLE_COUNT);
    ImGui::Checkbox("Display time", &activateTimer);
    if(activateTimer){
        ImGui::Text("Gpu time: %f ms", gpuTime);
        ImGui::Text("Cpu time: %f ms", cpuTime);
    }

    ImGui::DragFloat("Color upate", &colorUpdate, 0.0005f, 0.0f, 1.0f);
    ImGui::DragFloat("Color thresh", &densColorThreshold, 0.02f, 0.5f, 10.0f);

    if (ImGui::Button("Reset")) onCreate();
    ImGui::Text("Using %s", device.getPhysicalDeviceName().c_str());
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                ImGui::GetIO().Framerate);
    ImGui::End();

}

void SPHCPU2DSim::updateParticles() {
    particleHash.create(particles);
    computeDensityPressure();
    computeForces();
    moveObstacle();
    integrate();
}

void SPHCPU2DSim::computeDensityPressure() {
    for (auto& particle : particles) {
        particle.density = 0.0f;
//        for (uint32_t otherIdx = 0; otherIdx < particles.size(); otherIdx++) {
        particleHash.query(particle.position, H, [this, &particle](uint32_t otherIdx) {
            auto vec = particle.position - particles[otherIdx].position;
            auto dist2 = glm::dot(vec, vec);

            if (dist2 < HSQ) {
                float partialDensity = MASS * POLY6 * std::pow(HSQ - dist2, 3.f);
                particle.density += partialDensity;
            }
//        }
        });
        particle.pressure = GAS_CONST * (particle.density - REST_DENS);
    }

}

void SPHCPU2DSim::computeForces() {
    for (auto& particle: particles) {
        glm::vec3 fPress{0.0f}, fVisc{0.0f};
//        for (uint32_t otherIdx = 0; otherIdx < particles.size(); otherIdx++) {
        particleHash.query(particle.position, H, [this, &particle, &fVisc, &fPress](uint32_t otherIdx) {
            if (&particles[otherIdx] == &particle) return;

               auto vec = particles[otherIdx].position - particle.position;
               auto dist = glm::length(vec);

               if (dist < H) {
                   fPress += -glm::normalize(vec) * MASS * (particle.pressure + particles[otherIdx].pressure) / (2.f * particles[otherIdx].density) * SPIKY_GRAD * std::pow(H - dist, 3.f);
                   fVisc += VISC * MASS * (particles[otherIdx].velocity - particle.velocity) / particles[otherIdx].density * VISC_LAP * (H - dist);
               }

        });
        particle.force = fPress + fVisc + G*MASS/particle.density;
    }
}

void SPHCPU2DSim::pushParticlesApart() {
    for (int _ = 0; _ < 10; _++) {
        for (auto & particle : particles) {
        particleHash.query(particle.position, H, [this, &particle](uint32_t otherIdx) {
                auto vec = particle.position - particles[otherIdx].position;
                auto dist2 = glm::dot(vec, vec);

                if (dist2 != 0 && dist2 < 4*particleRadius*particleRadius){
                    auto dist = std::sqrt(dist2);
                    vec = vec*(0.5f*(2*particleRadius - dist)/dist);
                    particle.position += vec;
                    particles[otherIdx].position -= vec;
                }
            });
        }
    }
}

void SPHCPU2DSim::moveObstacle() {
    static glm::vec2 prevMouse{};
    auto mouse = window.getMousePos();
    bool followMouse  = mouse.x > 0 && mouse.x < float(window.width()) && mouse.y > 0 && mouse.y < float(window.height());

    float dist2ToMouse = (mouse.x - obstacle.position.x)*(mouse.x - obstacle.position.x) +
                            (mouse.y - obstacle.position.y)*(mouse.y - obstacle.position.y);
    if (followMouse && !selectedObstacle && dist2ToMouse < obstacleRadius*obstacleRadius &&
        glfwGetMouseButton(window.window(), GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        obstacle.color = {0.8f, 0.2f, 0.2f, 1.0f};
        selectedObstacle = true;
    } else if (!followMouse || glfwGetMouseButton(window.window(), GLFW_MOUSE_BUTTON_LEFT) != GLFW_PRESS) {
        obstacle.color = {1.0f, 0.0f, 0.0f, 1.0f};
        selectedObstacle = false;
    }

    if (selectedObstacle) {
        obstacle.position.x = mouse.x;
        obstacle.position.y = mouse.y;
        obstacle.velocity.x = mouse.x - prevMouse.x;
        obstacle.velocity.y = mouse.y - prevMouse.y;
    }

    prevMouse = mouse;
}

void SPHCPU2DSim::integrate() {
    float minDist2 = (particleRadius + obstacleRadius)*(particleRadius + obstacleRadius);
    for (auto& particle : particles) {
        particle.color = glm::vec4(
                std::clamp(particle.color.r - colorUpdate, 0.2f, 1.0f),
                std::clamp(particle.color.g - colorUpdate, 0.4f, 1.0f),
                std::clamp(particle.color.b + colorUpdate, 0.0f, 1.0f),
                particle.color.a
                );

        if (particle.density/MIN_DENS < densColorThreshold){
            particle.color = glm::vec4(0.8f, 0.8f, 1.0f, 1.0f);
        }

        // forward Euler integration
        particle.velocity += DT * particle.force / particle.density;
        particle.position += DT * particle.velocity;
//    }
//
//    pushParticlesApart();
//
//    for (auto& particle : particles) {
        // enforce boundary conditions
        if (particle.position.x - EPS < 0.f) {
            particle.velocity.x *= BOUND_DAMPING;
            particle.position.x = EPS;
        }
        if (particle.position.x + EPS > float(window.width())) {
            particle.velocity.x *= BOUND_DAMPING;
            particle.position.x = float(window.width()) - EPS;
        }
        if (particle.position.y - EPS < 0.f) {
            particle.velocity.y *= BOUND_DAMPING;
            particle.position.y = EPS;
        }
//        if (particle.position.y + EPS > float(window.height())) {
//            particle.velocity.y *= BOUND_DAMPING;
//            particle.position.y = float(window.height()) - EPS;
//        }

        auto nor = particle.position - obstacle.position;
        float dist2ToObstacle = glm::dot(nor, nor);

        if (dist2ToObstacle < minDist2) {
            float dist = std::sqrt(dist2ToObstacle);
            particle.position += nor*(particleRadius + obstacleRadius - dist)/dist;
            particle.velocity = obstacle.velocity;
        }

    }

}


