//
// Created by luc on 11/03/23.
//

#include "SPHCPUSim.h"

void SPHCPUSim::onCreate() {
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


void SPHCPUSim::initializeObjects() {
    camera.setViewTarget({0.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 0.0f }, {0.0f, 1.0f, 0.0f});
    camera.updateView();
    auto extent = window.extent();
    camera.setOrthographicProjection(0.0f, (float) extent.width, (float) extent.height, 0.0f, 0.1f, 1000.f);
    ubo.view = camera.getView();
    ubo.proj = camera.getProjection();
    ubo.radius = H/1.0f;

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
    vkb::Buffer stagingBuffer(device, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                              VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    stagingBuffer.singleWrite(particles.data());


    particleData.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (uint32_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; i++) {
        particleData[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
                                                                           VK_BUFFER_USAGE_TRANSFER_DST_BIT, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        device.copyBuffer(stagingBuffer.getBuffer(), particleData[i]->getBuffer(), bufferSize);
    }

}

void SPHCPUSim::createUniformBuffers() {
    VkDeviceSize bufferSize = sizeof(UniformBufferObject);
    uniformBuffers.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (size_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; ++i) {
        uniformBuffers[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        uniformBuffers[i]->map();
    }
}

void SPHCPUSim::mainLoop(float deltaTime) {
    auto currentTime = std::chrono::high_resolution_clock::now();

    updateParticles();
    updateBuffers(renderer.currentFrame(), deltaTime);

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


void SPHCPUSim::updateBuffers(uint32_t frameIndex, float deltaTime){
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

void SPHCPUSim::showImGui(){
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

void SPHCPUSim::updateParticles() {
    computeDensityPressure();
    computeForces();
    integrate();
}

void SPHCPUSim::computeDensityPressure() {
    for (auto &particle: particles) {
        particle.density = 0.0f;
        for (auto& other : particles) {
            auto vec = particle.position - other.position;
            auto dist2 = glm::dot(vec, vec);

            if (dist2 < HSQ) {
                particle.density += MASS * POLY6 * std::pow(HSQ - dist2, 3.f);
            }
        }
        particle.pressure = GAS_CONST * (particle.density - REST_DENS);
    }

}

void SPHCPUSim::computeForces() {
    for (auto &particle: particles) {
        glm::vec3 fPress{0.0f}, fVisc{0.0f};
        for (auto& other : particles) {
               if (&other == &particle) continue;

               auto vec = other.position - particle.position;
               auto dist = glm::length(vec);

               if (dist < H) {
                   fPress += -glm::normalize(vec) * MASS * (particle.pressure + other.pressure ) / (2.f * other.density) * SPIKY_GRAD * std::pow(H - dist, 3.f);
                   fVisc += VISC * MASS * (other.velocity - particle.velocity) / other.density * VISC_LAP * (H - dist);
               }

        }
        particle.force = fPress + fVisc + G*MASS/particle.density;
    }
}

void SPHCPUSim::integrate() {
    for (auto &particle : particles)
    {
        // forward Euler integration
        particle.velocity += DT * particle.force / particle.density;
        particle.position += DT * particle.velocity;

        // enforce boundary conditions
        if (particle.position.x - EPS < 0.f)
        {
            particle.velocity.x *= BOUND_DAMPING;
            particle.position.x = EPS;
        }
        if (particle.position.x + EPS > float(window.width()))
        {
            particle.velocity.x *= BOUND_DAMPING;
            particle.position.x = float(window.width()) - EPS;
        }
        if (particle.position.y - EPS < 0.f)
        {
            particle.velocity.y *= BOUND_DAMPING;
            particle.position.y = EPS;
        }
        if (particle.position.y + EPS > float(window.height()))
        {
            particle.velocity.y *= BOUND_DAMPING;
            particle.position.y = float(window.height()) - EPS;
        }
    }
}



