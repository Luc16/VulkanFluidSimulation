//
// Created by luc on 13/12/22.
//

#include "SPHCPU3DSim.h"
#include <execution>

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
    camera.setViewTarget({0.0f, 10.0f, 10.0f}, {12.0f, -1.0f, -12.0f }, {0.0f, 1.0f, 0.0f});
    camera.m_rotation = {0, glm::radians(180.0f), glm::radians(180.0f)};

    createInstances();
}

void SPHCPU3DSim::createInstances() {
    vkDeviceWaitIdle(device.device());

    instancedSpheres.resizeBuffer(INSTANCE_COUNT);
    sphereSpeeds.resize(INSTANCE_COUNT);
    iter.resize(INSTANCE_COUNT);

    float cubeSide = 25.f;
    plane.setScale(cubeSide);

    auto accPos = glm::vec3(0.0f, 0.0f, 0.0f);
    auto spherePerLine = (int) cubeSide;

    for (uint32_t i = 0; i < instancedSpheres.size(); i++) {
        auto& sphere = instancedSpheres[i];
        sphere.color = glm::vec3(
                0.2f + randomDouble(0.0f, 0.8f),
                0.2f + randomDouble(0.0f, 0.8f),
                0.2f + randomDouble(0.0f, 0.8f)
                );
        sphere.scale = 0.01f;
        sphere.position = glm::vec3(randomFloat(1.f, 24.f), randomFloat(1.0f, 20.0f),randomFloat(1.f, 24.f));
        accPos.x += 1.5f;

        sphereSpeeds[i] = 0.0f;
        iter[i] = i;
        if (i % spherePerLine == spherePerLine - 1) {
            accPos.z -= 1.5f;
            accPos.x = 0.0f;
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

    if (ImGui::Button("Double instance count")){
        if (INSTANCE_COUNT < 8388608) INSTANCE_COUNT *= 2;
        createInstances();
    }
    if (ImGui::Button("Half instance count")){
        if (INSTANCE_COUNT > 1) INSTANCE_COUNT /= 2;
        createInstances();
    }

    ImGui::SliderFloat("Gravity", &gravityFactor, 1.f, 1000.f);

    if (ImGui::CollapsingHeader("Plane", ImGuiTreeNodeFlags_DefaultOpen)) {

        ImGui::SliderFloat("y", &plane.m_translation.y, -100.0f, 10.0f);
        if (ImGui::Button("Reset")) createInstances();

    }

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                ImGui::GetIO().Framerate);
    ImGui::End();
}

void SPHCPU3DSim::updateSpheres(float deltaTime){
    computeDensityPressure();
    computeForces();
    integrate();
}

void SPHCPU3DSim::computeDensityPressure() {
    for (auto &particle: instancedSpheres) {
        for (auto& other : instancedSpheres) {
            auto vec = particle.position - other.position;
            auto dist2 = glm::dot(vec, vec);

            if (dist2 < HSQ) {
                float partialDensity = MASS * POLY6 * std::pow(HSQ - dist2, 3.f);
                particle.density += partialDensity;
                particle.pressure += GAS_CONST * partialDensity;
            }
        }
    }

}

void SPHCPU3DSim::computeForces() {
    for (auto &particle: instancedSpheres) {
//        glm::vec3 fPress{0.0f}, fVisc{0.0f};
        for (auto& other : instancedSpheres) {
            if (&other == &particle) continue;

            auto vec = other.position - particle.position;
            auto dist = glm::length(vec);

            if (dist < H) {
                particle.force += -glm::normalize(vec) * MASS * (particle.pressure + other.pressure ) / (2.f * other.density) * SPIKY_GRAD * std::pow(H - dist, 3.f)
                                  + VISC * MASS * (other.velocity - particle.velocity) / other.density * VISC_LAP * (H - dist);
            }

        }
//        particle.force = fPress + fVisc + G*MASS/particle.density;
    }
}

void SPHCPU3DSim::integrate() {
    for (auto &particle : instancedSpheres) {
        particle.force += gravityFactor*G*MASS/particle.density;

        // forward Euler integration
        particle.velocity += DT * particle.force / particle.density;
        particle.position += DT * particle.velocity;

        // enforce boundary conditions
        if (particle.position.x - EPS < 0.f) {
            particle.velocity.x *= BOUND_DAMPING;
            particle.position.x = EPS;
        } else if (particle.position.x + EPS > 25.f) {
            particle.velocity.x *= BOUND_DAMPING;
            particle.position.x = 25.f - EPS;
        }
        if (particle.position.z - EPS < 0.f) {
            particle.velocity.z *= BOUND_DAMPING;
            particle.position.z = EPS;
        } else if (particle.position.z + EPS > 25.f) {
            particle.velocity.z *= BOUND_DAMPING;
            particle.position.z = 25.f - EPS;
        }
        if (particle.position.y - EPS < 0.f) {
            particle.velocity.y *= BOUND_DAMPING;
            particle.position.y = EPS;
        } else if (particle.position.y + EPS > 25.f) {
            particle.velocity.y *= BOUND_DAMPING;
            particle.position.y = 25.f - EPS;
        }

        particle.density = 0.0f;
        particle.pressure = -GAS_CONST*REST_DENS;
        particle.force = glm::vec3(0.0f);

    }
}


