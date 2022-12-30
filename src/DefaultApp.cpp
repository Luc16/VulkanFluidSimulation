//
// Created by luc on 13/12/22.
//

#include "DefaultApp.h"



void DefaultApp::onCreate() {
    initializeObjects();
    createUniformBuffers();

// Room render system
    {
        auto roomDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
                .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
                .addBinding({1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1,
                             VK_SHADER_STAGE_FRAGMENT_BIT, nullptr}).build();
        roomDescriptorSets = createDescriptorSets(roomDescriptorLayout,
                                                  {uniformBuffers[0]->descriptorInfo()}, {vikingRoom.textureInfo()});
        roomSystem.createPipelineLayout(roomDescriptorLayout.descriptorSetLayout(),
                                        sizeof(vkb::DrawableObject::PushConstantData));
        roomSystem.createPipeline(renderer.renderPass(), vikingShaderPaths);
    }

// Default render system
    auto defaultDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
            .build();
    defaultDescriptorSets = createDescriptorSets(defaultDescriptorLayout, {uniformBuffers[0]->descriptorInfo()});
    {
        defaultSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(),
                                           sizeof(vkb::DrawableObject::PushConstantData));
        defaultSystem.createPipeline(renderer.renderPass(), shaderPaths);
    }

}

void DefaultApp::initializeObjects() {
    camera.setViewTarget({0.0f, 0.0f, 5.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
    camera.m_rotation = {0, glm::radians(180.0f), glm::radians(180.0f)};

    vikingRoom.translate({-2.0f, -1.2f, -0.5f});
    vikingRoom.rotateAxis(2, glm::radians(-90.0f));
    vikingRoom.rotateAxis(1, glm::radians(-90.0f));

    axis.translate({3.0f, -1.2f, -1.8f});
    axis.setScale(0.125f);

    plane.translate({0.0f, -1.0f, 0.0f});
    sphere.setScale(0.01f);

}

void DefaultApp::createUniformBuffers() {
    VkDeviceSize bufferSize = sizeof(UniformBufferObject);
    uniformBuffers.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (size_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; ++i) {
        uniformBuffers[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                                                          VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    }
}

void DefaultApp::mainLoop(float deltaTime) {

    cameraController.moveCamera(window.window(), deltaTime, camera);
    updateSphere(deltaTime);
    updateUniformBuffer(renderer.currentFrame(), deltaTime);

    renderer.runFrame([&](VkCommandBuffer commandBuffer) {

        showImGui();

        renderer.runRenderPass([&](VkCommandBuffer &commandBuffer) {
            roomSystem.bind(commandBuffer, &roomDescriptorSets[renderer.currentFrame()]);
            vikingRoom.render(roomSystem, commandBuffer);
            sphere.render(roomSystem, commandBuffer);

            defaultSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);
            axis.render(defaultSystem, commandBuffer);
            plane.render(defaultSystem, commandBuffer);

        });
    });

}

void DefaultApp::updateSphere(float deltaTime) {
    float a = 5;
    sphere.translate(deltaTime * sphereSpeed * glm::vec3(0.0f, 1.0f, 0.0f));
    sphereSpeed -= a * deltaTime;
    if (sphere.m_translation.y < plane.m_translation.y) {
        sphere.m_translation.y = plane.m_translation.y;
        sphereSpeed *= (damping - 1);
    }

}

void DefaultApp::updateUniformBuffer(uint32_t frameIndex, float deltaTime) {
    if (resetPos) {
        for (int i = 0; i < 3; i++) {
            if (i != rotAxis) vikingRoom.resetRotation(i);
        }
    }
    vikingRoom.setScale(scale);
    vikingRoom.rotateAxis(rotAxis, speed * deltaTime * glm::radians(90.0f));

    UniformBufferObject ubo{};
    camera.setPerspectiveProjection(glm::radians(50.f), renderer.getSwapChainAspectRatio(), 0.1f, 100.f);
    ubo.view = camera.getView();
    ubo.proj = camera.getProjection();
    uniformBuffers[frameIndex]->singleWrite(&ubo);

}

void DefaultApp::showImGui() {
    static bool show_demo_window = false, show_another_window = false;

    if (show_demo_window) {
        ImGui::ShowDemoWindow(&show_demo_window);
    }

    // main m_windowRef
    {

        ImGui::Begin("Control Panel");

        ImGui::Text("Other windows");
        ImGui::Checkbox("Demo Window", &show_demo_window);
        ImGui::Checkbox("Another Window", &show_another_window);

        if (ImGui::CollapsingHeader("Viking Room")) {

            ImGui::Text("Choose rotation axis:");
            ImGui::RadioButton("X-axis", &rotAxis, 2);
            ImGui::SameLine();
            ImGui::RadioButton("Y-axis", &rotAxis, 1);
            ImGui::SameLine();
            ImGui::RadioButton("Z-axis", &rotAxis, 0);
            ImGui::Checkbox("Reset Position", &resetPos);
            ImGui::SliderFloat("scale", &scale, 0.1f, 4.0f);
            ImGui::SliderFloat("speed", &speed, -4.0f, 4.0f);
//            static auto color = glm::vec3(0.0f);
//            ImGui::ColorEdit3("clear color", (float *) &color);

            if (ImGui::Button("Reset speed")) speed = 0;
        }

        if (ImGui::CollapsingHeader("Plane")) {

            ImGui::SliderFloat("x", &plane.m_translation.x, -2.0f, 2.0f);
            ImGui::SliderFloat("y", &plane.m_translation.y, -4.0f, 0.0f);
            ImGui::SliderFloat("z", &plane.m_translation.z, -2.0f, 2.0f);

//            static auto color = glm::vec3(0.0f);
//            ImGui::ColorEdit3("clear color", (float *) &color);

        }

        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                    ImGui::GetIO().Framerate);
        ImGui::End();
    }

    // 3. Show another simple m_windowRef.
    if (show_another_window) {
        ImGui::Begin("Another Window",
                     &show_another_window);   // Pass a pointer to our bool variable (the m_windowRef will have a closing button that will clear the bool when clicked)
        ImGui::Text("Hello from another m_windowRef!");
        if (ImGui::Button("Close Me"))
            show_another_window = false;
        ImGui::End();
    }

}