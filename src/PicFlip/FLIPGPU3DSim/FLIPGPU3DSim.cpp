//
// Created by luc on 11/03/23.
//

#include "FLIPGPU3DSim.h"

void FLIPGPU3DSim::onCreate() {
    initializeObjects();
    createBuffers();

    // Default render system
    auto descriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
            .addBinding({1, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr})
            .build();
    planeDescriptorSets = createDescriptorSets(descriptorLayout,{uniformBuffers[0]->descriptorInfo()}, {plane.textureInfo()});

    {
        defaultSystem.createPipelineLayout(descriptorLayout.descriptorSetLayout(),
                                           sizeof(vkb::DrawableObject::PushConstantData));
        defaultSystem.createPipeline(renderer.renderPass(), defaultShaderPaths);
    }

    auto defaultDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
            .build();
    defaultDescriptorSets = createDescriptorSets(defaultDescriptorLayout,{uniformBuffers[0]->descriptorInfo()});

    particleSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), 0);
    particleSystem.createPipeline(renderer.renderPass(), particleShaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& configInfo){
        configInfo.inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_POINT_LIST;

        configInfo.attributeDescription.clear();
        configInfo.attributeDescription.push_back({0, 0, VK_FORMAT_R32G32_SFLOAT, 0});
        configInfo.bindingDescription.clear();
        configInfo.bindingDescription.push_back({0, sizeof(glm::vec2), VK_VERTEX_INPUT_RATE_VERTEX});

    });
    lineSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), 0);
    lineSystem.createPipeline(renderer.renderPass(), lineShaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& configInfo){
        configInfo.inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_LINE_LIST;

        configInfo.attributeDescription = Point::getAttributeDescriptions();
        configInfo.bindingDescription = {Point::getBindingDescription()};
        configInfo.enableAlphaBlending();
    });

}

void FLIPGPU3DSim::initializeObjects() {
    vkDeviceWaitIdle(device.device());

    auto extent = window.extent();
    camera.m_translation = {-3.85021f, 6.08832f, 4.48576f};
    camera.m_rotation = {0.72675f, 2.22789f, 3.14159f};tion.z == 0) {
        gl_PointSize = 0.0;
    }
    camera.updateView();

//    ubo.radius = 2.0f*flipSolver.particleRadius();

    plane.setScale(70);

    flipSolver.initialize(globalDescriptorPool);
}

void FLIPGPU3DSim::createBuffers() {
    VkDeviceSize bufferSize = sizeof(UniformBufferObject);
    uniformBuffers.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (size_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; ++i) {
        uniformBuffers[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        uniformBuffers[i]->map();
    }
}

void FLIPGPU3DSim::mainLoop(float deltaTime) {
    auto currentTime = std::chrono::high_resolution_clock::now();

    cameraController.moveCamera(window.window(), deltaTime, camera);
    paused = true;
    if (!paused) flipSolver.updateSimulation(deltaTime);

    updateBuffers(renderer.currentFrame());

    if (activateTimer) {
        auto time = std::chrono::high_resolution_clock::now();
        cpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(time - currentTime).count();
        currentTime = time;
    }

    renderObjects();

    if (activateTimer) gpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(std::chrono::high_resolution_clock::now() - currentTime).count();
}

void FLIPGPU3DSim::renderObjects() {
    auto render = [this](){
        showImGui();

        renderer.runRenderPass([this](VkCommandBuffer &commandBuffer) {

            if (showParticles) {
                particleSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);
                VkBuffer vb = flipSolver.particleBuffer();
                VkDeviceSize offsets[] = {0};
                vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vb, offsets);
                vkCmdDraw(commandBuffer, flipSolver.getParticleCount(), 1, 0, 0);
            }

            defaultSystem.bind(commandBuffer, &planeDescriptorSets[renderer.currentFrame()]);
            plane.render(defaultSystem, commandBuffer);

        });
    };

    if (paused) {
        renderer.runFrame([&](VkCommandBuffer commandBuffer) {
            render();
        });
        return;
    }
    renderer.runFrame([&](VkCommandBuffer commandBuffer) {
        render();
    }, flipSolver.computeSemaphore(), vkb::ComputeShaderHandler::waitStages());

}

void FLIPGPU3DSim::updateBuffers(uint32_t frameIndex) {
    camera.setPerspectiveProjection(glm::radians(50.f), renderer.getSwapChainAspectRatio(), 0.1f, 500.f);
    ubo.view = camera.getView();
    ubo.inverseView = glm::inverse(ubo.view);
    ubo.proj = camera.getProjection();
    ubo.restDens = 0;
    uniformBuffers[frameIndex]->write(&ubo);
}


void FLIPGPU3DSim::showImGui(){
    ImGui::Begin("Control Panel");

    ImGui::Text("Rendering %d particles", flipSolver.getParticleCount());
    ImGui::Checkbox("Display time", &activateTimer);
    if(activateTimer){
        ImGui::Text("Gpu time: %f ms", gpuTime);
        ImGui::Text("Cpu time: %f ms", cpuTime);
    }

    ImGui::Checkbox("Show Particles", &showParticles);


    ImGui::DragFloat("Flip ratio", &flipSolver.flipRatio, 0.001f, 0.0001f, 1.0f);

    if (ImGui::Button("Reset") || glfwIsKeyJustPressed<GLFW_KEY_R>()) initializeObjects();
    if (ImGui::Button("Pause") || glfwIsKeyJustPressed<GLFW_KEY_SPACE>()) paused = !paused;

    static bool singleStep = false;
    if (singleStep) {
        paused = true;
        singleStep = false;
    }
    if (ImGui::Button("Step") || glfwIsKeyJustPressed<GLFW_KEY_S>()) {
        singleStep = true;
        paused = false;
    }

    ImGui::Text("Using %s", device.getPhysicalDeviceName().c_str());
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                ImGui::GetIO().Framerate);
    ImGui::End();
}

void FLIPGPU3DSim::compileShaders() {
    for (uint32_t i = 0; i < shaders.size(); i++) {
        std::string command{"glslc "};
        if (i < pressureSolverStartIdx) {
            command += RENDER_SHADER_DIR;
        } else if (i < computeShaderStartIdx) {
            command += PRESSURE_SOLVER_SHADER_DIR;
        } else {
            command += SIMULATIONS_SHADER_DIR;
        }
        command += shaders[i];
        command += " --target-env=vulkan1.1 ";
        command += " -o ";
        command += COMPILED_SHADER_DIR;
        command += shaders[i];
        command += ".spv";
        int status = system(command.c_str());
        if (status != 0) {
            throw std::runtime_error("Error compiling shader " + shaders[i]);
        }
    }
}






