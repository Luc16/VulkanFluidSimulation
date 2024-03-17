//
// Created by luc on 11/03/23.
//

#include "FLIPGPU3DSim.h"

void FLIPGPU3DSim::onCreate() {
    camera.m_translation = {-3.85021f, 6.08832f, 4.48576f};
    camera.m_rotation = {0.72675f, 2.22789f, 3.14159f};
    camera.updateView();
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

    skyboxDescriptorSets = createDescriptorSets(descriptorLayout,
                                                {uniformBuffers[0]->descriptorInfo()}, {skybox.descriptorInfo()});
    skyboxSystem.createPipelineLayout(descriptorLayout.descriptorSetLayout(), 0);
    skyboxSystem.createPipeline(renderer.renderPass(), skyboxShaderPaths, [](vkb::GraphicsPipeline::PipelineConfigInfo& info) {
        info.depthStencilInfo.depthTestEnable = VK_FALSE;
        info.depthStencilInfo.depthWriteEnable = VK_FALSE;
        info.depthStencilInfo.depthCompareOp = VK_COMPARE_OP_LESS_OR_EQUAL;
        info.colorBlendAttachment.blendEnable = VK_FALSE;

        info.bindingDescription.clear();
        info.bindingDescription.push_back({0, sizeof(glm::vec3), VK_VERTEX_INPUT_RATE_VERTEX});
        info.attributeDescription.clear();
        info.attributeDescription.push_back({0, 0, VK_FORMAT_R32G32B32_SFLOAT, 0});
    });

    flipRenderer.initialize(
            *globalDescriptorPool,
            renderer,
            uniformBuffers[0],
            skybox,
            plane
    );
}

void FLIPGPU3DSim::initializeObjects(bool start) {
    vkDeviceWaitIdle(device.device());

    plane.setScale(dimensions);

    flipSolver.initialize(globalDescriptorPool, start);
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
    auto render = [this](VkCommandBuffer commandBuffer){
        showImGui();

        flipRenderer.runOffscreenPasses(commandBuffer, flipSolver,
                                        renderer.currentFrame(), skybox, plane, defaultSystem,
                                        planeDescriptorSets, skyboxDescriptorSets, renderSkybox);
        renderer.runRenderPass([this](VkCommandBuffer &commandBuffer) {
            if (renderSkybox) {
                skyboxSystem.bind(commandBuffer, &skyboxDescriptorSets[renderer.currentFrame()]);
                skybox.bindAndDraw(commandBuffer);
            }

            flipRenderer.render(commandBuffer, flipSolver, renderer.currentFrame(), 8);

            defaultSystem.bind(commandBuffer, &planeDescriptorSets[renderer.currentFrame()]);
            plane.render(defaultSystem, commandBuffer);

        });
    };

    if (paused) {
        renderer.runFrame([&](VkCommandBuffer commandBuffer) {
            render(commandBuffer);
        });
        return;
    }
    renderer.runFrame([&](VkCommandBuffer commandBuffer) {
        render(commandBuffer);
    }, flipSolver.computeSemaphore(), vkb::ComputeShaderHandler::waitStages());

}

void FLIPGPU3DSim::updateBuffers(uint32_t frameIndex) {
    camera.setPerspectiveProjection(glm::radians(50.f), renderer.getSwapChainAspectRatio(), 0.1f, 500.f);
    ubo.view = camera.getView();
    ubo.inverseView = glm::inverse(ubo.view);
    ubo.proj = camera.getProjection();
    uniformBuffers[frameIndex]->write(&ubo);
}

// TODO: change render modes and tune params
void FLIPGPU3DSim::showImGui(){
    ImGui::Begin("Control Panel");

    ImGui::Text("Rendering %d particles", flipSolver.getParticleCount());
    ImGui::Checkbox("Display time", &activateTimer);
    if(activateTimer){
        ImGui::Text("Gpu time: %f ms", gpuTime);
        ImGui::Text("Cpu time: %f ms", cpuTime);
    }

    ImGui::Checkbox("Show Particles", &showParticles);
    ImGui::Checkbox("Show sky box", &renderSkybox);

    int ext = int(flipSolver.extensions);
    ImGui::SliderInt("Velocity extensions", &ext, 0, int(flipSolver.maxExtensions));
    flipSolver.extensions = ext;

    if (ImGui::Button("Reset") || glfwIsKeyJustPressed<GLFW_KEY_R>()) initializeObjects();
    if (ImGui::Button("Pause") || glfwIsKeyJustPressed<GLFW_KEY_SPACE>()) {
        if (paused && controlMode) {
            initializeObjects();
            controlMode = false;
        }
        paused = !paused;
    }
    if (ImGui::Button("Reset and enter control mode")) {
        paused = true;
        controlMode = true;
    }
    if (singleStep) {
        paused = true;
        singleStep = false;
    }
    if (ImGui::Button("Step") || (paused && glfwIsKeyJustPressed<GLFW_KEY_S>())) {
        singleStep = true;
        paused = false;
    }

    ImGui::Text("Using %s", device.getPhysicalDeviceName().c_str());
    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                ImGui::GetIO().Framerate);
    ImGui::End();


    if (controlMode) {
        ImGui::Begin("Control Mode");

        bool changedUniform = false;
        int numParticles = (int) flipSolver.getParticleCount();
        ImGui::SliderInt("Num Particles", &numParticles, 16, 1000000);

        float cellSize = flipSolver.getCellSize();
        ImGui::DragFloat("Cell Size", &cellSize, 0.01f, 0.01f, 1.0f);

        auto maxBound = glm::vec3(20.0f);
        auto minBound = glm::vec3(2.0f);
        auto prev = dimensions;
        ImGui::CDragFloatRanged3("Box Size", &dimensions[0], 0.01f, &minBound[0], &maxBound[0]);

        auto numParticleMin = glm::ivec3(2);
        auto numParticleMax = glm::ivec3(5);
        ImGui::CSliderIntRanged3("Particles per cell", &flipSolver.particlePerCell[0], &numParticleMin[0], &numParticleMax[0]);

        auto startMin = glm::ivec3(2);
        auto startMax = glm::ivec3(flipSolver.getDimension()) - flipSolver.particleSpan - glm::ivec3(1);
        ImGui::CSliderIntRanged3("Particles start", &flipSolver.particleStart[0], &startMin[0], &startMax[0]);

        auto spanMin = glm::ivec3(4);
        auto spanMax = glm::ivec3(flipSolver.getDimension());
        ImGui::CSliderIntRanged3("Particles size", &flipSolver.particleSpan[0], &spanMin[0], &spanMax[0]);

        // boundary

        if (prev != dimensions || cellSize != flipSolver.getCellSize() || numParticles != (int) flipSolver.getParticleCount()) {
            flipSolver.updateUniformBuffers(
                    numParticles,
                    dimensions,
                    cellSize,
                    -1
            );
        }

        initializeObjects(false);

        ImGui::NewLine();

        if (ImGui::Button("Launch and close")) {
            initializeObjects();
            controlMode = false;
            paused = false;
        }
        ImGui::End();
    }
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






