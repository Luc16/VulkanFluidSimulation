//
// Created by luc on 05/02/23.
//

#ifndef VULKANFLUIDSIMULATION_COMPUTESHADERTEST_H
#define VULKANFLUIDSIMULATION_COMPUTESHADERTEST_H


#include <sstream>
#include "../../external/imgui/imgui.h"
#include "../../external/objloader/tiny_obj_loader.h"
#include "../lib/SwapChain.h"
#include "../lib/Buffer.h"
#include "../lib/Model.h"
#include "../lib/utils.h"
#include "../lib/Texture.h"
#include "../lib/descriptors/DescriptorSetLayout.h"
#include "../lib/Camera.h"
#include "../lib/CameraMovementController.h"
#include "../lib/RenderSystem.h"
#include "../lib/DrawableObject.h"
#include "../lib/VulkanApp.h"
#include "../lib/InstancedObjects.h"
#include "../lib/ComputeSystem.h"

// TODO for compute shader:
/*
 * Create buffers to compute and send to GPU
 * Create the corresponding descriptors
 * Create compute pipeline
 * Shader stage
 * Synchronize the compute shaders
 * Call compute operation
 * */

class ComputeShaderTest: public vkb::VulkanApp {
public:
    ComputeShaderTest(int width, int height, const std::string &appName, vkb::Device::PhysicalDeviceType type = vkb::Device::INTEL):
            VulkanApp(width, height, appName, type) {}

private:
    static constexpr uint32_t PARTICLE_COUNT = 1024;
    const vkb::RenderSystem::ShaderPaths shaderPaths = vkb::RenderSystem::ShaderPaths {
            "../src/ComputeShaderTest/Shaders/default.vert.spv",
            "../src/ComputeShaderTest/Shaders/default.frag.spv"
    };
    const std::string computeShaderPath = "../src/ComputeShaderTest/Shaders/default.comp.spv";


    struct Particle {
        glm::vec2 position, velocity;
        glm::vec4 color;
    };

    struct UniformBufferObject {
        float deltaTime;
    };


    std::vector<std::unique_ptr<vkb::Buffer>> uniformBuffers;
    std::vector<std::unique_ptr<vkb::Buffer>> computeData;

    vkb::RenderSystem defaultSystem{device};
    std::vector<VkDescriptorSet> defaultDescriptorSets;
    vkb::ComputeSystem computeSystem{device};
    std::vector<VkDescriptorSet> computeDescriptorSets;

    vkb::Camera camera{};

    float gpuTime = 0, cpuTime = 0;
    bool activateTimer = false;

    void onCreate() override;
    void initializeObjects();
    void createUniformBuffers();
    void mainLoop(float deltaTime) override;
    void updateUniformBuffer(uint32_t frameIndex, float deltaTime);
    void showImGui();

};


#endif //VULKANFLUIDSIMULATION_COMPUTESHADERTEST_H
