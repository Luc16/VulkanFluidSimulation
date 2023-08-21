//
// Created by luc on 13/12/22.
//

#ifndef VULKANBASE_SINEWAVESIM_H
#define VULKANBASE_SINEWAVESIM_H

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

class SineWaveFluidSim: public vkb::VulkanApp {
public:
    SineWaveFluidSim(int width, int height, const std::string &appName, vkb::Device::PhysicalDeviceType type = vkb::Device::INTEL):
            VulkanApp(width, height, appName, type) {}

    void compileShaders();

private:
    const std::string SHADER_DIR = std::string("../src/SineWaveFluidSim/Shaders/");
    const std::vector<std::string> shaders = {
            "grid.vert",
            "grid.frag",
    };

    const std::string planeModelPath = "../Models/quadXZ.obj";
    const vkb::RenderSystem::ShaderPaths shaderPaths = vkb::RenderSystem::ShaderPaths {
            SHADER_DIR + shaders[0] + ".spv",
            SHADER_DIR + shaders[1] + ".spv"
    };


    struct UniformBufferObject {
        alignas(16) glm::mat4 viewProj;
        alignas(16) glm::vec3 lightDirection = glm::normalize(glm::vec3(1.0f, 1.0f, 0.0f));
        float time = 0.0f;
        float omega = 0.14f;
        float phi = 4.0f;
        float amp = 0.5f;
        uint32_t numSines = 12;
    };

    vkb::DrawableObject plane{vkb::Model::createModelFromFile(device, planeModelPath)};

    std::vector<std::unique_ptr<vkb::Buffer>> uniformBuffers;

    vkb::RenderSystem defaultSystem{device};
    std::vector<VkDescriptorSet> defaultDescriptorSets;

    UniformBufferObject ubo;

    std::vector<glm::vec3> vertices;
    std::unique_ptr<vkb::Buffer> vertexBuffer;

    std::vector<uint32_t> indices;
    std::unique_ptr<vkb::Buffer> indexBuffer;

    vkb::Camera camera{};
    vkb::CameraMovementController cameraController{};

    uint32_t vertexPerSide = 1000;
    float scale = 500;
    float gpuTime = 0, cpuTime = 0;
    bool activateTimer = false;

    void onCreate() override;
    void initializeObjects();
    void createGrid();
    void createUniformBuffers();
    void mainLoop(float deltaTime) override;
    void renderGrid(VkCommandBuffer commandBuffer);
    void updateUniformBuffer(uint32_t frameIndex, float deltaTime);
    void showImGui();

};


#endif //VULKANBASE_SINEWAVESIM_H
