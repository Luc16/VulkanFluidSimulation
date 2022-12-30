//
// Created by luc on 30/12/22.
//

#ifndef VULKANFLUIDSIMULATION_GRID2DSIM_H
#define VULKANFLUIDSIMULATION_GRID2DSIM_H


#include <sstream>
#include "../external/imgui/imgui.h"
#include "../external/objloader/tiny_obj_loader.h"
#include "lib/SwapChain.h"
#include "lib/Buffer.h"
#include "lib/Model.h"
#include "lib/utils.h"
#include "lib/Texture.h"
#include "lib/descriptors/DescriptorSetLayout.h"
#include "lib/Camera.h"
#include "lib/CameraMovementController.h"
#include "lib/RenderSystem.h"
#include "lib/DrawableObject.h"
#include "lib/VulkanApp.h"
#include "lib/InstancedObjects.h"

class Grid2DSim: public vkb::VulkanApp {
public:
    Grid2DSim(int width, int height, const std::string &appName, vkb::Device::PhysicalDeviceType type = vkb::Device::INTEL):
            VulkanApp(width, height, appName, type) {}

private:
    uint32_t INSTANCE_COUNT = 13*13;

    const std::string planeModelPath = "../models/quad.obj";
    const vkb::RenderSystem::ShaderPaths shaderPaths = vkb::RenderSystem::ShaderPaths {
            "../shaders/default.vert.spv",
            "../shaders/default.frag.spv"
    };

    const vkb::RenderSystem::ShaderPaths instanceShaderPaths = vkb::RenderSystem::ShaderPaths {
            "../shaders/instancing.vert.spv",
            "../shaders/instancing.frag.spv"
    };

    struct UniformBufferObject {
        alignas(16) glm::mat4 view;
        alignas(16) glm::mat4 proj;
        alignas(16) glm::vec3 lightDirection = glm::normalize(glm::vec3(0.0f, 0.0f, 1.0f));
    };

    struct InstanceData {
        glm::vec3 position;
        glm::vec3 color;
        float scale;
    };

    std::vector<std::unique_ptr<vkb::Buffer>> uniformBuffers;

    vkb::RenderSystem defaultSystem{device};
    std::vector<VkDescriptorSet> defaultDescriptorSets;
    vkb::RenderSystem instanceSystem{device};

    vkb::Camera camera{};
    vkb::CameraMovementController cameraController{};

    vkb::InstancedObjects<InstanceData> grid{device, INSTANCE_COUNT, vkb::Model::createModelFromFile(device, planeModelPath)};

    std::vector<float> sphereSpeeds;
    std::vector<uint32_t> iter;
    float gpuTime = 0, cpuTime = 0;
    bool activateTimer = false;

    void onCreate() override;
    void initializeObjects();
    void createInstances();
    void createUniformBuffers();
    void mainLoop(float deltaTime) override;
    void updateUniformBuffer(uint32_t frameIndex, float deltaTime);
    void showImGui();

};



#endif //VULKANFLUIDSIMULATION_GRID2DSIM_H
