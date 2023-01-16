//
// Created by luc on 30/12/22.
//

#ifndef VULKANFLUIDSIMULATION_GRID2DSIM_H
#define VULKANFLUIDSIMULATION_GRID2DSIM_H

// TODO:
/*
 * Batch rendering
 * Change only parts of buffer
 *
 *  */

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

#define IX(i, j) ((i)+(N+2)*(j))

class Grid2DSim: public vkb::VulkanApp {
public:
    Grid2DSim(int width, int height, const std::string &appName, vkb::Device::PhysicalDeviceType type = vkb::Device::INTEL):
            VulkanApp(width, height, appName, type){}

private:
    const uint32_t SIZE = 10;
    uint32_t INSTANCE_COUNT = 0;

    const std::string planeModelPath = "../src/Grid2DSim/Models/quad.obj";
    const vkb::RenderSystem::ShaderPaths shaderPaths = vkb::RenderSystem::ShaderPaths {
            "../src/Grid2DSim/Shaders/default.vert.spv",
            "../src/Grid2DSim/Shaders/default.frag.spv"
    };

    const vkb::RenderSystem::ShaderPaths instanceShaderPaths = vkb::RenderSystem::ShaderPaths {
            "../src/Grid2DSim/Shaders/instancing.vert.spv",
            "../src/Grid2DSim/Shaders/instancing.frag.spv",
    };

    struct UniformBufferObject {
        alignas(16) glm::mat4 view;
        alignas(16) glm::mat4 proj;
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

    std::vector<uint32_t> iter;
    float gpuTime = 0, cpuTime = 0;
    bool activateTimer = false;
    std::vector<float> dens, prevDens, velX, prevVelX, velY, prevVelY;
    uint32_t numTilesX, numTilesY;

    void onCreate() override;
    void initializeObjects();
    void createInstances();
    void createUniformBuffers();
    void mainLoop(float deltaTime) override;
    void onResize(int width, int height) override;
    void updateGrid(float deltaTime);
    void updateUniformBuffer(uint32_t frameIndex, float deltaTime);
    void showImGui();

    // fluid simulation functions

    static void diffuse( uint32_t N, int b, std::vector<float>& x, const std::vector<float>& x0, float diff, float dt );
    static void setBounds( uint32_t N, int b, std::vector<float>& x);
    static void advect(uint32_t N, int b, std::vector<float>& d, const std::vector<float>& d0, const std::vector<float>& u, const std::vector<float>& v, float dt);
    static void project(uint32_t N, std::vector<float> &u, std::vector<float> &v, std::vector<float> &p, std::vector<float> &div);
    void updateDensities(float deltaTime);
    void updateVelocities(float deltaTime);
};



#endif //VULKANFLUIDSIMULATION_GRID2DSIM_H
