//
// Created by luc on 11/03/23.
//

#ifndef VULKANFLUIDSIMULATION_FLIPCPU2DSIM_H
#define VULKANFLUIDSIMULATION_FLIPCPU2DSIM_H

#define GLM_GTX_norm
#include <sstream>
#include "../../../external/imgui/imgui.h"
#include "../../../external/objloader/tiny_obj_loader.h"
#include "../../lib/SwapChain.h"
#include "../../lib/Buffer.h"
#include "../../lib/Model.h"
#include "../../lib/utils.h"
#include "../../lib/Texture.h"
#include "../../lib/descriptors/DescriptorSetLayout.h"
#include "../../lib/Camera.h"
#include "../../lib/CameraMovementController.h"
#include "../../lib/RenderSystem.h"
#include "../../lib/DrawableObject.h"
#include "../../lib/VulkanApp.h"
#include "../../lib/InstancedObjects.h"
#include "../../lib/ComputeSystem.h"
#include "../../lib/ComputeShaderHandler.h"
#include "../../lib/graphicsDataStructures/Matrices.h"
#include "structs.h"
#include "PressureSolver.h"
#include "FlipSolver.h"

class FLIPGPU2DSim: public vkb::VulkanApp {
public:
    constexpr static uint32_t WIDTH = 1500;
    constexpr static uint32_t HEIGHT = 1000;

    explicit FLIPGPU2DSim(const std::string &appName, vkb::Device::PhysicalDeviceType type = vkb::Device::INTEL):
            VulkanApp(WIDTH, HEIGHT, appName, type) {}

private:
    std::string DIR = std::string("../src/PicFlip/FLIPGPU2DSim/");

    static constexpr uint32_t PARTICLE_COUNT = 20000;
    static constexpr float dt = 1/60.0f;
    static constexpr float radius = 4.0f;
    static constexpr uint32_t SIZE = 10;
    constexpr static uint32_t numTilesX = WIDTH/SIZE;
    constexpr static uint32_t numTilesY = HEIGHT/SIZE;
    float flipRatio = 0.90f;
    uint32_t numIterations = 200;
    uint32_t extensions = 4;

    const vkb::RenderSystem::ShaderPaths particleShaderPaths = vkb::RenderSystem::ShaderPaths {
            DIR + "Shaders/particle.vert.spv",
            DIR + "Shaders/particle.frag.spv"
    };

    const vkb::RenderSystem::ShaderPaths defaultShaderPaths = vkb::RenderSystem::ShaderPaths {
            DIR + "Shaders/default.vert.spv",
            DIR + "Shaders/default.frag.spv"
    };

    std::vector<std::unique_ptr<vkb::Buffer>> uniformBuffers;
    UniformBufferObject ubo{};


    vkb::RenderSystem particleSystem{device};
    vkb::RenderSystem lineSystem{device};
    vkb::RenderSystem quadSystem{device};
    std::vector<VkDescriptorSet> defaultDescriptorSets;

    vkb::Camera camera{};

    FlipSolver<numTilesX, numTilesY, SIZE, PARTICLE_COUNT> flipSolver{radius, numIterations, extensions};
    std::unique_ptr<vkb::Buffer> particleBuffer;
    bool showParticles = true;

    // render grid
    std::vector<Line> gridLines;
    glm::vec3 gridColor{0.01f};
    bool showGrid = false;
    std::unique_ptr<vkb::Buffer> gridLinesBuffer;

    // render velocity field
    std::vector<Line> velocityLines;
    std::unique_ptr<vkb::Buffer> velocityFieldBuffer;
    bool showVelField = false;

    // render fluid quads
    std::vector<GridQuad> fluidQuads;
    std::unique_ptr<vkb::Buffer> fluidQuadBuffer;
    bool showFluidQuads = false;


    float gpuTime = 0, cpuTime = 0;
    bool activateTimer = false, paused = false;

    void onCreate() override;
    void initializeObjects();
    void generateGridLines();
    void createBuffers();
    void mainLoop(float deltaTime) override;
    void renderObjects();
    void drawGrid(VkCommandBuffer commandBuffer);
    void applyGridLineColor();
    void updateAndDrawVelocityField(VkCommandBuffer commandBuffer);
    void updateAndDrawFluidQuads(VkCommandBuffer commandBuffer);
    void updateBuffers(uint32_t frameIndex);
    void showImGui();
};

#endif //VULKANFLUIDSIMULATION_FLIPCPU2DSIM_H
