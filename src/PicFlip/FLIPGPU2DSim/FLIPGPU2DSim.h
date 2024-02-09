//
// Created by luc on 11/03/23.
//

#ifndef VULKANFLUIDSIMULATION_FLIPCPU2DSIM_H
#define VULKANFLUIDSIMULATION_FLIPCPU2DSIM_H

#define GLM_GTX_norm
#include <sstream>
#include <ranges>
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
//    constexpr static uint32_t WIDTH = 1500;
//    constexpr static uint32_t HEIGHT = 1000;
    constexpr static uint32_t WIDTH = 10;
    constexpr static uint32_t HEIGHT = 10;

    explicit FLIPGPU2DSim(const std::string &appName, vkb::Device::PhysicalDeviceType type = vkb::Device::INTEL):
            VulkanApp(WIDTH, HEIGHT, appName, type) {}

    void compileShaders();

private:
    std::string DIR = std::string("../src/PicFlip/FLIPGPU2DSim/");
    const std::string SHADER_DIR = DIR + "Shaders/";
    const std::string RENDER_SHADER_DIR = SHADER_DIR + "rendering/";
    const std::string SIMULATIONS_SHADER_DIR = SHADER_DIR + "simulation/";
    const std::string COMPILED_SHADER_DIR = SHADER_DIR + "bin/";
    const std::string PRESET_DIR = DIR + "presets/";

    static constexpr uint32_t PARTICLE_COUNT = 20000;
    static constexpr float dt = 1/60.0f;
    static constexpr float radius = 4.0f;
    static constexpr uint32_t SIZE = 10;
    constexpr static uint32_t numTilesX = WIDTH/SIZE;
    constexpr static uint32_t numTilesY = HEIGHT/SIZE;
    float flipRatio = 0.90f;
    uint32_t numIterations = 200;
    uint32_t extensions = 4;

    static constexpr uint32_t computeShaderStartIdx = 4;
    const std::vector<std::string> shaders = {
            "default.vert",
            "default.frag",
            "particle.vert",
            "particle.frag",
            "add_scaled.comp",
    };

    const vkb::RenderSystem::ShaderPaths defaultShaderPaths = vkb::RenderSystem::ShaderPaths {
            COMPILED_SHADER_DIR + shaders[0] + ".spv",
            COMPILED_SHADER_DIR + shaders[1] + ".spv"
    };

    const vkb::RenderSystem::ShaderPaths particleShaderPaths = vkb::RenderSystem::ShaderPaths {
            COMPILED_SHADER_DIR + shaders[2] + ".spv",
            COMPILED_SHADER_DIR + shaders[3] + ".spv"
    };

    std::vector<std::unique_ptr<vkb::Buffer>> uniformBuffers;
    UniformBufferObject ubo{};


    vkb::RenderSystem particleSystem{device};
    vkb::RenderSystem lineSystem{device};
    vkb::RenderSystem quadSystem{device};
    std::vector<VkDescriptorSet> defaultDescriptorSets;

    vkb::Camera camera{};

    std::function<std::string(const std::string&)> transformFunc = [this](const std::string& shader){return COMPILED_SHADER_DIR + shader + ".spv";};

    std::ranges::transform_view<std::ranges::drop_view<std::ranges::ref_view<const std::vector<std::string>>>, std::function<std::string(const std::string&)>>
            simulationShaderPaths = shaders |
                                    std::ranges::views::drop(computeShaderStartIdx) |
                                    std::ranges::views::transform(transformFunc);
    FlipSolver flipSolver {device, {simulationShaderPaths.begin(), simulationShaderPaths.end()}};
    std::unique_ptr<vkb::Buffer> particleBuffer;
    bool showParticles = true;

    // render grid
    std::vector<Line> gridLines;
    glm::vec3 gridColor{0.01f};
    bool showGrid = false;
    std::unique_ptr<vkb::Buffer> gridLinesBuffer;



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
    void updateBuffers(uint32_t frameIndex);
    void showImGui();

};

#endif //VULKANFLUIDSIMULATION_FLIPCPU2DSIM_H
