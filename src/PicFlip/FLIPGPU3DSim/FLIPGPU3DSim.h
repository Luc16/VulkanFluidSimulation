//
// Created by luc on 11/03/23.
//

#ifndef VULKANFLUIDSIMULATION_FLIPCPU2DSIM_H
#define VULKANFLUIDSIMULATION_FLIPCPU2DSIM_H

#define GLM_GTX_norm
#include <sstream>
#include <ranges>
#include "../../../external/imgui/imgui.h"
#include "../../../external/imgui/imgui_stdlib.h"
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
#include "FlipRenderer.h"
#include "../../lib/CubeMapModel.h"
#include "RigidObject.h"
#include "FlipSceneManager.h"

class FLIPGPU3DSim: public vkb::VulkanApp {
public:
    constexpr static uint32_t WIDTH = 1500;
    constexpr static uint32_t HEIGHT = 1000;

    explicit FLIPGPU3DSim(const std::string &appName, vkb::Device::PhysicalDeviceType type = vkb::Device::NVIDIA):
            VulkanApp(WIDTH, HEIGHT, appName, type) {}

    void compileShaders();

private:
    std::string DIR = std::string("../src/PicFlip/FLIPGPU3DSim/");
    const std::string SHADER_DIR = DIR + "Shaders/";
    const std::string RENDER_SHADER_DIR = SHADER_DIR + "rendering/";
    const std::string SIMULATIONS_SHADER_DIR = SHADER_DIR + "simulation/";
    const std::string PRESSURE_SOLVER_SHADER_DIR = SHADER_DIR + "pressure_solver/";
    const std::string COMPILED_SHADER_DIR = SHADER_DIR + "bin/";
    const std::string PRESET_DIR = DIR + "presets/";
    std::string_view curFile{"bunny_bath.json"};



    static constexpr uint32_t pressureSolverStartIdx = 16;
    static constexpr uint32_t computeShaderStartIdx = pressureSolverStartIdx+6;

    const std::vector<std::string> shaders = {
            "default.vert",
            "default.frag",
            "line.vert",
            "line.frag",
            "skybox.vert",
            "skybox.frag",
            "point_particle.vert",
            "point_particle.frag",
            "ssf_depth.vert",
            "ssf_depth.frag",
            "ssf_thickness.vert",
            "ssf_thickness.frag",
            "ssf_smooth.vert",
            "ssf_smooth.frag",
            "ssf_shading.vert",
            "ssf_shading.frag",
            "add_scaled.comp",
            "dot_product.comp",
            "reduce.comp",
            "finish_dot.comp",
            "matrix_multiply.comp",
            "form_preconditioner.comp",
            "advect_particles.comp",
            "collide_with_objects.comp",
            "reset_grid.comp",
            "particles_to_grid.comp",
            "apply_weights_and_gravity.comp",
            "extend_velocities.comp",
            "apply_boundary_conditions.comp",
            "apply_obj_boundary.comp",
            "set_prev_vel.comp",
            "create_matrix.comp",
            "pressure_update.comp",
            "grid_to_particles.comp",
    };

    const vkb::RenderSystem::ShaderPaths defaultShaderPaths = vkb::RenderSystem::ShaderPaths {
            COMPILED_SHADER_DIR + shaders[0] + ".spv",
            COMPILED_SHADER_DIR + shaders[1] + ".spv"
    };

    const vkb::RenderSystem::ShaderPaths lineShaderPaths = vkb::RenderSystem::ShaderPaths {
            COMPILED_SHADER_DIR + shaders[2] + ".spv",
            COMPILED_SHADER_DIR + shaders[3] + ".spv"
    };

    const vkb::RenderSystem::ShaderPaths skyboxShaderPaths = vkb::RenderSystem::ShaderPaths {
            COMPILED_SHADER_DIR + shaders[4] + ".spv",
            COMPILED_SHADER_DIR + shaders[5] + ".spv"
    };

    std::vector<std::unique_ptr<vkb::Buffer>> uniformBuffers;
    UniformBufferObject ubo{
        .radius = 0.05f,
        .screenHeight = HEIGHT,
        .screenWidth = WIDTH,
    };
    vkb::DrawableObject plane{vkb::Model::createModelFromFile(device, "../Models/quadXZ1.obj", true),
                              std::make_shared<vkb::Texture>(device, "../textures/coral_reef_texture.jpg")};
    std::shared_ptr<vkb::Texture> rockTex = std::make_shared<vkb::Texture>(device, "../textures/rock_tex.png");

    std::vector<RigidObject> rigidObjects{};
    std::vector<std::string> rigidObjectsNames{};
    uint32_t selectedObj = 0;

    vkb::CubeMapModel skybox{device, {
            "../textures/skybox/right.jpg",
            "../textures/skybox/left.jpg",
            "../textures/skybox/bottom.jpg",
            "../textures/skybox/top.jpg",
            "../textures/skybox/front.jpg",
            "../textures/skybox/back.jpg",
    }, true};
    vkb::RenderSystem skyboxSystem{device};
    vkb::RenderSystem skyboxTexSystem{device};
    vkb::RenderSystem defaultSystem{device};
    vkb::RenderSystem lineSystem{device};
    std::vector<VkDescriptorSet> planeDescriptorSets;
    std::vector<VkDescriptorSet> rigidObjDescriptorSets;
    std::vector<VkDescriptorSet> lineDescriptorSets;
    std::vector<VkDescriptorSet> skyboxDescriptorSets;

    vkb::CameraMovementController cameraController;
    vkb::Camera camera{};

    std::function<std::string(const std::string&)> transformFunc = [this](const std::string& shader){return COMPILED_SHADER_DIR + shader + ".spv";};

    std::ranges::transform_view<std::ranges::take_view<std::ranges::ref_view<const std::vector<std::string>>>, std::function<std::string(const std::string&)>>
            flipRendererShaderPaths = shaders |
                                        std::ranges::views::take(pressureSolverStartIdx) |
                                        std::ranges::views::transform(transformFunc);

    FlipRenderer flipRenderer {
        device,
        {flipRendererShaderPaths.begin(), flipRendererShaderPaths.end()},
        renderer.getSwapChainExtent()
    };

    std::ranges::transform_view<std::ranges::drop_view<std::ranges::ref_view<const std::vector<std::string>>>, std::function<std::string(const std::string&)>>
            simulationShaderPaths = shaders |
                                    std::ranges::views::drop(computeShaderStartIdx) |
                                    std::ranges::views::transform(transformFunc);

    std::ranges::transform_view<std::ranges::take_view<std::ranges::drop_view<std::ranges::ref_view<const std::vector<std::string>>>>, std::function<std::string(const std::string&)>>
            pressureSolverShaderPaths = shaders |
                                    std::ranges::views::drop(pressureSolverStartIdx) |
                                    std::ranges::views::take(computeShaderStartIdx - pressureSolverStartIdx) |
                                    std::ranges::views::transform(transformFunc);

    glm::vec3 dimensions = glm::vec3(9.0f, 10.0f, 7.0f);
    FlipSolver flipSolver {
        device,
        {simulationShaderPaths.begin(), simulationShaderPaths.end()},
        {pressureSolverShaderPaths.begin(), pressureSolverShaderPaths.end()},
        dimensions,
        0.175f
    };
    bool showParticles = true;
    bool renderSkybox = true;

    // render grid
    std::vector<Line> gridLines{};
    glm::vec3 gridColor{0.02f};
    bool showGrid = false;
    std::unique_ptr<vkb::Buffer> gridLinesBuffer;

    float gpuTime = 0, cpuTime = 0;
    bool activateTimer = false, paused = false, singleStep = false, controlMode = false;
    bool isLoadWindowOpen = false, isSaveWindowOpen = false;
    std::vector<std::string> presets;
    std::string saveFileName;


    void onCreate() override;
    void initializeObjects(bool start = true);
    void initializeGridLines();
    void createBuffers();
    void mainLoop(float deltaTime) override;
    void onResize(int width, int height) override;
    void renderObjects();
    void drawGrid(VkCommandBuffer commandBuffer);
    void applyGridLineColor();
    void updateBuffers(uint32_t frameIndex);
    void showImGui();

};

#endif //VULKANFLUIDSIMULATION_FLIPCPU2DSIM_H
