//
// Created by luc on 01/06/23.
//

#ifndef VULKANFLUIDSIMULATION_PBFGPU3DSIM_H
#define VULKANFLUIDSIMULATION_PBFGPU3DSIM_H

#define GLM_GTX_norm
#include <sstream>
#include <filesystem>
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
#include "PbfGpuSpatialGridHandler.h"
#include "../../lib/simulations/OffscreenPass.h"
#include "../../lib/CubeMapModel.h"
#include "particles_and_ubos.h"
#include "RigidObject.h"
#include "PbfInitializer.h"
#include "PBFSceneManager.h"


class PBFGPU3DSim: public vkb::VulkanApp {
public:
    PBFGPU3DSim(int width, int height, const std::string &appName, vkb::Device::PhysicalDeviceType type = vkb::Device::NVIDIA):
            VulkanApp(width, height, appName, type) {}

    void compileShaders();

private:

    const std::string SIM_DIR = std::string("../src/SPH/PBFGPU3DSim/");
    const std::string SHADER_DIR = SIM_DIR + "Shaders/";
    const std::string RENDER_SHADER_DIR = SHADER_DIR + "rendering/";
    const std::string GRID_SHADER_DIR = SHADER_DIR + "grid/";
    const std::string SIMULATIONS_SHADER_DIR = SHADER_DIR + "simulation/";
    const std::string COMPILED_SHADER_DIR = SHADER_DIR + "bin/";
    const std::string PRESET_DIR = SIM_DIR + "presets/";


    uint32_t NUM_PARTICLES = 200'000;
    uint32_t NUM_RIGID_PARTICLES = 0;
    uint32_t NUM_FLUID_PARTICLES = NUM_PARTICLES - NUM_RIGID_PARTICLES;
    static constexpr uint32_t MAX_PARTICLES = 1'000'000;
    static constexpr float MAX_BOUND = 100.0f;
    uint32_t jacobiIterations = 4;
    uint32_t substeps = 1;
    uint32_t gaussPartition = 2;
    bool test = false;
    int blurIterations = 2;
    uint32_t GRID_SIZE = 0;
    std::string_view curFile{"dam_break.json"};

    static constexpr uint32_t gridShaderStartIdx = 16;
    static constexpr uint32_t computeShaderStartIdx = 21;
    const std::vector<std::string> shaders = {
            "default.vert",
            "default.frag",
            "point_particle.vert",
            "point_particle.frag",
            "ssf_depth.vert",
            "ssf_depth.frag",
            "ssf_thickness.vert",
            "ssf_thickness.frag",
            "ssf_calculate_normals.vert",
            "ssf_calculate_normals.frag",
            "ssf_smooth.vert",
            "ssf_smooth.frag",
            "ssf_shading.vert",
            "ssf_shading.frag",
            "skybox.vert",
            "skybox.frag",
            "reset_grid.comp",
            "insert_particles.comp",
            "scan.comp",
            "scan_add.comp",
            "sort_particles.comp",
            "predict_positions.comp",
            "compute_lambda.comp",
            "compute_position_correction.comp",
            "update_velocities.comp",
            "apply_visc_and_compute_vort.comp",
            "apply_vort.comp",
    };

    const vkb::RenderSystem::ShaderPaths shaderPaths = vkb::RenderSystem::ShaderPaths {
            COMPILED_SHADER_DIR + shaders[0] + ".spv",
            COMPILED_SHADER_DIR + shaders[1] + ".spv"
    };

    const vkb::RenderSystem::ShaderPaths particleShaderPaths = vkb::RenderSystem::ShaderPaths {
            COMPILED_SHADER_DIR + shaders[2] + ".spv",
            COMPILED_SHADER_DIR + shaders[3] + ".spv"
    };

    const vkb::RenderSystem::ShaderPaths depthShaderPaths = vkb::RenderSystem::ShaderPaths {
            COMPILED_SHADER_DIR + shaders[4] + ".spv",
            COMPILED_SHADER_DIR + shaders[5] + ".spv",
    };

    const vkb::RenderSystem::ShaderPaths thicknessShaderPaths = vkb::RenderSystem::ShaderPaths {
            COMPILED_SHADER_DIR + shaders[6] + ".spv",
            COMPILED_SHADER_DIR + shaders[7] + ".spv",
    };

    const vkb::RenderSystem::ShaderPaths normalsShaderPaths = vkb::RenderSystem::ShaderPaths {
            COMPILED_SHADER_DIR + shaders[8] + ".spv",
            COMPILED_SHADER_DIR + shaders[9] + ".spv",
    };

    const vkb::RenderSystem::ShaderPaths smoothShaderPaths = vkb::RenderSystem::ShaderPaths {
            COMPILED_SHADER_DIR + shaders[10] + ".spv",
            COMPILED_SHADER_DIR + shaders[11] + ".spv",
    };

    const vkb::RenderSystem::ShaderPaths quadShaderPaths = vkb::RenderSystem::ShaderPaths {
            COMPILED_SHADER_DIR + shaders[12] + ".spv",
            COMPILED_SHADER_DIR + shaders[13] + ".spv",
    };

    const vkb::RenderSystem::ShaderPaths skyboxShaderPaths = vkb::RenderSystem::ShaderPaths {
            COMPILED_SHADER_DIR + shaders[14] + ".spv",
            COMPILED_SHADER_DIR + shaders[15] + ".spv",
    };


    vkb::DrawableObject plane{vkb::Model::createModelFromFile(device, "../Models/quadXZ1.obj", true),
                              std::make_shared<vkb::Texture>(device, "../textures/coral_reef_texture.jpg")};
    std::shared_ptr<vkb::Texture> rockTex = std::make_shared<vkb::Texture>(device, "../textures/rock_tex.png");


    std::vector<std::unique_ptr<vkb::Buffer>> graphicsUniformBuffers;
    UniformBufferObject gUbo{};
    vkb::RenderSystem defaultSystem{device};

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

    std::vector<VkDescriptorSet> defaultDescriptorSets;
    std::vector<VkDescriptorSet> rockDescriptorSets;
    std::vector<VkDescriptorSet> simulationDescriptorSets;
    std::vector<VkDescriptorSet> smooth1DescriptorSets;
    std::vector<VkDescriptorSet> smooth2DescriptorSets;
    std::array<std::vector<VkDescriptorSet>, 3> shadingDescriptorSets;
    std::vector<VkDescriptorSet> skyboxDescriptorSets;
    vkb::RenderSystem particleSystem{device};

    // rendering with screen space fluids
    vkb::OffscreenPass depthPass{device, renderer.getSwapChainExtent(), true};
    vkb::OffscreenPass thicknessPass{device, renderer.getSwapChainExtent()};
    vkb::OffscreenPass scenePass{device, renderer.getSwapChainExtent()};
    vkb::OffscreenPass smoothPass{device, renderer.getSwapChainExtent(), true, true};

    vkb::RenderSystem shadingRenderSystem{device};


    ParticleData particles{};
    std::unique_ptr<vkb::Buffer> idxBuffer; // index buffer for rendering
    std::array<std::unique_ptr<vkb::Buffer>, 2> positionBuffers;
    std::array<std::unique_ptr<vkb::Buffer>, 2> velocityBuffers;
    std::array<std::unique_ptr<vkb::Buffer>, 2> predPosBuffers;
    std::array<std::unique_ptr<vkb::Buffer>, 2> particleTypeBuffers; // 0 for fluid 1 for solid
    std::array<std::unique_ptr<vkb::Buffer>, 2> deltaPBuffers;
    std::unique_ptr<vkb::Buffer> vorticityBuffer;
    std::unique_ptr<vkb::Buffer> densityBuffer;
    std::unique_ptr<vkb::Buffer> lambdaBuffer;
    std::unique_ptr<vkb::Buffer> gridIdxBuffer;
    std::unique_ptr<vkb::Buffer> avgDensBuffer;

    std::array<std::vector<std::pair<VkBuffer, VkDeviceSize>>, 2> particleBarrierData;

    vkb::PbfGpuSpatialGridHandler gridHandler{
            device, 256,
            COMPILED_SHADER_DIR + shaders[gridShaderStartIdx] + ".spv",
            COMPILED_SHADER_DIR + shaders[gridShaderStartIdx + 1] + ".spv",
            COMPILED_SHADER_DIR + shaders[gridShaderStartIdx + 2] + ".spv",
            COMPILED_SHADER_DIR + shaders[gridShaderStartIdx + 3] + ".spv",
            COMPILED_SHADER_DIR + shaders[gridShaderStartIdx + 4] + ".spv"
    };

    u_char computeFrameIdx = 0;

    std::vector<std::unique_ptr<vkb::Buffer>> solverUniformBuffers;
    std::unique_ptr<vkb::Buffer> computeUniformBuffer;
    ComputeUniformBufferObject cUbo{};
    vkb::ComputeShaderHandler computeHandler{device};

    vkb::SimulationKernel predictPositionKernel {
            .computeSystem{device, COMPILED_SHADER_DIR + shaders[computeShaderStartIdx] + ".spv"},
            .descSets = std::vector<VkDescriptorSet>(2),
            .layout = vkb::DescriptorSetLayout::Builder(device)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // pos, vel, predPos, type
                    .addSameTypeBindings(1, 4,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel lambdaKernel {
            .computeSystem{device, COMPILED_SHADER_DIR + shaders[computeShaderStartIdx + 1] + ".spv"},
            .descSets = std::vector<VkDescriptorSet>(2*gaussPartition),
            .layout = vkb::DescriptorSetLayout::Builder(device)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    .addBinding({1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // grid, predPos, lambda, density, grid idx, type, avg
                    .addSameTypeBindings(2, 8,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel posCorrectionKernel {
            .computeSystem{device, COMPILED_SHADER_DIR + shaders[computeShaderStartIdx + 2] + ".spv"},
            .descSets = std::vector<VkDescriptorSet>(2*gaussPartition),
            .layout = vkb::DescriptorSetLayout::Builder(device)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    .addBinding({1, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // grid, predPos, lambda, grid idx, type, corr
                    .addSameTypeBindings(2, 7,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel updateVelocitiesKernel {
            .computeSystem{device, COMPILED_SHADER_DIR + shaders[computeShaderStartIdx + 3] + ".spv"},
            .descSets = std::vector<VkDescriptorSet>(2),
            .layout = vkb::DescriptorSetLayout::Builder(device)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // pos, vel, predPos, type
                    .addSameTypeBindings(1, 4,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel applyViscAndComputeVortKernel {
            .computeSystem{device, COMPILED_SHADER_DIR + shaders[computeShaderStartIdx + 4] + ".spv"},
            .descSets = std::vector<VkDescriptorSet>(2),
            .layout = vkb::DescriptorSetLayout::Builder(device)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // grid, predPos, vel, vorticity, grid idx, type
                    .addSameTypeBindings(1, 6,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel applyVortAndUpdatePos {
            .computeSystem{device, COMPILED_SHADER_DIR + shaders[computeShaderStartIdx + 5] + ".spv"},
            .descSets = std::vector<VkDescriptorSet>(2),
            .layout = vkb::DescriptorSetLayout::Builder(device)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // grid, predPos, pos, vel, vorticity, grid idx, type
                    .addSameTypeBindings(1, 7,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };


    vkb::Camera camera{};
    vkb::CameraMovementController cameraController{};

    std::array<std::pair<uint32_t, std::string>, 3> rigidObjectTypes = {
            std::make_pair(0, "rockA"), std::make_pair(0, "rockB"), std::make_pair(0, "rockC")
    };
    std::vector<RigidObject> rigidObjects{};
    std::vector<std::string> rigidObjectsNames{};
    uint32_t selectedRigidObj = 0;

    glm::ivec2 numParticlesXZ = glm::ivec2(int(std::cbrt(NUM_PARTICLES)));
    float particleSpacing = cUbo.H*0.56f;
    float particleVerticalSpacing = cUbo.H*0.50f;
    glm::vec4 initialPos = {cUbo.EPS, cUbo.EPS, cUbo.EPS, 0};
    float drawTime = 0, cpuTime = 0, computeTime = 0;

    std::string saveFileName;
    std::vector<std::string> presets;
    uint32_t sourceParticleSum = 0;
    uint32_t initialParticles = 0;
    bool isSaveWindowOpen = false, isLoadWindowOpen = false, disableKeyboardControl = false, isAddWindowOpen = false;
    bool activateTimer = false, controlMode = false, objectsInitialized = false, pausedSimulation = false;
    bool activateWaves = false, showParticles = false, singleStep = false;
    bool activateVisc = true, activateVorticity = true, renderSkybox = true;
    float wallForwardSpeed = 0.4f * cUbo.H / cUbo.DT, wallBackwardSpeed = 0.1f * cUbo.H / cUbo.DT, wallLimit = 0.2f * cUbo.BOUNDARY_SIZE.x, curSpeed = 0.0f;
    uint32_t hardResetFrame = 0;

    void onCreate() override;
    void initializeObjects(bool activateRandomOffsets);
    void createComputeDescriptorSets();
    void createUniformBuffers();
    void onResize(int width, int height) override;
    void mainLoop(float deltaTime) override;
    void renderObjects(VkCommandBuffer commandBuffer);
    void updateSimulation();
    void keyboardControl(float deltaTime);
    void updateUniformBuffers(uint32_t frameIndex, float deltaTime);
    void showImGui();
    void addRigidObject(uint32_t type);

    friend class PBFSceneManager;


};

#endif //VULKANFLUIDSIMULATION_PBFGPU3DSIM_H
