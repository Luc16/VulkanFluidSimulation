//
// Created by luc on 01/06/23.
//

#ifndef VULKANFLUIDSIMULATION_PBFGPU3DSIM_H
#define VULKANFLUIDSIMULATION_PBFGPU3DSIM_H

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
#include "PbfGpuSpatialGridHandler.h"
#include "OffscreenPass.h"


class PBFGPU3DSim: public vkb::VulkanApp {
public:
    PBFGPU3DSim(int width, int height, const std::string &appName, vkb::Device::PhysicalDeviceType type = vkb::Device::NVIDIA):
            VulkanApp(width, height, appName, type) {}

    void compileShaders();

private:
    const std::string SHADER_DIR = std::string("../src/SPH/PBFGPU3DSim/Shaders/");
    const std::string RENDER_SHADER_DIR = SHADER_DIR + "/rendering/";
    const std::string GRID_SHADER_DIR = SHADER_DIR + "/grid/";
    const std::string SIMULATIONS_SHADER_DIR = SHADER_DIR + "/simulation/";
    const std::string COMPILED_SHADER_DIR = std::string("../src/SPH/PBFGPU3DSim/Shaders/bin/");


    uint32_t INSTANCE_COUNT = 200'000;
    static constexpr uint32_t MAX_PARTICLES = 1'000'000;
    static constexpr float MAX_BOUND = 1000.0f;
    uint32_t jacobiIterations = 3;
    uint32_t GRID_SIZE = 0;

    static constexpr uint32_t gridShaderStartIdx = 14;
    static constexpr uint32_t computeShaderStartIdx = 19;
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
            "quad.vert",
            "quad.frag",
            "reset_grid.comp",
            "insert_particles.comp",
            "scan.comp",
            "scan_add.comp",
            "sort_particles.comp",
            "predict_positions.comp",
            "compute_density.comp",
            "compute_lambda.comp",
            "compute_position_correction.comp",
            "correct_positions.comp",
            "update_velocities.comp",
            "apply_visc_and_compute_vort.comp",
            "apply_vort_and_update_pos.comp",
    };

    const std::string planeModelPath = "../Models/quadXZ1.obj";
    const std::string planeTexPath = "../textures/chess_tex.png";
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

    struct SimulationKernel {
        vkb::ComputeSystem computeSystem;
        std::array<VkDescriptorSet, 2> descSets{};
        vkb::DescriptorSetLayout layout;

        void createPipeline(){
            computeSystem.createPipelineWithLayout(layout.descriptorSetLayout());
        }
        void bindAndDispatch(VkCommandBuffer commandBuffer, uint32_t compFrameIdx, uint32_t x, uint32_t y, uint32_t z){
            computeSystem.bindAndDispatch(commandBuffer, &descSets[compFrameIdx], x, y, z);
        }
    };

    // these vec4s can be read as vec3 in shader because of the 16 byte spacing
    struct ParticleData {
        std::vector<glm::vec4> position{}, velocity{}, correction{}, predPos{}, vorticity{};
        std::vector<float> density{}, lambda{};
        std::vector<uint32_t> idx{};

        void resize(size_t newSize) {
            position.resize(newSize);
            velocity.resize(newSize);
            correction.resize(newSize);
            predPos.resize(newSize);
            vorticity.resize(newSize);
            density.resize(newSize);
            lambda.resize(newSize);
            idx.resize(newSize);
        }
    };

    struct UniformBufferObject {
        alignas(16) glm::mat4 view;
        alignas(16) glm::mat4 proj;
        alignas(16) glm::vec3 lightDir = glm::vec3(1.0f, -1.0f, 0.0f);
        float radius;
        float screenHeight;
        float screenWidth;
        float tanHalfFov = std::tan(glm::radians(50.0f)/2);
        uint32_t renderType = 4;
        float zNear = 0.1f;
        float zFar = 500.0f;
        int filterRadius = 5;
        float blurScale = 0.2;
        float blurDepthFalloff = 8;
    };

    struct ComputeUniformBufferObject {
        alignas(16) glm::vec3 BOUNDARY_SIZE = glm::vec3(150.0f);
        alignas(16) glm::vec3 G = glm::vec3(0.0f, -9.8f, 0.0f);

        float planeY = 0.0f;
        float REST_DENS = 20.0f;  // rest density
        float H = 1.5f;           // kernel radius
        float HSQ = H * H;        // radius^2 for optimization
        float MASS = 5.0f;        // assume all particles have the same mass
        float VISC = 0.8f;       // viscosity constant
        float DT = 0.01666f/2;       // integration timestep
        float ART_PRESSURE_COEF = 0.1f;
        float VORTICITY_COEF = 0.0004f;
        uint numParticles = 0;
        float POLY6 = 315.0f / (64.0f * glm::pi<float>() *H*H*H *H*H*H *H*H*H);
        float SPIKY_GRAD = 45.0f / (glm::pi<float>() * H*H*H *H*H*H);

        float CFM = 0.5f;
        float EPS = H; // boundary epsilon
        uint32_t GRID_SIZE = 0;
    };
//    struct ComputeUniformBufferObject {
//        alignas(16) glm::vec3 BOUNDARY_SIZE = glm::vec3(8.0f);
//        alignas(16) glm::vec3 G = glm::vec3(0.0f, -9.8f, 0.0f);
//
//        float planeY = 0.0f;
//        float REST_DENS = 6378.0f;  // rest density
//        float H = 0.1f;           // kernel radius
//        float HSQ = H * H;        // radius^2 for optimization
//        float MASS = 1.0f;        // assume all particles have the same mass
//        float VISC = 0.01f;       // viscosity constant
////        float DT = 0.0083f;       // integration timestep
//        float DT = 0.00005f;       // integration timestep
//        float ART_PRESSURE_COEF = 0.0001f;
//        float VORTICITY_COEF = 0.0004f;
//        uint numParticles = 0;
//        float POLY6 = 315.0f / (64.0f * glm::pi<float>() *H*H*H *H*H*H *H*H*H);
//        float SPIKY_GRAD = 45.0f / (glm::pi<float>() * H*H*H *H*H*H);
//
//        float CFM = 600.0f;
//        float EPS = H; // boundary epsilon
//        uint32_t GRID_SIZE = 0;
//    };

    vkb::DrawableObject plane{vkb::Model::createModelFromFile(device, planeModelPath), std::make_shared<vkb::Texture>(device, planeTexPath)};

    std::vector<std::unique_ptr<vkb::Buffer>> graphicsUniformBuffers;
    UniformBufferObject gUbo{};
    vkb::RenderSystem defaultSystem{device};
    std::vector<VkDescriptorSet> defaultDescriptorSets;
    std::vector<VkDescriptorSet> simulationDescriptorSets;
    std::vector<VkDescriptorSet> normalDescriptorSets;
    std::vector<VkDescriptorSet> debugDescriptorSets;
    vkb::RenderSystem particleSystem{device};
    // rendering
    vkb::OffscreenPass depthPass{device, renderer.getSwapChainExtent(), true};
    vkb::OffscreenPass thicknessPass{device, renderer.getSwapChainExtent()};
    vkb::OffscreenPass normalsPass{device, renderer.getSwapChainExtent()};
    vkb::OffscreenPass smoothPass{device, renderer.getSwapChainExtent(), true};

    vkb::RenderSystem debugRenderSystem{device};


    ParticleData particles{};
    std::array<std::unique_ptr<vkb::Buffer>, 2> positionBuffers;
    std::array<std::unique_ptr<vkb::Buffer>, 2> velocityBuffers;
    std::array<std::unique_ptr<vkb::Buffer>, 2> predPosBuffers;
    std::unique_ptr<vkb::Buffer> correctionBuffer;
    std::unique_ptr<vkb::Buffer> vorticityBuffer;
    std::unique_ptr<vkb::Buffer> densityBuffer;
    std::unique_ptr<vkb::Buffer> lambdaBuffer;
    std::unique_ptr<vkb::Buffer> gridIdxBuffer;

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

    std::unique_ptr<vkb::Buffer> computeUniformBuffer;
    ComputeUniformBufferObject cUbo{};
    vkb::ComputeShaderHandler computeHandler{device};

    SimulationKernel predictPositionKernel {
            .computeSystem{device, COMPILED_SHADER_DIR + shaders[computeShaderStartIdx] + ".spv"},
            .layout = vkb::DescriptorSetLayout::Builder(device)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // pos
                    .addBinding({2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // vel
                    .addBinding({3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // predPos
                    .build()
    };

    SimulationKernel densityKernel {
            .computeSystem{device, COMPILED_SHADER_DIR + shaders[computeShaderStartIdx + 1] + ".spv"},
            .layout = vkb::DescriptorSetLayout::Builder(device)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // grid
                    .addBinding({2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // predPos
                    .addBinding({3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // density
                    .addBinding({4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // grid idx
                    .build()
    };

    SimulationKernel lambdaKernel {
            .computeSystem{device, COMPILED_SHADER_DIR + shaders[computeShaderStartIdx + 2] + ".spv"},
            .layout = vkb::DescriptorSetLayout::Builder(device)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // grid
                    .addBinding({2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // predPos
                    .addBinding({3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // lambda
                    .addBinding({4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // density
                    .addBinding({5, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // grid idx
                    .build()
    };

    SimulationKernel posCorrectionKernel {
            .computeSystem{device, COMPILED_SHADER_DIR + shaders[computeShaderStartIdx + 3] + ".spv"},
            .layout = vkb::DescriptorSetLayout::Builder(device)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // grid
                    .addBinding({2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // predPos
                    .addBinding({3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // correction
                    .addBinding({4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // lambda
                    .addBinding({5, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // grid idx
                    .build()
    };

    SimulationKernel correctPositionsKernel {
        .computeSystem{device, COMPILED_SHADER_DIR + shaders[computeShaderStartIdx + 4] + ".spv"},
        .layout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
            .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // predPos
            .addBinding({2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // correction
            .build()
    };

    SimulationKernel updateVelocitiesKernel {
            .computeSystem{device, COMPILED_SHADER_DIR + shaders[computeShaderStartIdx + 5] + ".spv"},
            .layout = vkb::DescriptorSetLayout::Builder(device)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // pos
                    .addBinding({2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // vel
                    .addBinding({3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // predPos
                    .build()
    };

    SimulationKernel applyViscAndComputeVortKernel {
            .computeSystem{device, COMPILED_SHADER_DIR + shaders[computeShaderStartIdx + 6] + ".spv"},
            .layout = vkb::DescriptorSetLayout::Builder(device)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // grid
                    .addBinding({2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // predPos
                    .addBinding({3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // vel
                    .addBinding({4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // correction
                    .addBinding({5, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // vorticity
                    .addBinding({6, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // grid idx
                    .build()
    };

    SimulationKernel applyVortAndUpdatePos {
            .computeSystem{device, COMPILED_SHADER_DIR + shaders[computeShaderStartIdx + 7] + ".spv"},
            .layout = vkb::DescriptorSetLayout::Builder(device)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // grid
                    .addBinding({2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // predPos
                    .addBinding({3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // pos
                    .addBinding({4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // vel
                    .addBinding({5, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // correction
                    .addBinding({6, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // vorticity
                    .addBinding({7, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // grid idx
                    .build()
    };




    vkb::Camera camera{};
    vkb::CameraMovementController cameraController{};

    glm::ivec2 numParticlesXZ = glm::ivec2(int(std::cbrt(INSTANCE_COUNT)));
    float particleSpacing = cUbo.H*(1 + 0.13333f);
    float particleVerticalSpacing = cUbo.H*(1 + 0.13333f);
    glm::vec4 initialPos = {cUbo.EPS, cUbo.EPS, cUbo.EPS, 0};
    float drawTime = 0, cpuTime = 0, computeTime = 0, gravityFactor = 1.0f;
    bool activateTimer = false, controlMode = false, objectsInitialized = false, pausedSimulation = false, debugScene = false;

    void onCreate() override;
    void initializeObjects(bool activateRandomOffsets);
    void createComputeDescriptorSets();
    void createUniformBuffers();
    void onResize(int width, int height) override;
    void mainLoop(float deltaTime) override;
    void renderObjects(VkCommandBuffer commandBuffer);
    void updateSimulation();
    void updateUniformBuffers(uint32_t frameIndex, float deltaTime);
    void showImGui();

};

#endif //VULKANFLUIDSIMULATION_PBFGPU3DSIM_H
