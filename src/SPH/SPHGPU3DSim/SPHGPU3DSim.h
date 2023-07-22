//
// Created by luc on 01/06/23.
//

#ifndef VULKANFLUIDSIMULATION_SPHGPU3DSIM_H
#define VULKANFLUIDSIMULATION_SPHGPU3DSIM_H

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
#include "../../lib/graphicsDataStructures/GpuSpatialGridHandler.h"


class SPHGPU3DSim: public vkb::VulkanApp {
public:
    SPHGPU3DSim(int width, int height, const std::string &appName, vkb::Device::PhysicalDeviceType type = vkb::Device::NVIDIA):
            VulkanApp(width, height, appName, type) {}

    void compileShaders();

private:
    const std::string SHADER_DIR = std::string("../src/SPH/SPHGPU3DSim/Shaders/");
    const std::string COMPILED_SHADER_DIR = std::string("../src/SPH/SPHGPU3DSim/Shaders/bin/");


    uint32_t INSTANCE_COUNT = 27000;

    const std::vector<std::string> shaders = {
            "default.vert",
            "default.frag",
            "point_particle.vert",
            "point_particle.frag",
            "calculate_forces.comp",
            "integrate.comp",
            "calculate_density_pressure.comp",
            "reset_grid.comp",
            "insert_particles.comp",
            "scan.comp",
            "scan_add.comp",
            "sort_particles.comp",
    };

    const std::string planeModelPath = "../Models/quadXZ1.obj";
    const vkb::RenderSystem::ShaderPaths shaderPaths = vkb::RenderSystem::ShaderPaths {
            COMPILED_SHADER_DIR + shaders[0] + ".spv",
            COMPILED_SHADER_DIR + shaders[1] + ".spv"
    };

    const vkb::RenderSystem::ShaderPaths particleShaderPaths = vkb::RenderSystem::ShaderPaths {
            COMPILED_SHADER_DIR + shaders[2] + ".spv",
            COMPILED_SHADER_DIR + shaders[3] + ".spv"
    };

    struct Particle {
        alignas(16) glm::vec3 position{};
        alignas(16) glm::vec3 velocity{};
        alignas(16) glm::vec3 force{};
        alignas(16) glm::vec3 color{};
        float density{}, pressure{}, scale{};
    };

    struct UniformBufferObject {
        alignas(16) glm::mat4 viewProj;
        alignas(16) glm::vec3 cameraPos;
        alignas(16) glm::vec3 lightDir = glm::vec3(1.0f, -1.0f, 0.0f);
        float radius;
    };

    struct ComputeUniformBufferObject {
        float deltaTime = 1/60.0f;
        alignas(16) glm::vec3 BOUNDARY_SIZE = glm::vec3(75.0f);
        float planeY = 0.0f;

        alignas(16) glm::vec3 G{0.0f, -10.0f, 0.0f};   // external (gravitational) forces
        float REST_DENS = 300.f;  // rest density
        float GAS_CONST = 2000.f; // const for equation of state
        float H = 1.5f;           // kernel radius
        float HSQ = H * H;        // radius^2 for optimization
        float MASS = 2.5f;        // assume all particles have the same mass
        float VISC = 200.f;       // viscosity constant
        float DT = 0.0007f;       // integration timestep
        // smoothing kernels defined in Müller and their gradients
        // adapted to 2D per "SPH Based Shallow Water Simulation" by Solenthaler et al.
        float POLY6 = 4.f / (glm::pi<float>() * H*H*H*H*H*H*H*H);

        float SPIKY_GRAD = -10.f / (glm::pi<float>() * H*H*H*H*H);
        float VISC_LAP = 40.f / (glm::pi<float>() * H*H*H*H*H);
        // simulation parameters
        float EPS = H; // boundary epsilon

        float BOUND_DAMPING = -0.5f;
        uint numParticles = 0;
        uint32_t GRID_SIZE = 0;
    };
    glm::ivec2 numParticlesXZ = glm::ivec2(int(std::cbrt(INSTANCE_COUNT)));

    vkb::DrawableObject plane{vkb::Model::createModelFromFile(device, planeModelPath)};

    std::vector<std::unique_ptr<vkb::Buffer>> graphicsUniformBuffers;
    UniformBufferObject gUbo{};
    vkb::RenderSystem defaultSystem{device};
    std::vector<VkDescriptorSet> defaultDescriptorSets;
    vkb::RenderSystem particleSystem{device};


    std::vector<Particle> particles{};
    std::vector<std::unique_ptr<vkb::Buffer>> particleBuffers{2};
    u_char computeFrameIdx = 0;

    std::unique_ptr<vkb::Buffer> computeUniformBuffer;
    ComputeUniformBufferObject cUbo{};
    vkb::ComputeShaderHandler computeHandler{device};
    std::array<VkDescriptorSet, 2> computeDescriptorSets = {nullptr};

    vkb::ComputeSystem calculateForcesComputeSystem{device, COMPILED_SHADER_DIR + shaders[4] + ".spv"};
    vkb::ComputeSystem integrateComputeSystem{device, COMPILED_SHADER_DIR + shaders[5] + ".spv"};
    vkb::ComputeSystem calculateDensityPressureComputeSystem{device, COMPILED_SHADER_DIR + shaders[6] + ".spv"};

    vkb::GpuSpatialGridHandler gridHandler{
        device, 256,
        COMPILED_SHADER_DIR + shaders[7] + ".spv",
        COMPILED_SHADER_DIR + shaders[8] + ".spv",
        COMPILED_SHADER_DIR + shaders[9] + ".spv",
        COMPILED_SHADER_DIR + shaders[10] + ".spv",
        COMPILED_SHADER_DIR + shaders[11] + ".spv"
        };

    vkb::Camera camera{};
    vkb::CameraMovementController cameraController{};

    glm::vec3 initialPos = {cUbo.EPS, cUbo.EPS, cUbo.EPS};
    float drawTime = 0, cpuTime = 0, computeTime = 0, gravityFactor = 50.0f;
    bool activateTimer = false, controlMode = false;

    void onCreate() override;
    void initializeObjects(bool activateRandomOffsets);
    void createComputeDescriptorSets(vkb::DescriptorSetLayout &layout);
    void createUniformBuffers();
    void mainLoop(float deltaTime) override;
    void renderObjects();
    void updateSimulation();
    void updateUniformBuffers(uint32_t frameIndex, float deltaTime);
    void showImGui();

};

#endif //VULKANFLUIDSIMULATION_SPHGPU3DSIM_H
