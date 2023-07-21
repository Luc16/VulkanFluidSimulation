//
// Created by luc on 13/12/22.
//

#ifndef VULKANBASE_SPHCPU3DSIM_H
#define VULKANBASE_SPHCPU3DSIM_H

#include <sstream>
#include <thread>
#include <barrier>
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
#include "../../lib/graphicsDataStructures/SpatialHash.h"
#include "../../lib/graphicsDataStructures/SpatialGrid.h"

class SPHCPU3DSim: public vkb::VulkanApp {
public:
    SPHCPU3DSim(int width, int height, const std::string &appName, vkb::Device::PhysicalDeviceType type = vkb::Device::INTEL):
    VulkanApp(width, height, appName, type) {}

public:
    std::string DIR = std::string("../src/SPH/SPHCPU3DSim/");

    uint32_t INSTANCE_COUNT = 27000;
    static constexpr glm::vec3 G{0.0f, -10.0f, 0.0f};   // external (gravitational) forces
    static constexpr float REST_DENS = 300.f;  // rest density
    static constexpr float GAS_CONST = 2000.f; // const for equation of state
    static constexpr float H = 1.5f;           // kernel radius
    static constexpr float HSQ = H * H;        // radius^2 for optimization
    static constexpr float MASS = 2.5f;        // assume all particles have the same mass
    static constexpr float VISC = 200.f;       // viscosity constant
    static constexpr float DT = 0.001f;       // integration timestep

    // smoothing kernels defined in Müller and their gradients
    // adapted to 2D per "SPH Based Shallow Water Simulation" by Solenthaler et al.
    static constexpr float POLY6 = 4.f / (glm::pi<float>() * H*H*H*H*H*H*H*H);
    static constexpr float SPIKY_GRAD = -10.f / (glm::pi<float>() * H*H*H*H*H);
    static constexpr float VISC_LAP = 40.f / (glm::pi<float>() * H*H*H*H*H);
    static constexpr float MIN_DENS = MASS * POLY6 * HSQ * HSQ * HSQ;

    // simulation parameters
    static constexpr float EPS = H; // boundary epsilon
    static constexpr float BOUND_DAMPING = -0.5f;
    glm::vec3 BOUNDARY_SIZE = glm::vec3(70.0f) ;
    glm::ivec2 numParticlesXZ = glm::ivec2(int(std::cbrt(INSTANCE_COUNT)));
    static constexpr uint32_t numThreads = 10;
    uint32_t particlesPerThread = INSTANCE_COUNT/numThreads;


    const std::string planeModelPath = "../Models/quadXZ1.obj";
    const vkb::RenderSystem::ShaderPaths shaderPaths = vkb::RenderSystem::ShaderPaths {
            DIR + "Shaders/default.vert.spv",
            DIR + "Shaders/default.frag.spv"
    };

    const vkb::RenderSystem::ShaderPaths particleShaderPaths = vkb::RenderSystem::ShaderPaths {
            DIR + "Shaders/point_particle.vert.spv",
            DIR + "Shaders/point_particle.frag.spv",
    };

    struct UniformBufferObject {
        alignas(16) glm::mat4 viewProj;
        alignas(16) glm::vec3 cameraPos;
        alignas(16) glm::vec3 lightDir = glm::vec3(1.0f, -1.0f, 0.0f);
        float radius = H;
    };

    struct Particle {
        alignas(16) glm::vec3 position, velocity, force;
        float density, pressure;
        alignas(16) glm::vec3 color;
    };

    vkb::DrawableObject plane{vkb::Model::createModelFromFile(device, planeModelPath)};

    std::vector<std::unique_ptr<vkb::Buffer>> uniformBuffers;

    vkb::RenderSystem defaultSystem{device};
    std::vector<VkDescriptorSet> defaultDescriptorSets;
    vkb::RenderSystem particleSystem{device};

    vkb::Camera camera{};
    vkb::CameraMovementController cameraController{};

    std::vector<Particle> particles{};
    std::vector<Particle> sortedParticles{};
    std::unique_ptr<vkb::Buffer> particleBuffer;

    vkb::SpatialGrid grid{};
    std::array<std::jthread, numThreads> threads;

    glm::vec3 initialPos = {EPS, EPS, EPS};
    float gravityFactor = 40.f, sphereRadius = 0.641f;
    float colorUpdate = 0.008f, densColorThreshold = 1.01f;
    float gpuTime = 0, cpuTime = 0;
    bool activateTimer = false, controlMode = false;

    void onCreate() override;
    void initializeObjects();
    void createInstances(bool activateRandomOffsets);
    void createUniformBuffers();
    void mainLoop(float deltaTime) override;
    void updateUniformBuffer(uint32_t frameIndex);
    void showImGui();
    void updateParticles(float deltaTime);

};


#endif //VULKANBASE_SPHCPU3DSIM_H
