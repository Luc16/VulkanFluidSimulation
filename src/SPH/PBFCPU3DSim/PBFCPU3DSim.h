//
// Created by luc on 13/12/22.
//

#ifndef VULKANBASE_PBFCPU3DSIM_H
#define VULKANBASE_PBFCPU3DSIM_H

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

class PBFCPU3DSim: public vkb::VulkanApp {
public:
    PBFCPU3DSim(int width, int height, const std::string &appName, vkb::Device::PhysicalDeviceType type = vkb::Device::INTEL):
    VulkanApp(width, height, appName, type) {}

public:
    std::string DIR = std::string("../src/SPH/PBFCPU3DSim/");

    uint32_t INSTANCE_COUNT = 10000;
    static constexpr glm::vec3 G{0.0f, -9.8f, 0.0f};   // external (gravitational) forces
    float REST_DENS = 8.0f;  // rest density
    static constexpr float H = 1.5f;           // kernel radius
    static constexpr float HSQ = H * H;        // radius^2 for optimization
    float MASS = 5.0f;        // assume all particles have the same mass
    float VISC = 0.8f;       // viscosity constant
    float DT = 0.010f;       // integration timestep
    float ART_PRESSURE_COEF = 0.1f;
    float VORTICITY_COEF = 0.004f;

    // smoothing kernels defined in Müller and their gradients
    static constexpr float POLY6 = 315.0f / (64.0f * glm::pi<float>() *H*H*H *H*H*H *H*H*H);
    static constexpr float SPIKY_GRAD = 45.0f / (glm::pi<float>() * H*H*H *H*H*H);

    float CFM = 4.0f;

    // simulation parameters
    static constexpr float EPS = H; // boundary epsilon
    glm::vec3 BOUNDARY_SIZE = glm::vec3(50.0f) ;
    glm::ivec2 numParticlesXZ = glm::ivec2(int(std::cbrt(INSTANCE_COUNT)));
    static constexpr uint32_t numThreads = 12;
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
        alignas(16) glm::vec3 position, velocity, posCorrection, predPos, vorticity;
        float density, lambda;
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

    // simulation functions
    std::function<void(uint32_t,uint32_t)> predictPositionsThreaded;
    std::function<void(uint32_t,uint32_t)> computeDensityThreaded;
    std::function<void(uint32_t,uint32_t)> computeLambdaThreaded;
    std::function<void(uint32_t,uint32_t)> computePositionCorrectionThreaded;
    std::function<void(uint32_t,uint32_t)> correctPositionThreaded;
    std::function<void(uint32_t,uint32_t)> updateVelocitiesThreaded;
    std::function<void(uint32_t,uint32_t)> applyXsphViscosityAndComputeVorticity;
    std::function<void(uint32_t,uint32_t)> applyVorticity;
    std::function<void(uint32_t,uint32_t)> updatePositionThreaded;

    glm::vec3 initialPos = {EPS, EPS, EPS};
    float gravityFactor = 40.f;
    float colorUpdate = 0.008f, densColorThreshold = 0.0f;//1.01f;
    float gpuTime = 0, cpuTime = 0;
    bool activateTimer = false, controlMode = false;

    void threadedCall(const std::function<void(uint32_t,uint32_t)>& func);

    void onCreate() override;
    void createFunctions();
    void initializeObjects();
    void createInstances(bool activateRandomOffsets);
    void createUniformBuffers();
    void mainLoop(float deltaTime) override;
    void updateUniformBuffer(uint32_t frameIndex);
    void showImGui();
    void updateParticles(float deltaTime);


};


#endif //VULKANBASE_PBFCPU3DSIM_H
