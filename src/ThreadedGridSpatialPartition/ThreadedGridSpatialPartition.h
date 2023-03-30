//
// Created by luc on 13/12/22.
//

#ifndef VULKANBASE_INSTANCINGAPP_H
#define VULKANBASE_INSTANCINGAPP_H

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

class ThreadedGridSpatialPartition: public vkb::VulkanApp {
public:
    ThreadedGridSpatialPartition(int width, int height, const std::string &appName, vkb::Device::PhysicalDeviceType type = vkb::Device::INTEL):
    VulkanApp(width, height, appName, type) {}

private:
    uint32_t INSTANCE_COUNT = 1024;
    static constexpr glm::vec3 G{0.0f, -10.0f, 0.0f};   // external (gravitational) forces
    static constexpr float REST_DENS = 300.f;  // rest density
    static constexpr float GAS_CONST = 2000.f; // const for equation of state
    static constexpr float H = 1.5f;           // kernel radius
    static constexpr float HSQ = H * H;        // radius^2 for optimization
    static constexpr float MASS = 2.5f;        // assume all particles have the same mass
    static constexpr float VISC = 200.f;       // viscosity constant
    static constexpr float DT = 0.0007f;       // integration timestep

    // smoothing kernels defined in Müller and their gradients
    // adapted to 2D per "SPH Based Shallow Water Simulation" by Solenthaler et al.
    static constexpr float POLY6 = 4.f / (glm::pi<float>() * H*H*H*H*H*H*H*H);
    static constexpr float SPIKY_GRAD = -10.f / (glm::pi<float>() * H*H*H*H*H);
    static constexpr float VISC_LAP = 40.f / (glm::pi<float>() * H*H*H*H*H);

    // simulation parameters
    static constexpr float EPS = H; // boundary epsilon
    static constexpr float BOUND_DAMPING = -0.5f;

    const std::string planeModelPath = "../Models/quadXZ1.obj";
    const vkb::RenderSystem::ShaderPaths shaderPaths = vkb::RenderSystem::ShaderPaths {
            "../src/ThreadedGridSpatialPartition/Shaders/default.vert.spv",
            "../src/ThreadedGridSpatialPartition/Shaders/default.frag.spv"
    };

    const std::string sphereModelPath = "../Models/sphere.obj";
    const vkb::RenderSystem::ShaderPaths instanceShaderPaths = vkb::RenderSystem::ShaderPaths {
            "../src/ThreadedGridSpatialPartition/Shaders/instancing.vert.spv",
            "../src/ThreadedGridSpatialPartition/Shaders/instancing.frag.spv",
    };

    struct UniformBufferObject {
        alignas(16) glm::mat4 view;
        alignas(16) glm::mat4 proj;
        alignas(16) glm::vec3 lightDirection = glm::normalize(glm::vec3(1.0f, 1.0f, 0.0f));
    };

    struct InstanceData {
        alignas(16) glm::vec3 position, velocity, force;
        float density, pressure;
        alignas(16) glm::vec3 color;
        float scale;
    };

    vkb::DrawableObject plane{vkb::Model::createModelFromFile(device, planeModelPath)};

    std::vector<std::unique_ptr<vkb::Buffer>> uniformBuffers;

    vkb::RenderSystem defaultSystem{device};
    std::vector<VkDescriptorSet> defaultDescriptorSets;
    vkb::RenderSystem instanceSystem{device};

    vkb::Camera camera{};
    vkb::CameraMovementController cameraController{};

    vkb::InstancedObjects<InstanceData> instancedSpheres{device, INSTANCE_COUNT, vkb::Model::createModelFromFile(device, sphereModelPath)};

    float gravityFactor = 10.f, sphereRadius = 0.641f;
    std::vector<float> sphereSpeeds;
    std::vector<uint32_t> iter;
    float gpuTime = 0, cpuTime = 0;
    bool activateTimer = false;

    void onCreate() override;
    void initializeObjects();
    void createInstances();
    void createUniformBuffers();
    void mainLoop(float deltaTime) override;
    void updateUniformBuffer(uint32_t frameIndex);
    void showImGui();

    void updateSpheres(float deltaTime);

    void computeDensityPressure();
    void computeForces();
    void integrate();
};


#endif //VULKANBASE_INSTANCINGAPP_H
