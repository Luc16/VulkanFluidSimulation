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


class SPHGPU3DSim: public vkb::VulkanApp {
public:
    SPHGPU3DSim(int width, int height, const std::string &appName, vkb::Device::PhysicalDeviceType type = vkb::Device::NVIDIA):
            VulkanApp(width, height, appName, type) {}

private:
    uint32_t INSTANCE_COUNT = 1024;

    const std::string planeModelPath = "../Models/quadXZ1.obj";
    const vkb::RenderSystem::ShaderPaths shaderPaths = vkb::RenderSystem::ShaderPaths {
            "../src/SPH/SPHGPU3DSim/Shaders/default.vert.spv",
            "../src/SPH/SPHGPU3DSim/Shaders/default.frag.spv"
    };

    const std::string sphereModelPath = "../Models/lowsphere.obj";
    const vkb::RenderSystem::ShaderPaths instanceShaderPaths = vkb::RenderSystem::ShaderPaths {
            "../src/SPH/SPHGPU3DSim/Shaders/instancing.vert.spv",
            "../src/SPH/SPHGPU3DSim/Shaders/instancing.frag.spv",
    };

    const std::string calculateForcesShaderPath = "../src/SPH/SPHGPU3DSim/Shaders/calculate_forces.comp.spv";
    const std::string integrateShaderPath = "../src/SPH/SPHGPU3DSim/Shaders/integrate.comp.spv";
    const std::string calculateDensityPressureShaderPath = "../src/SPH/SPHGPU3DSim/Shaders/calculate_density_pressure.comp.spv";

    struct Particle {
        alignas(16) glm::vec3 position{};
        alignas(16) glm::vec3 velocity{};
        alignas(16) glm::vec3 force{};
        alignas(16) glm::vec3 color{};
        float density{}, pressure{}, scale{};
    };

    struct UniformBufferObject {
        glm::mat4 view;
        glm::mat4 proj;
        alignas(16) glm::vec3 lightDirection = glm::normalize(glm::vec3(1.0f, 1.0f, 0.0f));
    };

    struct ComputeUniformBufferObject {
        float deltaTime = 1/60.0f;
        float BOUNDARY_SIZE = 70.0f;
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
    };

    vkb::DrawableObject plane{vkb::Model::createModelFromFile(device, planeModelPath)};

    std::vector<std::unique_ptr<vkb::Buffer>> graphicsUniformBuffers;
    UniformBufferObject gUbo{};
    vkb::RenderSystem defaultSystem{device};
    std::vector<VkDescriptorSet> defaultDescriptorSets;
    vkb::RenderSystem instanceSystem{device};


    vkb::InstancedObjects<Particle> instancedSpheres {
        device,0,
        vkb::Model::createModelFromFile(device, sphereModelPath),
        nullptr, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT
    };
    std::unique_ptr<vkb::Buffer> computeUniformBuffer;
    ComputeUniformBufferObject cUbo{};
    vkb::ComputeShaderHandler computeHandler{device};
    vkb::ComputeSystem calculateDensityPressureComputeSystem{device};
    vkb::ComputeSystem calculateForcesComputeSystem{device};
    vkb::ComputeSystem integrateComputeSystem{device};
    VkDescriptorSet computeDescriptorSet = nullptr;

    vkb::Camera camera{};
    vkb::CameraMovementController cameraController{};

    glm::vec3 initialPos = {cUbo.EPS, cUbo.EPS, cUbo.EPS};
    float drawTime = 0, cpuTime = 0, computeTime = 0, gravityFactor = 1.0f;
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
