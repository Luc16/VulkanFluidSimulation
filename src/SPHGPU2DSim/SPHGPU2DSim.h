//
// Created by luc on 11/03/23.
//

#ifndef VULKANFLUIDSIMULATION_SPHGPU2DSIM_H
#define VULKANFLUIDSIMULATION_SPHGPU2DSIM_H

#define GLM_GTX_norm
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
#include "../lib/ComputeSystem.h"
#include "../lib/ComputeShaderHandler.h"


class SPHGPU2DSim: public vkb::VulkanApp {
public:
    SPHGPU2DSim(int width, int height, const std::string &appName, vkb::Device::PhysicalDeviceType type = vkb::Device::INTEL):
            VulkanApp(width, height, appName, type) {}

private:
    static constexpr uint32_t PARTICLE_COUNT = 512;

    const vkb::RenderSystem::ShaderPaths shaderPaths = vkb::RenderSystem::ShaderPaths {
            "../src/SPHGPU2DSim/Shaders/default.vert.spv",
            "../src/SPHGPU2DSim/Shaders/default.frag.spv"
    };

    const std::string calculateForcesShaderPath = "../src/SPHGPU2DSim/Shaders/calculate_forces.comp.spv";
    const std::string integrateShaderPath = "../src/SPHGPU2DSim/Shaders/integrate.comp.spv";
    const std::string calculateDensityPressureShaderPath = "../src/SPHGPU2DSim/Shaders/calculate_density_pressure.comp.spv";

    struct Particle {
        alignas(16) glm::vec3 position{};
        alignas(16) glm::vec3 velocity{};
        alignas(16) glm::vec3 force{};
        float density{}, pressure{};
        glm::vec4 color{};

        static VkVertexInputBindingDescription getBindingDescription() {
            VkVertexInputBindingDescription bindingDescription{};
            bindingDescription.binding = 0;
            bindingDescription.stride = sizeof(Particle);
            bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

            return bindingDescription;
        }

        static std::vector<VkVertexInputAttributeDescription> getAttributeDescriptions() {
            std::vector<VkVertexInputAttributeDescription> attributeDescriptions{2};

            attributeDescriptions[0].binding = 0;
            attributeDescriptions[0].location = 0;
            attributeDescriptions[0].format = VK_FORMAT_R32G32_SFLOAT;
            attributeDescriptions[0].offset = offsetof(Particle, position);

            attributeDescriptions[1].binding = 0;
            attributeDescriptions[1].location = 1;
            attributeDescriptions[1].format = VK_FORMAT_R32G32B32A32_SFLOAT;
            attributeDescriptions[1].offset = offsetof(Particle, color);

            return attributeDescriptions;
        }
    };

    struct UniformBufferObject {
        glm::mat4 view;
        glm::mat4 proj;
        float radius;
    };

    struct ComputeUniformBufferObject {
        float deltaTime = 1/60.0f;
        float width = 0.0f;
        float height = 0.0f;

        alignas(16) glm::vec3 G{0.0f, -10.0f, 0.0f};   // external (gravitational) forces
        float REST_DENS = 300.f;  // rest density
        float GAS_CONST = 2000.f; // const for equation of state
        float H = 16.f;           // kernel radius
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
    };

    std::vector<std::unique_ptr<vkb::Buffer>> graphicsUniformBuffers;
    UniformBufferObject gUbo{};
    vkb::RenderSystem defaultSystem{device};
    std::vector<VkDescriptorSet> defaultDescriptorSets;


    std::unique_ptr<vkb::Buffer> particleBuffer;
    std::unique_ptr<vkb::Buffer> computeUniformBuffer;
    ComputeUniformBufferObject cUbo{};
    vkb::ComputeShaderHandler computeHandler{device};
    vkb::ComputeSystem calculateDensityPressureComputeSystem{device};
    vkb::ComputeSystem calculateForcesComputeSystem{device};
    vkb::ComputeSystem integrateComputeSystem{device};
    VkDescriptorSet computeDescriptorSet{};

    vkb::Camera camera{};

    float gpuTime = 0, cpuTime = 0;
    bool activateTimer = false;

    void onCreate() override;
    void initializeObjects();
    void createComputeDescriptorSets(vkb::DescriptorSetLayout &layout);
    void createUniformBuffers();
    void mainLoop(float deltaTime) override;
    void updateBuffers(uint32_t frameIndex, float deltaTime);
    void showImGui();

};

#endif //VULKANFLUIDSIMULATION_SPHGPU2DSIM_H
