//
// Created by luc on 11/03/23.
//

#ifndef VULKANFLUIDSIMULATION_SPHCPU2DSIM_H
#define VULKANFLUIDSIMULATION_SPHCPU2DSIM_H

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
#include "../lib/Matrices.h"


class FLIPCPU2DSim: public vkb::VulkanApp {
public:
    constexpr static uint32_t WIDTH = 1500;
    constexpr static uint32_t HEIGHT = 1000;

    explicit FLIPCPU2DSim(const std::string &appName, vkb::Device::PhysicalDeviceType type = vkb::Device::INTEL):
            VulkanApp(WIDTH, HEIGHT, appName, type) {}

private:
    static constexpr uint32_t PARTICLE_COUNT = 800;
    static constexpr glm::vec3 gravity{0.0f, -9.81f, 0.0f};
    static constexpr float dt = 1/120.0f;
    static constexpr float radius = 8.0f;
    static constexpr uint32_t SIZE = 10;
    constexpr static uint32_t numTilesX = WIDTH/SIZE + 1;
    constexpr static uint32_t numTilesY = HEIGHT/SIZE + 1;
    constexpr static uint32_t CELL_COUNT = numTilesX*numTilesY;

    enum CellType {
        AIR, SOLID, FLUID
    };

    const vkb::RenderSystem::ShaderPaths shaderPaths = vkb::RenderSystem::ShaderPaths {
            "../src/FLIPCPU2DSim/Shaders/default.vert.spv",
            "../src/FLIPCPU2DSim/Shaders/default.frag.spv"
    };


    struct Particle {
        alignas(16) glm::vec3 position, velocity;
        glm::vec4 color;

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

        [[nodiscard]] glm::ivec2 gridPos(uint32_t xShift = 0, uint32_t yShift = 0) const { return {
            (uint32_t(std::clamp(position.x, radius, float(SIZE*(numTilesX - 1)))) - xShift) / SIZE,
            (uint32_t(std::clamp(position.y, radius, float(SIZE*(numTilesY - 1)))) - yShift) / SIZE}; }
    };

    struct UniformBufferObject {
        glm::mat4 view;
        glm::mat4 proj;
        float radius;
    };

    struct FluidData {
        Matrix<float, CELL_COUNT, numTilesX> velX{}, velY{};
    };

    struct GridParticle {
        glm::ivec2 gridPos;
        std::array<float, 4> weights;
    };

    std::vector<std::unique_ptr<vkb::Buffer>> uniformBuffers;
    UniformBufferObject ubo{};


    vkb::RenderSystem defaultSystem{device};
    std::vector<VkDescriptorSet> defaultDescriptorSets;

    vkb::Camera camera{};

    FluidData current, previous;
    Matrix<CellType, CELL_COUNT, numTilesX> cellTypes{};
    Matrix<float, CELL_COUNT, numTilesX> weightVelX{}, weightVelY{};
    Matrix<float, CELL_COUNT, numTilesX> solidCells{};
    Matrix<float, CELL_COUNT, numTilesX> densities{};
    float flipRatio = 0.9f, overRelaxation = 1.9f, restDensity = 0.0f, gasCoefficient = 1.0f;
    uint32_t numIterations = 25;
    std::vector<Particle> particles{PARTICLE_COUNT};
    std::vector<std::unique_ptr<vkb::Buffer>> particleData;

    float gpuTime = 0, cpuTime = 0;

    bool activateTimer = false;

    void onCreate() override;
    void resetGrid(bool hardReset = false);
    void initializeObjects();
    void createUniformBuffers();
    void mainLoop(float deltaTime) override;
    void updateBuffers(uint32_t frameIndex);
    void showImGui();

    void updateSimulation();
    void integrateParticles();
    void particlesCollideParticles();
    void particlesCollideObjects();
    void updateDensities();
    static std::tuple<float, float, float, float> particleGridWeights(const Particle& particle, glm::ivec2 gridPos, uint32_t xShift, uint32_t yShift);
    void transferParticlesVelocitiesToGrid();
    void transferGridVelocitiesToParticles();
    void projectVelocities();

};

#endif //VULKANFLUIDSIMULATION_SPHCPU2DSIM_H
