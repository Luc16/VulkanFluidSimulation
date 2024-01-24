//
// Created by luc on 11/03/23.
//

#ifndef VULKANFLUIDSIMULATION_FLIPCPU2DSIM_H
#define VULKANFLUIDSIMULATION_FLIPCPU2DSIM_H

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
#include "../lib/graphicsDataStructures/Matrices.h"
#include "structs.h"
#include "PressureSolver.h"

class FLIPCPU2DSim: public vkb::VulkanApp {
public:
    constexpr static uint32_t WIDTH = 1500;
    constexpr static uint32_t HEIGHT = 1000;

    explicit FLIPCPU2DSim(const std::string &appName, vkb::Device::PhysicalDeviceType type = vkb::Device::INTEL):
            VulkanApp(WIDTH, HEIGHT, appName, type) {}

private:
    static constexpr uint32_t PARTICLE_COUNT = 2000;
    static constexpr float dt = 1/120.0f;
//    static constexpr float dt = 1/60.0f;
    static constexpr float radius = 8.0f;
    static constexpr uint32_t SIZE = 20;
    constexpr static uint32_t numTilesX = WIDTH/SIZE;
    constexpr static uint32_t numTilesY = HEIGHT/SIZE;
    constexpr static uint32_t CELL_COUNT = numTilesX*numTilesY;
    float flipRatio = 0.9f;
    uint32_t numIterations = 200;

    enum CellType {
        AIR, SOLID, FLUID
    };

    const vkb::RenderSystem::ShaderPaths particleShaderPaths = vkb::RenderSystem::ShaderPaths {
            "../src/FLIPCPU2DSim/Shaders/particle.vert.spv",
            "../src/FLIPCPU2DSim/Shaders/particle.frag.spv"
    };

    const vkb::RenderSystem::ShaderPaths defaultShaderPaths = vkb::RenderSystem::ShaderPaths {
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
            attributeDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
            attributeDescriptions[0].offset = offsetof(Particle, position);

            attributeDescriptions[1].binding = 0;
            attributeDescriptions[1].location = 1;
            attributeDescriptions[1].format = VK_FORMAT_R32G32B32A32_SFLOAT;
            attributeDescriptions[1].offset = offsetof(Particle, color);

            return attributeDescriptions;
        }

        [[nodiscard]] glm::ivec2 gridPos(uint32_t xShift = 0, uint32_t yShift = 0) const { return {
            (uint32_t(std::clamp(position.x, float(xShift), float(SIZE*(numTilesX - 1)))) - xShift) / SIZE,
            (uint32_t(std::clamp(position.y, float(yShift), float(SIZE*(numTilesY - 1)))) - yShift) / SIZE}; }
    };

    struct FluidData {
        Matrix<float, CELL_COUNT, numTilesX> velX{}, velY{};
    };

    std::vector<std::unique_ptr<vkb::Buffer>> uniformBuffers;
    UniformBufferObject ubo{};


    vkb::RenderSystem particleSystem{device};
    vkb::RenderSystem lineSystem{device};
    vkb::RenderSystem quadSystem{device};
    std::vector<VkDescriptorSet> defaultDescriptorSets;

    vkb::Camera camera{};

    FluidData current, previous;
    Matrix<CellType, CELL_COUNT, numTilesX> cellTypes{};
    Matrix<float, CELL_COUNT, numTilesX> weightVelX{}, weightVelY{};
    Matrix<float, CELL_COUNT, numTilesX> solidCells{};
    HeapMatrix<double, numTilesX> pressure{CELL_COUNT};
    HeapMatrix<double, numTilesX> rhs{CELL_COUNT};
    PressureSolver pressureSolver{};
    std::vector<Particle> particles{PARTICLE_COUNT};
    std::unique_ptr<vkb::Buffer> particleBuffer;
    bool showParticles = true;

    // render grid
    std::vector<Line> gridLines;
    glm::vec3 gridColor{0.01f};
    bool showGrid = false;
    std::unique_ptr<vkb::Buffer> gridLinesBuffer;

    // render velocity field
    std::vector<Line> velocityLines;
    std::unique_ptr<vkb::Buffer> velocityFieldBuffer;
    bool showVelField = false;

    // render fluid quads
    std::vector<GridQuad> fluidQuads;
    std::unique_ptr<vkb::Buffer> fluidQuadBuffer;
    bool showFluidQuads = false;


    float gpuTime = 0, cpuTime = 0;
    bool activateTimer = false, paused = false;

    void onCreate() override;
    void resetGrid(bool hardReset = false);
    void initializeObjects();
    void generateGridLines();
    void createBuffers();
    void mainLoop(float deltaTime) override;
    void renderObjects();
    void drawGrid(VkCommandBuffer commandBuffer);
    void applyGridLineColor();
    void updateAndDrawVelocityField(VkCommandBuffer commandBuffer);
    void updateAndDrawFluidQuads(VkCommandBuffer commandBuffer);
    void updateBuffers(uint32_t frameIndex);
    void showImGui();

    void updateSimulation(float deltaTime);
    void advectParticles();
    static std::tuple<float, float, float, float> particleGridWeights(const Particle& particle, glm::ivec2 gridPos, uint32_t xShift, uint32_t yShift);
    static void applyWeightedValuesToMatrix(Matrix<float, CELL_COUNT, numTilesX>& matrix, glm::ivec2 gridPos, std::tuple<float, float, float, float> weights, float value);
    void transferParticlesVelocitiesToGrid();
    void transferGridVelocitiesToParticles();
    void enforceDirichlet();
    void projectVelocities(float deltaTime);

};

#endif //VULKANFLUIDSIMULATION_FLIPCPU2DSIM_H
