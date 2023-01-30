//
// Created by luc on 30/12/22.
//

#ifndef VULKANFLUIDSIMULATION_GRID2DSIM_H
#define VULKANFLUIDSIMULATION_GRID2DSIM_H

#include <sstream>
#include <forward_list>
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
#include "Grid2D.h"



template<typename T, size_t size, uint32_t row>
class Matrix{
public:

    void swap(Matrix& other){
        m_matrix.swap(other.m_matrix);
    }

    constexpr T& operator() (uint32_t i, uint32_t j) {
        return m_matrix[i + row*j];
    }
    constexpr const T& operator()(uint32_t i, uint32_t j) const{
        return m_matrix[i + row*j];
    }
    constexpr T& operator() (glm::ivec2& vec) {
        return m_matrix[vec.x + row*vec.y];
    }
    constexpr const T& operator()(glm::ivec2& vec) const{
        return m_matrix[vec.x + row*vec.y];
    }
    constexpr T& operator[] (uint32_t i) {
        return m_matrix[i];
    }
    constexpr const T& operator[](uint32_t i) const{
        return m_matrix[i];
    }

private:
    std::array<T, size> m_matrix{};
};

class Grid2DSim: public vkb::VulkanApp {
public:
    Grid2DSim(int width, int height, const std::string &appName, vkb::Device::PhysicalDeviceType type = vkb::Device::INTEL):
            VulkanApp(width, height, appName, type, false) {}

private:
    enum BoundConfig {
        REGULAR, MIRROR_X, MIRROR_Y
    };

    enum CellType {
        EMPTY, OUT_BOUNDARY, IN_BOUNDARY, WALL
    };

    constexpr static uint32_t WIDTH = 1000;
    constexpr static uint32_t HEIGHT = 1000;
    constexpr static uint32_t SIZE = 10;
    constexpr static uint32_t numTilesX = WIDTH/SIZE;
    constexpr static uint32_t numTilesMiddle = numTilesX - 2;
    constexpr static uint32_t numTilesY = HEIGHT/SIZE;
    constexpr static uint32_t INSTANCE_COUNT = numTilesX*numTilesY;

    const std::string planeModelPath = "../Models/quadXY.obj";
    const vkb::RenderSystem::ShaderPaths shaderPaths = vkb::RenderSystem::ShaderPaths {
            "../src/Grid2DSim/Shaders/default.vert.spv",
            "../src/Grid2DSim/Shaders/default.frag.spv"
    };

    const vkb::RenderSystem::ShaderPaths instanceShaderPaths = vkb::RenderSystem::ShaderPaths {
            "../src/Grid2DSim/Shaders/instancing.vert.spv",
            "../src/Grid2DSim/Shaders/instancing.frag.spv",
    };

    struct UniformBufferObject {
        alignas(16) glm::mat4 view;
        alignas(16) glm::mat4 proj;
    };

    struct InstanceData {
        glm::vec3 position;
        glm::vec3 color;
        float scale;
    };

    struct FluidData {
        Matrix<float, INSTANCE_COUNT, numTilesX> density{}, velX{}, velY{};
    };

    // Vulkan variables
    std::vector<std::unique_ptr<vkb::Buffer>> uniformBuffers;

    vkb::RenderSystem defaultSystem{device};
    std::vector<VkDescriptorSet> defaultDescriptorSets;
    vkb::RenderSystem instanceSystem{device};

    vkb::Camera camera{};

    vkb::InstancedObjects<InstanceData> grid{device, INSTANCE_COUNT, vkb::Model::createModelFromFile(device, planeModelPath)};

    // Simulation and control variables

    std::vector<uint32_t> iter;
    float gpuTime = 0, cpuTime = 0;
    bool activateTimer = false, wallMode = false;

    float viscosity = 0.005f, diffusionFactor = 0.001f, dissolveFactor = 0.015f, initialSpeed = 500.0f;

    FluidData curState, prevState;
    Matrix<CellType, INSTANCE_COUNT, numTilesX> cellTypes;
    std::forward_list<glm::ivec2> insideBoundaries;

    static void forEachNeighbor(uint32_t i, uint32_t j, const std::function<void(uint32_t ni, uint32_t nj)>& func); // does not check if i, j pair is out of bounds

    void onCreate() override;
    void initializeObjects();
    void createUniformBuffers();
    void mainLoop(float deltaTime) override;
    void updateGrid(float deltaTime);
    void createWalls();
    void resetGrid(bool hardReset = false);
    void updateUniformBuffer(uint32_t frameIndex);
    void showImGui();

    // fluid simulation functions
    void updateVelocities(float deltaTime);
    void updateDensities(float deltaTime);

    void diffuse(Matrix<float, INSTANCE_COUNT, numTilesX>& x, const Matrix<float, INSTANCE_COUNT, numTilesX>& x0, float diff, float dt);
    void advect(Matrix<float, INSTANCE_COUNT, numTilesX>& d, const Matrix<float, INSTANCE_COUNT, numTilesX>& d0, const Matrix<float, INSTANCE_COUNT, numTilesX>& velX, const Matrix<float, INSTANCE_COUNT, numTilesX>& velY, float dt, BoundConfig b = REGULAR);
    void project(Matrix<float, INSTANCE_COUNT, numTilesX> &velX, Matrix<float, INSTANCE_COUNT, numTilesX> &velY, Matrix<float, INSTANCE_COUNT, numTilesX> &div, Matrix<float, INSTANCE_COUNT, numTilesX> &p);
    static void setBounds(Matrix<float, INSTANCE_COUNT, numTilesX>& x, BoundConfig b = REGULAR) ;

    // custom boundaries functions

    void diffuseInnerBounds(Matrix<float, INSTANCE_COUNT, numTilesX>& x, const Matrix<float, INSTANCE_COUNT, numTilesX>& x0, float a);
    void setInnerBounds(Matrix<float, INSTANCE_COUNT, numTilesX> &d, BoundConfig b = REGULAR);
};



#endif //VULKANFLUIDSIMULATION_GRID2DSIM_H
