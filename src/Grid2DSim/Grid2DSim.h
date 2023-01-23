//
// Created by luc on 30/12/22.
//

#ifndef VULKANFLUIDSIMULATION_GRID2DSIM_H
#define VULKANFLUIDSIMULATION_GRID2DSIM_H

// TODO:
/*
 * Arbitrary bounds
 *  */

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

template<typename T>
class Matrix{
private:
    std::vector<T> m_matrix;
    uint32_t m_row{};
public:

    void resize(size_t newSize, uint32_t row){
        m_row = row;
        m_matrix.resize(newSize);
    }

    void swap(Matrix& other){
        m_matrix.swap(other.m_matrix);
    }

    constexpr T& operator() (uint32_t i, uint32_t j) {
        return m_matrix[i + m_row*j];
    }
    constexpr const T& operator()(uint32_t i, uint32_t j) const{
        return m_matrix[i + m_row*j];
    }
    constexpr T& operator() (glm::ivec2& vec) {
        return m_matrix[vec.x + m_row*vec.y];
    }
    constexpr const T& operator()(glm::ivec2& vec) const{
        return m_matrix[vec.x + m_row*vec.y];
    }
    constexpr T& operator[] (uint32_t i) {
        return m_matrix[i];
    }
    constexpr const T& operator[](uint32_t i) const{
        return m_matrix[i];
    }
};

class Grid2DSim: public vkb::VulkanApp {
public:
    Grid2DSim(int width, int height, const std::string &appName, vkb::Device::PhysicalDeviceType type = vkb::Device::INTEL):
            VulkanApp(width, height, appName, type){}

private:
    enum BoundConfig {
        REGULAR, MIRROR_X, MIRROR_Y
    };

    enum CellType {
        EMPTY, BOUNDARY, WALL
    };

    const uint32_t SIZE = 10;
    uint32_t INSTANCE_COUNT = 0;

    const std::string planeModelPath = "../src/Grid2DSim/Models/quad.obj";
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
        Matrix<float> density, velX, velY;

        void resize(size_t newSize, uint32_t row);
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
    Matrix<CellType> cellTypes;
    std::forward_list<glm::ivec2> insideBoundaries;

    uint32_t numTilesX{}, numTilesY{}, numTilesMiddle{};

    static void forEachNeighbor(uint32_t i, uint32_t j, const std::function<void(uint32_t ni, uint32_t nj)>& func); // does not check if i, j pair is out of bounds

    void onCreate() override;
    void initializeObjects();
    void createUniformBuffers();
    void mainLoop(float deltaTime) override;
    void onResize(int width, int height) override;
    void updateGrid(float deltaTime);
    void createWalls();
    void resetGrid(bool hardReset = false);
    void updateUniformBuffer(uint32_t frameIndex, float deltaTime);
    void showImGui();

    // fluid simulation functions
    void updateVelocities(float deltaTime);
    void updateDensities(float deltaTime);

    void diffuse(Matrix<float>& x, const Matrix<float>& x0, float diff, float dt);
    void advect(Matrix<float>& d, const Matrix<float>& d0, const Matrix<float>& velX, const Matrix<float>& velY, float dt, BoundConfig b = REGULAR);
    void project(Matrix<float> &velX, Matrix<float> &velY, Matrix<float> &div, Matrix<float> &p);
    void setBounds(Matrix<float>& x, BoundConfig b = REGULAR) const;

    // custom boundaries functions

    void diffuseInnerBounds(Matrix<float>& x, const Matrix<float>& x0, float a);
    void setInnerBounds(Matrix<float> &d, BoundConfig b = REGULAR);
};



#endif //VULKANFLUIDSIMULATION_GRID2DSIM_H
