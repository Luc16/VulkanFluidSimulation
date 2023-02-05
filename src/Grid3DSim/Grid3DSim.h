//
// Created by luc on 30/01/23.
//

#ifndef VULKANFLUIDSIMULATION_GRID3DSIM_H
#define VULKANFLUIDSIMULATION_GRID3DSIM_H

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



template<typename T>
class Matrix3D{
public:
    Matrix3D(uint32_t row, uint32_t col, uint32_t depth): m_row(row), m_col(col), m_depth(depth), m_rowCol(row*col), m_data(row*col*depth) {}
    Matrix3D() = default;
    Matrix3D(const Matrix3D<T> &) = delete;
    Matrix3D &operator=(const Matrix3D<T> &) = delete;

    void resize(uint32_t row, uint32_t col, uint32_t depth) {
        m_row = row;
        m_col = col;
        m_depth = depth;
        m_rowCol = row*col;
        m_data.resize(row*col*depth);
    }

    void swap(Matrix3D& other){
        m_data.swap(other.m_data);
    }

    constexpr T& operator() (uint32_t i, uint32_t j, uint32_t k) {
        return m_data[i + m_row*j + m_rowCol*k];
    }
    constexpr const T& operator()(uint32_t i, uint32_t j, uint32_t k) const{
        return m_data[i + m_row*j + m_rowCol*k];
    }
    constexpr T& operator() (glm::ivec3& vec) {
        return m_data[vec.x + m_row*vec.y + m_rowCol*vec.z];
    }
    constexpr const T& operator()(glm::ivec3& vec) const{
        return m_data[vec.x + m_row*vec.y + m_rowCol*vec.z];
    }
    constexpr T& operator[] (uint32_t i) {
        return m_data[i];
    }
    constexpr const T& operator[](uint32_t i) const{
        return m_data[i];
    }

private:
    std::vector<T> m_data{};
    uint32_t m_row{}, m_col{}, m_depth{}, m_rowCol{};
};

class Grid3DSim: public vkb::VulkanApp {
public:
    Grid3DSim(int width, int height, const std::string &appName, vkb::Device::PhysicalDeviceType type = vkb::Device::INTEL):
            VulkanApp(width, height, appName, type) {}

private:
    enum BoundConfig {
        REGULAR, MIRROR_X, MIRROR_Y, MIRROR_Z
    };

    uint32_t CUBE_SIDE = 25;
    uint32_t INSTANCE_COUNT = CUBE_SIDE*CUBE_SIDE*CUBE_SIDE;
    uint32_t CUBE_N = CUBE_SIDE - 2;

    const std::string planeModelPath = "../Models/quadXZ.obj";
    const vkb::RenderSystem::ShaderPaths shaderPaths = vkb::RenderSystem::ShaderPaths {
            "../src/Grid3DSim/Shaders/default.vert.spv",
            "../src/Grid3DSim/Shaders/default.frag.spv"
    };

    const std::string cubeModelPath = "../Models/cube.obj";
    const vkb::RenderSystem::ShaderPaths instanceShaderPaths = vkb::RenderSystem::ShaderPaths {
            "../src/Grid3DSim/Shaders/instancing.vert.spv",
            "../src/Grid3DSim/Shaders/instancing.frag.spv",
    };

    struct UniformBufferObject {
        alignas(16) glm::mat4 view;
        alignas(16) glm::mat4 proj;
    };

    struct InstanceData {
        glm::vec3 position{};
        glm::vec4 color{};
        float scale{};
    };

    struct FluidData {
        Matrix3D<float> density{}, velX{}, velY{}, velZ{};
    };

    vkb::DrawableObject plane{vkb::Model::createModelFromFile(device, planeModelPath)};

    std::vector<std::unique_ptr<vkb::Buffer>> uniformBuffers;

    vkb::RenderSystem defaultSystem{device};
    std::vector<VkDescriptorSet> defaultDescriptorSets;
    vkb::RenderSystem instanceSystem{device};

    vkb::Camera camera{};
    vkb::CameraMovementController cameraController{};

    vkb::InstancedObjects<InstanceData> instancedCubes{device, INSTANCE_COUNT, vkb::Model::createModelFromFile(device, cubeModelPath)};

    float gpuTime = 0, cpuTime = 0;
    bool activateTimer = false;

    float viscosity = 0.005f, diffusionFactor = 0.001f, dissolveFactor = 0.006f, initialSpeed = 200.0f, gravity = 0.5f;

    FluidData curState, prevState;
    std::vector<InstanceData> sortedData;
    std::vector<std::pair<uint32_t, uint32_t>> indices;

    void onCreate() override;
    void initializeObjects();
    void createInstances();
    void createUniformBuffers();
    void mainLoop(float deltaTime) override;
    void updateGrid(float deltaTime);
    void updateUniformBuffer(uint32_t frameIndex);
    void showImGui();


    // fluid simulation functions
    void updateVelocities(float deltaTime);
    void updateDensities(float deltaTime);

    void diffuse(Matrix3D<float>& x, const Matrix3D<float>& x0, float diff, float dt);
    void advect(Matrix3D<float>& d, const Matrix3D<float>& d0, const Matrix3D<float>& velX, const Matrix3D<float>& velY, const Matrix3D<float>& velZ, float dt, BoundConfig b = REGULAR);
    void project(Matrix3D<float> &velX, Matrix3D<float> &velY, Matrix3D<float> &velZ, Matrix3D<float> &div, Matrix3D<float> &p);
    void setBounds(Matrix3D<float>& x, BoundConfig b = REGULAR) const;

    static std::tuple<uint32_t, uint32_t, float, float> linearBackTrace(float pos, float fN);
};
#endif //VULKANFLUIDSIMULATION_GRID3DSIM_H
