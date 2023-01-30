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



template<typename T, uint32_t row, uint32_t col, uint32_t depth>
class Matrix3D{
public:

    void swap(Matrix3D& other){
        m_matrix.swap(other.m_matrix);
    }

    constexpr T& operator() (uint32_t i, uint32_t j, uint32_t k) {
        return m_matrix[i + row*j + row*col*k];
    }
    constexpr const T& operator()(uint32_t i, uint32_t j, uint32_t k) const{
        return m_matrix[i + row*j + row*col*k];
    }
    constexpr T& operator() (glm::ivec3& vec) {
        return m_matrix[vec.x + row*vec.y + row*col*vec.z];
    }
    constexpr const T& operator()(glm::ivec3& vec) const{
        return m_matrix[vec.x + row*vec.y + row*col*vec.z];
    }
    constexpr T& operator[] (uint32_t i) {
        return m_matrix[i];
    }
    constexpr const T& operator[](uint32_t i) const{
        return m_matrix[i];
    }

private:
    std::array<T, row*col*depth> m_matrix{};
};

class Grid3DSim: public vkb::VulkanApp {
public:
    Grid3DSim(int width, int height, const std::string &appName, vkb::Device::PhysicalDeviceType type = vkb::Device::INTEL):
            VulkanApp(width, height, appName, type) {}

private:
    uint32_t INSTANCE_COUNT = 1;

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
        glm::vec3 position;
        glm::vec3 color;
        float scale;
    };

    vkb::DrawableObject plane{vkb::Model::createModelFromFile(device, planeModelPath)};

    std::vector<std::unique_ptr<vkb::Buffer>> uniformBuffers;

    vkb::RenderSystem defaultSystem{device};
    std::vector<VkDescriptorSet> defaultDescriptorSets;
    vkb::RenderSystem instanceSystem{device};

    vkb::Camera camera{};
    vkb::CameraMovementController cameraController{};

    vkb::InstancedObjects<InstanceData> instancedCubes{device, INSTANCE_COUNT, vkb::Model::createModelFromFile(device, cubeModelPath)};

    std::vector<uint32_t> iter;
    float gpuTime = 0, cpuTime = 0;
    bool activateTimer = false;

    void onCreate() override;
    void initializeObjects();
    void createInstances();
    void createUniformBuffers();
    void mainLoop(float deltaTime) override;
    void updateUniformBuffer(uint32_t frameIndex, float deltaTime);
    void showImGui();

};
#endif //VULKANFLUIDSIMULATION_GRID3DSIM_H
