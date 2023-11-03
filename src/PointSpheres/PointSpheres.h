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

class PointSpheres: public vkb::VulkanApp {
public:
    PointSpheres(int width, int height, const std::string &appName, vkb::Device::PhysicalDeviceType type = vkb::Device::INTEL):
    VulkanApp(width, height, appName, type) {}

private:
//    uint32_t NUM_PARTICLES = 65536;
    uint32_t INSTANCE_COUNT = 64;

    const std::string planeModelPath = "../Models/quadXZ.obj";
    const vkb::RenderSystem::ShaderPaths shaderPaths = vkb::RenderSystem::ShaderPaths {
            "../src/PointSpheres/Shaders/default.vert.spv",
            "../src/PointSpheres/Shaders/default.frag.spv"
    };

    const std::string sphereModelPath = "../Models/lowsphere.obj";
    const vkb::RenderSystem::ShaderPaths instanceShaderPaths = vkb::RenderSystem::ShaderPaths {
            "../src/PointSpheres/Shaders/point_sphere.vert.spv",
            "../src/PointSpheres/Shaders/point_sphere.frag.spv",
    };

    struct UniformBufferObject {
        alignas(16) glm::mat4 viewProj;
        alignas(16) glm::vec3 cameraPos;
        alignas(16) glm::vec3 lightDir = glm::vec3(-1.0f, -1.0f, 0.0f);
        float radius = 1.2f;
    };

    struct SphereData {
        glm::vec3 position;
        glm::vec4 color;
    };

    vkb::DrawableObject plane{vkb::Model::createModelFromFile(device, planeModelPath)};

    std::vector<std::unique_ptr<vkb::Buffer>> uniformBuffers;

    vkb::RenderSystem defaultSystem{device};
    std::vector<VkDescriptorSet> defaultDescriptorSets;
    vkb::RenderSystem pointSphereSystem{device};

    vkb::Camera camera{};
    vkb::CameraMovementController cameraController{};

    std::vector<SphereData> spheres{};
    std::unique_ptr<vkb::Buffer> sphereBuffer;

    UniformBufferObject ubo{};
    float damping = 0.05f, sphereRadius = 0.641f;
    std::vector<float> sphereSpeeds;
    std::vector<uint32_t> iter;
    float gpuTime = 0, cpuTime = 0;
    bool activateTimer = false;
    bool drawPlane = true;

    void onCreate() override;
    void initializeObjects();
    void createInstances();
    void createUniformBuffers();
    void mainLoop(float deltaTime) override;
    void updateSpheres(float deltaTime);
    void updateUniformBuffer(uint32_t frameIndex, float deltaTime);
    void showImGui();

};


#endif //VULKANBASE_INSTANCINGAPP_H
