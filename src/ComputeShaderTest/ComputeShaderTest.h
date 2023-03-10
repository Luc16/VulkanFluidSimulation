//
// Created by luc on 05/02/23.
//

#ifndef VULKANFLUIDSIMULATION_COMPUTESHADERTEST_H
#define VULKANFLUIDSIMULATION_COMPUTESHADERTEST_H


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


class ComputeShaderTest: public vkb::VulkanApp {
public:
    ComputeShaderTest(int width, int height, const std::string &appName, vkb::Device::PhysicalDeviceType type = vkb::Device::INTEL):
            VulkanApp(width, height, appName, type) {}

private:
    static constexpr uint32_t PARTICLE_COUNT = 8192;
    const vkb::RenderSystem::ShaderPaths shaderPaths = vkb::RenderSystem::ShaderPaths {
            "../src/ComputeShaderTest/Shaders/default.vert.spv",
            "../src/ComputeShaderTest/Shaders/default.frag.spv"
    };
    const std::string calculateForcesShaderPath = "../src/ComputeShaderTest/Shaders/calculate_forces.comp.spv";
    const std::string moveParticlesShaderPath = "../src/ComputeShaderTest/Shaders/move_particles.comp.spv";


    struct Particle {
        glm::vec2 position, velocity;
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
    };

    struct ComputeUniformBufferObject {
        unsigned int numParticles;
        float gravitationalConstant;
        float deltaTime;
        float width;
        float height;
    };

    struct UniformBufferObject {
        glm::mat4 view;
        glm::mat4 proj;
    };

    std::vector<std::unique_ptr<vkb::Buffer>> computeData;
    std::vector<std::unique_ptr<vkb::Buffer>> computeUniformBuffers;
    ComputeUniformBufferObject cUbo{PARTICLE_COUNT, 500.0f, 1/60.0f, float(window.width()), float(window.height())};

    std::vector<std::unique_ptr<vkb::Buffer>> graphicsUniformBuffers;
    UniformBufferObject gUbo{};

    vkb::RenderSystem defaultSystem{device};
    std::vector<VkDescriptorSet> defaultDescriptorSets;

    vkb::ComputeShaderHandler computeHandler{device};
    vkb::ComputeSystem calculateForcesComputeSystem{device};
    vkb::ComputeSystem moveParticlesComputeSystem{device};
    std::vector<VkDescriptorSet> computeDescriptorSets;


    vkb::Camera camera{};

    float gpuTime = 0, cpuTime = 0;
    bool activateTimer = false;

    void onCreate() override;
    void initializeObjects();
    void createComputeDescriptorSets(vkb::DescriptorSetLayout& layout);
    void createUniformBuffers();
    void mainLoop(float deltaTime) override;
    void updateUniformBuffers(uint32_t frameIndex, float deltaTime);
    void showImGui();

};


#endif //VULKANFLUIDSIMULATION_COMPUTESHADERTEST_H
