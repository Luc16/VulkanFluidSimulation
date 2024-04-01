//
// Created by luc on 12/03/24.
//

#ifndef VULKANFLUIDSIMULATION_FLIPRENDERER_H
#define VULKANFLUIDSIMULATION_FLIPRENDERER_H

#include <sstream>
#include <ranges>
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
#include "../../lib/graphicsDataStructures/Matrices.h"
#include "../../lib/descriptors/DescriptorPool.h"
#include "structs.h"
#include "PressureSolver.h"
#include "FlipSolver.h"
#include "../../lib/simulations/OffscreenPass.h"
#include "../../lib/CubeMapModel.h"

class FlipRenderer {
public:
    FlipRenderer(const vkb::Device& device, const std::vector<std::string>& shaders, VkExtent2D extent):
        m_deviceRef(device),
        m_particleSystem(device),
        m_shadingRenderSystem(device),
        m_depthPass(device, extent, true),
        m_thicknessPass(device, extent),
        m_scenePass(device, extent),
        m_smoothPass(device, extent, true, true),
        m_skyboxTexSystem(device),
        m_shaderPaths(shaders)
        {}

    void initialize(vkb::DescriptorPool& pool, const vkb::Renderer& renderer, const std::unique_ptr<vkb::Buffer>& uniformBuffer,
                    const vkb::CubeMapModel& skybox, const vkb::DrawableObject& plane, const std::unique_ptr<vkb::Buffer>& sdf);
    void runOffscreenPasses(VkCommandBuffer commandBuffer, const FlipSolver &solver, uint32_t currentFrame,
                            const vkb::CubeMapModel& skybox, const vkb::DrawableObject& plane, vkb::RenderSystem& defaultRenderSystem,
                            std::vector<VkDescriptorSet>& defaultDescriptorSets, std::vector<VkDescriptorSet>& skyboxDescriptorSets, bool renderSkybox);
    void render(VkCommandBuffer& commandBuffer, const FlipSolver& solver, uint32_t frame, uint32_t renderType);

    void resize(VkExtent2D newExtent, vkb::DescriptorPool& pool, const std::unique_ptr<vkb::Buffer>& uniformBuffer,
                const vkb::CubeMapModel& skybox, const vkb::DrawableObject& plane);

    int blurIterations = 2;

private:
    void createOffscreenPasses();
    void initializeDescriptorSets(vkb::DescriptorPool& pool, const std::unique_ptr<vkb::Buffer>& uniformBuffer,
                                  const vkb::CubeMapModel& skybox, const vkb::DrawableObject& plane);

    const vkb::Device& m_deviceRef;
    const std::vector<std::string> m_shaderPaths;
    bool initialized = false;

    vkb::RenderSystem m_particleSystem;
    std::vector<VkDescriptorSet> m_particleDescriptorSets;

    vkb::RenderSystem m_shadingRenderSystem;

    // rendering with screen space fluids
    vkb::OffscreenPass m_depthPass;
    vkb::OffscreenPass m_thicknessPass;
    vkb::OffscreenPass m_scenePass;
    vkb::OffscreenPass m_smoothPass;
    vkb::RenderSystem m_skyboxTexSystem;


    std::vector<VkDescriptorSet> simulationDescriptorSets;
    std::vector<VkDescriptorSet> smooth1DescriptorSets;
    std::vector<VkDescriptorSet> smooth2DescriptorSets;
    std::array<std::vector<VkDescriptorSet>, 3> shadingDescriptorSets;


};


#endif //VULKANFLUIDSIMULATION_FLIPRENDERER_H
