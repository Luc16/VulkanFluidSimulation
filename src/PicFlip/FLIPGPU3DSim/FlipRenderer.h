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

class FlipRenderer {
public:
    FlipRenderer(const vkb::Device& device, const std::vector<std::string>& shaders):
        m_deviceRef(device),
        m_particleSystem(device),
        m_shaderPaths(shaders)
        {}

    void createRenderSystems(vkb::DescriptorPool& pool, const vkb::Renderer& renderer, const std::unique_ptr<vkb::Buffer>& uniformBuffer);
    void render(VkCommandBuffer& commandBuffer, const FlipSolver& solver, uint32_t frame);

private:
    const vkb::Device& m_deviceRef;
    const std::vector<std::string> m_shaderPaths;

    vkb::RenderSystem m_particleSystem;
    std::vector<VkDescriptorSet> m_particleDescriptorSets;

};


#endif //VULKANFLUIDSIMULATION_FLIPRENDERER_H
