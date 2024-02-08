//
// Created by luc on 25/01/24.
//

#ifndef VULKANFLUIDSIMULATION_FLIPSOLVER_H
#define VULKANFLUIDSIMULATION_FLIPSOLVER_H
#include <sstream>
#include <span>
#include "../../lib/utils.h"
#include "../../lib/graphicsDataStructures/Matrices.h"
#include "structs.h"
#include "PressureSolver.h"
#include "../../lib/ComputeSystem.h"
#include "../../lib/descriptors/DescriptorSetLayout.h"
#include "../../lib/Buffer.h"
#include "../../lib/ComputeShaderHandler.h"

class FlipSolver {
public:
    FlipSolver(const vkb::Device& device, const std::vector<std::string>& shaders): m_deviceRef(device), m_shaderPaths(shaders) {}

    void updateSimulation(float deltaTime, float flipRatio);
    void initializeParticles();

private:

    const vkb::Device& m_deviceRef;
    const std::vector<std::string>& m_shaderPaths;
    std::unique_ptr<vkb::Buffer> m_computeUniformBuffer;
    vkb::ComputeShaderHandler m_computeHandler{m_deviceRef};
    vkb::SimulationKernel m_testKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[0]},
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    .addSameTypeBindings(1, 2,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };



};


#endif //VULKANFLUIDSIMULATION_FLIPSOLVER_H
