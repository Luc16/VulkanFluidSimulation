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
#include "../../lib/descriptors/DescriptorPool.h"
#include "../../lib/descriptors/DescriptorWriter.h"
#include "../../lib/graphicsDataStructures/Matrices.h"


class FlipSolver {
public:
    FlipSolver(const vkb::Device& device, const std::vector<std::string>& shaders): m_deviceRef(device), m_shaderPaths(shaders) {}

    void updateSimulation(float deltaTime, float flipRatio);
    void initialize(const std::unique_ptr<vkb::DescriptorPool> &globalPool);

private:
    constexpr static uint32_t m_workGroupSize = workGroupSize;

    const vkb::Device& m_deviceRef;
    const std::vector<std::string>& m_shaderPaths;

    ComputeUniformBufferObject m_cUbo{20*20, 20};
    std::unique_ptr<vkb::Buffer> m_computeUniformBuffer;
    vkb::ComputeShaderHandler m_computeHandler{m_deviceRef};

    std::unique_ptr<vkb::Buffer> m_matrixBuffer;
    std::unique_ptr<vkb::Buffer> m_typesBuffer;
    std::unique_ptr<vkb::Buffer> m_rhsBuffer;
    std::unique_ptr<vkb::Buffer> m_pressureBuffer;

    PressureSolver m_pressureSolver{m_deviceRef, m_shaderPaths, m_cUbo.size};

    vkb::SimulationKernel m_createMatrixAndRhsKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[5]},
            .descSets = std::vector<VkDescriptorSet>(1),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // matrix, types, rhs, velX, velY
                    .addSameTypeBindings(1, 5,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };


};


#endif //VULKANFLUIDSIMULATION_FLIPSOLVER_H
