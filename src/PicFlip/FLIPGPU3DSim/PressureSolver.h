//
// Created by luc on 23/01/24.
//

#ifndef VULKANFLUIDSIMULATION_PRESSURESOLVER_H
#define VULKANFLUIDSIMULATION_PRESSURESOLVER_H
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

#include <cstdint>
#include <vector>
#include "../../lib/graphicsDataStructures/Matrices.h"

class PressureSolver {
public:

    PressureSolver(const vkb::Device &device, const std::vector<std::string> &shaders, uint32_t size);

    int solve(double epsilon, uint32_t maxIters);

    void initializeKernels(const std::unique_ptr<vkb::DescriptorPool> &globalPool,
                           const std::unique_ptr<vkb::Buffer>& uniformBuffer,
                           const std::unique_ptr<vkb::Buffer> &matrixBuffer,
                           const std::unique_ptr<vkb::Buffer> &typesBuffer,
                           const std::unique_ptr<vkb::Buffer> &residualBuffer,
                           const std::unique_ptr<vkb::Buffer> &pressureBuffer);
private:
    constexpr static uint32_t m_workGroupSize = workGroupSize;
    struct PressureSolverUniformBufferObject {
        uint32_t size;
        uint32_t extra;
    };

    void createBuffers();
    void applyDotProductKernel(VkCommandBuffer commandBuffer,uint32_t dotProductDescSetIdx, uint32_t resDescSetIdx);

    const vkb::Device& m_deviceRef;
    const std::vector<std::string>& m_shaderPaths;
    const uint32_t m_size;

    std::array<PressureSolverUniformBufferObject, 3> m_pUbos{};
    std::array<std::unique_ptr<vkb::Buffer>, 3> m_pressureSolverUniformBuffer;
    vkb::ComputeShaderHandler m_computeHandler{m_deviceRef};

    std::unique_ptr<vkb::Buffer> m_directionBuffer;
    std::unique_ptr<vkb::Buffer> m_auxBuffer;
    std::unique_ptr<vkb::Buffer> m_alphaBuffer;
    std::unique_ptr<vkb::Buffer> m_gammaBuffer;
    std::unique_ptr<vkb::Buffer> m_betaBuffer;
    std::vector<std::unique_ptr<vkb::Buffer>> m_dotProductAuxBuffers;
    std::vector<std::pair<VkBuffer, VkDeviceSize>> m_externalBarriers;

    vkb::SimulationKernel m_addScaledKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[0]},
            .descSets = std::vector<VkDescriptorSet>(3),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                            // result buffer, unscaled buffer, constant, buffer to be scaled
                    .addSameTypeBindings(1, 4,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_dotProductKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[1]},
            .descSets = std::vector<VkDescriptorSet>(2),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                            // result buffer, vec1, vec2
                    .addSameTypeBindings(1, 3,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_reduceKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[2]},
            .descSets = std::vector<VkDescriptorSet>(),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // result buffer, vec1
                    .addSameTypeBindings(1, 2,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_finishDotKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[3]},
            .descSets = std::vector<VkDescriptorSet>(3),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // result buffer, vec1, constant
                    .addSameTypeBindings(1, 3,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_matrixMultiplyKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[4]},
            .descSets = std::vector<VkDescriptorSet>(1),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // matrix, types, x, res
                    .addSameTypeBindings(1, 4,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

};
#endif //VULKANFLUIDSIMULATION_PRESSURESOLVER_H
