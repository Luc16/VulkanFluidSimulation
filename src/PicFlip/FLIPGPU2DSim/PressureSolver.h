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
#include "FLIPGPU2DSim.h"

class PressureSolver {

    PressureSolver(const vkb::Device& device, const std::vector<std::string>& shaders);

public:
    int solve();

    void initializeKernels(const std::unique_ptr<vkb::DescriptorPool> &globalPool,
                           const std::unique_ptr<vkb::Buffer> &matrixBuffer,
                           const std::unique_ptr<vkb::Buffer> &typesBuffer,
                           const std::unique_ptr<vkb::Buffer> &residualBuffer,
                           const std::unique_ptr<vkb::Buffer> &pressureBuffer);
private:
    constexpr static uint32_t m_workGroupSize = FLIPGPU2DSim::workGroupSize;

    void createBuffers();
    void applyDotProductKernel(VkCommandBuffer commandBuffer,uint32_t dotProductDescSetIdx, uint32_t resDescSetIdx);

    const vkb::Device& m_deviceRef;
    const std::vector<std::string>& m_shaderPaths;

    ComputeUniformBufferObject m_cUbo{};
    std::unique_ptr<vkb::Buffer> m_computeUniformBuffer;
    vkb::ComputeShaderHandler m_computeHandler{m_deviceRef};

    std::unique_ptr<vkb::Buffer> m_directionBuffer;
    std::unique_ptr<vkb::Buffer> m_auxBuffer;
    std::unique_ptr<vkb::Buffer> m_alphaBuffer;
    std::vector<std::unique_ptr<vkb::Buffer>> m_dotProductAuxBuffers;

    vkb::SimulationKernel m_addScaledKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[0]},
            .descSets = std::vector<VkDescriptorSet>(1),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                            // result buffer, unscaled buffer, constant, buffer to be scaled
                    .addSameTypeBindings(1, 4,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_dotProductKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[1]},
            .descSets = std::vector<VkDescriptorSet>(1),
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
