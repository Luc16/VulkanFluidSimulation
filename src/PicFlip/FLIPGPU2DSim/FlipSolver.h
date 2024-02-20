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
    FlipSolver(const vkb::Device& device, const std::vector<std::string>& shaders,
               const std::vector<std::string>& pressureSolverShaders, uint32_t width, uint32_t height):
               m_deviceRef(device),
               m_shaderPaths(shaders),
               m_pressureSolverShaderPaths(pressureSolverShaders),
               cellSize(10),
               numTilesX(width/cellSize),
               numTilesY(height/cellSize){}

    void updateSimulation(float deltaTime);
    void initialize(const std::unique_ptr<vkb::DescriptorPool> &globalPool);
    [[nodiscard]] uint32_t getNumTilesX() const { return numTilesX; }
    [[nodiscard]] uint32_t getNumTilesY() const { return numTilesY; }
    [[nodiscard]] uint32_t getCellSize() const { return cellSize; }
    [[nodiscard]] uint32_t getParticleCount() const { return numParticles; }
    [[nodiscard]] float particleRadius() const { return radius; }
    [[nodiscard]] VkBuffer particleBuffer() const { return m_particlePosBuffer->getBuffer(); }
    [[nodiscard]] std::vector<VkSemaphore> computeSemaphore() { return m_computeHandler.currentSemaphore(0); }

    float flipRatio = 0.90f;

private:
    constexpr static uint32_t m_workGroupSize = workGroupSize;

    // simulation params
    uint32_t numParticles = 20000;
    float dt = 1/60.0f;
    float radius = 4.0f;
    uint32_t cellSize;
    uint32_t numTilesX;
    uint32_t numTilesY;
    uint32_t numIterations = 200;
    uint32_t extensions = 4;

    void createBuffers();
    void initializeKernels(const std::unique_ptr<vkb::DescriptorPool> &globalPool);
    void initializeParticles();

    const vkb::Device& m_deviceRef;
    const std::vector<std::string>& m_shaderPaths;
    const std::vector<std::string>& m_pressureSolverShaderPaths;

    ComputeUniformBufferObject m_cUbo{
        numTilesX*numTilesY,
        numParticles,
        1.0f/float(cellSize),
        float(cellSize),
        dt,
        flipRatio,
        {numTilesX, numTilesY}
    };
    std::unique_ptr<vkb::Buffer> m_computeUniformBuffer;
    vkb::ComputeShaderHandler m_computeHandler{m_deviceRef};

    std::unique_ptr<vkb::Buffer> m_matrixBuffer;
    std::unique_ptr<vkb::Buffer> m_typesBuffer;
    std::unique_ptr<vkb::Buffer> m_hasVelBuffer;
    std::unique_ptr<vkb::Buffer> m_rhsBuffer;
    std::unique_ptr<vkb::Buffer> m_pressureBuffer;
    std::unique_ptr<vkb::Buffer> m_velXBuffer;
    std::unique_ptr<vkb::Buffer> m_velYBuffer;
    std::unique_ptr<vkb::Buffer> m_prevVelXBuffer;
    std::unique_ptr<vkb::Buffer> m_prevVelYBuffer;
    std::unique_ptr<vkb::Buffer> m_weightsBuffer;
    std::unique_ptr<vkb::Buffer> m_particlePosBuffer;
    std::unique_ptr<vkb::Buffer> m_particleVelBuffer;

    std::vector<std::pair<VkBuffer, VkDeviceSize>> m_velBarrier, m_velWeightBarrier;

    PressureSolver m_pressureSolver{m_deviceRef, m_pressureSolverShaderPaths, m_cUbo.size};

    vkb::SimulationKernel m_advectParticlesKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[0]},
            .descSets = std::vector<VkDescriptorSet>(1),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    //velX, velY, pPos, pVel
                    .addSameTypeBindings(1, 4,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_resetGridKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[1]},
            .descSets = std::vector<VkDescriptorSet>(1),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // types, velX, velY, weights, pressure, rhs, hasVel
                    .addSameTypeBindings(1, 7,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_particleToGridKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[2]},
            .descSets = std::vector<VkDescriptorSet>(1),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // types, velX, velY, weights, pPos, pVel, hasVel
                    .addSameTypeBindings(1, 7,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_applyWeightsAndGravityKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[3]},
            .descSets = std::vector<VkDescriptorSet>(1),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // types, velX, velY, weights
                    .addSameTypeBindings(1, 4,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_extendVelocitiesKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[4]},
            .descSets = std::vector<VkDescriptorSet>(1),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // types, velX, velY, hasVel
                    .addSameTypeBindings(1, 4,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_applyBoundaryConditionsKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[5]},
            .descSets = std::vector<VkDescriptorSet>(1),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // types, velX, velY
                    .addSameTypeBindings(1, 3,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_setPrevVelocitiesKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[6]},
            .descSets = std::vector<VkDescriptorSet>(1),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // velX, velY, prevVelX, prevVelY
                    .addSameTypeBindings(1, 4,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_createMatrixAndRhsKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[7]},
            .descSets = std::vector<VkDescriptorSet>(1),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // matrix, types, rhs, velX, velY
                    .addSameTypeBindings(1, 5,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_applyPressureKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[8]},
            .descSets = std::vector<VkDescriptorSet>(1),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // types, velX, velY, pressure
                    .addSameTypeBindings(1, 4,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_gridToParticleKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[9]},
            .descSets = std::vector<VkDescriptorSet>(1),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // velX, velY, prevVelX, prevVelY, pPos, pVel
                    .addSameTypeBindings(1, 6,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };
};


#endif //VULKANFLUIDSIMULATION_FLIPSOLVER_H
