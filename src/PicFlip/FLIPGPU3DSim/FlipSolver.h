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
#include "RigidObject.h"
#include "FlipInitializer.h"
#include "FlipSceneManager.h"


class FlipSolver {
public:
    FlipSolver(const vkb::Device& device, const std::vector<std::string>& shaders,
               const std::vector<std::string>& pressureSolverShaders, glm::vec<3, float> boxSize, float _cellSize = 0.1):
               m_deviceRef(device),
               m_shaderPaths(shaders),
               m_pressureSolverShaderPaths(pressureSolverShaders),
               m_boxSize(boxSize),
               m_cUbo({ uint32_t (boxSize.x * boxSize.y * boxSize.z / (_cellSize * _cellSize * _cellSize)),
                        200'000,
                        1.0f / _cellSize,
                        _cellSize,
                        1 / 60.0f,
                        0.95f,
                        0.0f,
                        {boxSize.x / _cellSize, boxSize.y / _cellSize, boxSize.z / _cellSize},
                        {boxSize},
                        0.85}
                       ) {
        m_maxParticles = m_cUbo.numParticles;
    }

    void updateSimulation(float deltaTime);
    void initialize(const std::unique_ptr<vkb::DescriptorPool> &globalPool, std::vector<RigidObject>& sceneObjectBuffers, const std::string& scene, bool dislocatePos = true);
    void updateUniformBuffers(uint32_t numParticles, glm::vec3 boxSize, float cellSize = -1, float flipRatio = -1, double w = -1.0);

    [[nodiscard]] uint32_t getNumTilesX() const { return m_cUbo.dim.x; }
    [[nodiscard]] uint32_t getNumTilesY() const { return m_cUbo.dim.y; }
    [[nodiscard]] uint32_t getNumTilesZ() const { return m_cUbo.dim.z; }
    [[nodiscard]] float getCellSize() const { return m_cUbo.cellSize; }
    [[nodiscard]] glm::ivec3 getDimension() const { return m_cUbo.dim; }
    [[nodiscard]] glm::vec3 getBoxSize() const { return m_cUbo.boxSize; }
    [[nodiscard]] uint32_t getParticleCount() const { return m_cUbo.numParticles; }
    [[nodiscard]] uint32_t getCellCount() const { return m_cUbo.size; }
    [[nodiscard]] VkBuffer particleBuffer() const { return m_particlePosBuffer->getBuffer(); }
    [[nodiscard]] std::vector<VkSemaphore> computeSemaphore() { return m_computeHandler.currentSemaphore(0); }

    uint32_t extensions = 0;
    const uint32_t maxExtensions = 4;

private:
    constexpr static uint32_t m_workGroupSize = workGroupSize;
    struct ParticleData {
        std::vector<glm::vec4> positions;
        std::vector<glm::vec4> velocities;

        void resize(size_t newSize) {
            positions.resize(newSize);
            velocities.resize(newSize);
        }
    };

    // simulation params
    uint32_t m_numIterations = 200;
    uint32_t m_maxParticles;
    uint32_t m_particlesToAdd = 0;
    bool m_kernelsInitialized = false;
    std::string m_scene;
    glm::vec3 m_boxSize;
    uint32_t m_initialParticles{};
    ParticleData m_particleData{};

    void createBuffers();
    void initializeKernels(const std::unique_ptr<vkb::DescriptorPool> &globalPool, const std::vector<RigidObject>& sceneObjectBuffers);
    void initializeParticles(const std::string &scene);
    void updateWall();

    const vkb::Device& m_deviceRef;
    const std::vector<std::string>& m_shaderPaths;
    const std::vector<std::string>& m_pressureSolverShaderPaths;
    const FlipInitializer m_initializer{glm::vec3(3)};

    struct ExtensionUBO {
        uint32_t size;
        uint32_t k;
        alignas(16) glm::ivec3 dim;

    };

    ComputeUniformBufferObject m_cUbo;
    std::unique_ptr<vkb::Buffer> m_computeUniformBuffer;
    std::vector<std::unique_ptr<vkb::Buffer>> m_extensionUniformBuffers{maxExtensions};

    vkb::ComputeShaderHandler m_computeHandler{m_deviceRef};

    std::unique_ptr<vkb::Buffer> m_matrixBuffer;
    std::unique_ptr<vkb::Buffer> m_typesBuffer;
    std::unique_ptr<vkb::Buffer> m_hasVelBuffer;
    std::unique_ptr<vkb::Buffer> m_rhsBuffer;
    std::unique_ptr<vkb::Buffer> m_pressureBuffer;
    std::unique_ptr<vkb::Buffer> m_velXBuffer;
    std::unique_ptr<vkb::Buffer> m_velYBuffer;
    std::unique_ptr<vkb::Buffer> m_velZBuffer;
    std::unique_ptr<vkb::Buffer> m_prevVelXBuffer;
    std::unique_ptr<vkb::Buffer> m_prevVelYBuffer;
    std::unique_ptr<vkb::Buffer> m_prevVelZBuffer;
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
                    //velX, velY, velZ, pPos, pVel
                    .addSameTypeBindings(1, 5,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_collideObjKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[1]},
            .descSets = std::vector<VkDescriptorSet>(0),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    //pPos, sdf
                    .addSameTypeBindings(1, 3,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_resetGridKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[2]},
            .descSets = std::vector<VkDescriptorSet>(1),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // types, velX, velY, velZ, weights, pressure, rhs, hasVel
                    .addSameTypeBindings(1, 8,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_particleToGridKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[3]},
            .descSets = std::vector<VkDescriptorSet>(1),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // types, velX, velY, velZ, weights, pPos, pVel, hasVel
                    .addSameTypeBindings(1, 8,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_applyWeightsAndGravityKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[4]},
            .descSets = std::vector<VkDescriptorSet>(1),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // types, velX, velY, velZ, weights
                    .addSameTypeBindings(1, 5,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_extendVelocitiesKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[5]},
            .descSets = std::vector<VkDescriptorSet>(maxExtensions),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // types, velX, velY, velZ, hasVel
                    .addSameTypeBindings(1, 5,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_applyBoundaryConditionsKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[6]},
            .descSets = std::vector<VkDescriptorSet>(1),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // types, velX, velY, velZ
                    .addSameTypeBindings(1, 4,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_applyObjBoundaryConditionsKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[7]},
            .descSets = std::vector<VkDescriptorSet>(1),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                            // types, velX, velY, velZ, sdf
                    .addSameTypeBindings(1, 5,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_setPrevVelocitiesKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[8]},
            .descSets = std::vector<VkDescriptorSet>(1),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // velX, velY, velZ, prevVelX, prevVelY, prevVelZ
                    .addSameTypeBindings(1, 6,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_createMatrixAndRhsKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[9]},
            .descSets = std::vector<VkDescriptorSet>(1),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // matrix, types, rhs, velX, velY, velZ
                    .addSameTypeBindings(1, 6,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_applyPressureKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[10]},
            .descSets = std::vector<VkDescriptorSet>(1),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // types, velX, velY, velZ, pressure
                    .addSameTypeBindings(1, 5,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    vkb::SimulationKernel m_gridToParticleKernel {
            .computeSystem{m_deviceRef, m_shaderPaths[11]},
            .descSets = std::vector<VkDescriptorSet>(1),
            .layout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                    .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                    // velX, velY, velZ, prevVelX, prevVelY, prevVelZ, pPos, pVel
                    .addSameTypeBindings(1, 8,VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, VK_SHADER_STAGE_COMPUTE_BIT)
                    .build()
    };

    uint32_t frame = 0;
    float accTime = 0;

public:
    bool activateWaves = false;
    float wallForwardSpeed = 0.04f * m_cUbo.cellSize / m_cUbo.dt, wallBackwardSpeed = 0.01f * m_cUbo.cellSize / m_cUbo.dt, wallLimit = 0.2f * m_cUbo.boxSize.x, curSpeed = 0.0f;

    friend class FlipSceneManager;
};


#endif //VULKANFLUIDSIMULATION_FLIPSOLVER_H
