//
// Created by luc on 25/01/24.
//

#include "FlipSolver.h"

void FlipSolver::updateSimulation(float deltaTime) {

    uint32_t nGrid = m_cUbo.size/m_workGroupSize + 1;
    uint32_t nParticles = m_cUbo.numParticles/m_workGroupSize + 1;
    m_computeHandler.runComputeIsolated(0, [&](VkCommandBuffer commandBuffer) {
        for (uint32_t i = 0; i < 5; i++) {
            m_advectParticlesKernel.bindAndDispatch(commandBuffer, 0, nParticles, 1, 1);

            vkb::ComputeShaderHandler::computeBarrier(commandBuffer, m_particlePosBuffer);
        }

        for (uint32_t i = 0; i < m_collideObjKernel.descSets.size(); i++) {
            m_collideObjKernel.bindAndDispatch(commandBuffer, i, nParticles, 1, 1);
            vkb::ComputeShaderHandler::computeBarrier(commandBuffer, m_particlePosBuffer);
        }

        m_resetGridKernel.bindAndDispatch(commandBuffer, 0, nGrid, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(commandBuffer,m_velWeightBarrier);

        for (uint32_t i = 0; i < m_applyObjBoundaryConditionsKernel.descSets.size(); i++) {
            m_applyObjBoundaryConditionsKernel.bindAndDispatch(commandBuffer, i, nGrid, 1, 1);
            vkb::ComputeShaderHandler::computeBarriers(commandBuffer, m_velBarrier);
        }

        m_particleToGridKernel.bindAndDispatch(commandBuffer, 0, nParticles, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(commandBuffer,m_velWeightBarrier);

        m_applyWeightsAndGravityKernel.bindAndDispatch(commandBuffer, 0, nGrid, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(commandBuffer,m_velBarrier);

        for (uint32_t k = 0; k < extensions; k++) {
            m_extendVelocitiesKernel.bindAndDispatch(commandBuffer, k, nGrid, 1, 1);
            vkb::ComputeShaderHandler::computeBarriers(commandBuffer,m_velBarrier);
        }

        uint32_t nBound = std::max(m_cUbo.dim.x*m_cUbo.dim.y, std::max(m_cUbo.dim.x*m_cUbo.dim.z, m_cUbo.dim.y*m_cUbo.dim.z))/m_workGroupSize + 1;
        m_applyBoundaryConditionsKernel.bindAndDispatch(commandBuffer, 0, nBound, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(commandBuffer,m_velBarrier);

        for (uint32_t i = 0; i < m_applyObjBoundaryConditionsKernel.descSets.size(); i++) {
            m_applyObjBoundaryConditionsKernel.bindAndDispatch(commandBuffer, i, nGrid, 1, 1);
            vkb::ComputeShaderHandler::computeBarriers(commandBuffer, m_velBarrier);
        }

        m_setPrevVelocitiesKernel.bindAndDispatch(commandBuffer, 0, nGrid, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(commandBuffer,m_velBarrier);

        m_createMatrixAndRhsKernel.bindAndDispatch(commandBuffer, 0, nGrid, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(commandBuffer, {m_matrixBuffer->getBarrierData(), m_rhsBuffer->getBarrierData()});
    });


    int res = m_pressureSolver.solve(1e-4, numIterations);
    if (res == 1) {
        std::cout << "Pressure solver did not converge\n";
    }

    m_computeHandler.runCompute(0, [&](VkCommandBuffer commandBuffer) {
        vkb::ComputeShaderHandler::computeBarrier(commandBuffer, m_pressureBuffer);

        m_applyPressureKernel.bindAndDispatch(commandBuffer, 0, nGrid, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(commandBuffer, m_velBarrier);

        for (uint32_t k = 0; k < extensions; k++) {
            m_extendVelocitiesKernel.bindAndDispatch(commandBuffer, k, nGrid, 1, 1);
            vkb::ComputeShaderHandler::computeBarriers(commandBuffer,m_velBarrier);
        }

        m_gridToParticleKernel.bindAndDispatch(commandBuffer, 0, nParticles, 1, 1);
    });
}

void FlipSolver::updateUniformBuffers(uint32_t numParticles, glm::vec3 boxSize, float cellSize, float flipRatio, double w) {

    m_cUbo.numParticles = numParticles;
    if (cellSize > 0) {
        m_cUbo.cellSize = cellSize;
        m_cUbo.overCellSize = 1/cellSize;
    }
    m_cUbo.dim = glm::vec<3, uint32_t>(boxSize/m_cUbo.cellSize);
    if (flipRatio > 0) m_cUbo.flipRatio = flipRatio;
    m_cUbo.size = m_cUbo.dim.x*m_cUbo.dim.y*m_cUbo.dim.z;
    if (w > 0) m_cUbo.w = w;

    std::vector<ComputeUniformBufferObject> uboVec = {m_cUbo};
    vkb::Buffer::writeVectorToBuffer(m_deviceRef, m_computeUniformBuffer, uboVec);

}

void FlipSolver::initialize(const std::unique_ptr<vkb::DescriptorPool> &globalPool, const std::vector<RigidObject>& sceneObjectBuffers, bool dislocatePos)  {
    createBuffers();
    initializeKernels(globalPool, sceneObjectBuffers);
    initializeParticles(dislocatePos);
}

void FlipSolver::initializeParticles(bool dislocatePos) {
    std::vector<glm::vec4> particles{m_cUbo.numParticles};
    uint32_t p = 0;
    for (uint32_t j = particleStart.y; j < particleStart.y + particleSpan.y; j++) {
        for (uint32_t k = particleStart.z; k < particleStart.z + particleSpan.z; k++) {
            for (uint32_t i = particleStart.x; i < particleStart.x + particleSpan.x; i++) {
                for (uint32_t a = 0; a < particlePerCell.x; a++) {
                    for (uint32_t b = 0; b < particlePerCell.y; b++) {
                        for (uint32_t c = 0; c < particlePerCell.z; c++) {
                            if (p < particles.size()) {
                                particles[p++] = glm::vec4(
                                        float(i) * m_cUbo.cellSize + float(a) * m_cUbo.cellSize / float(particlePerCell.x) +
                                        m_cUbo.cellSize / float(particlePerCell.x * particlePerCell.x),
                                        float(j) * m_cUbo.cellSize + float(b) * m_cUbo.cellSize / float(particlePerCell.y) +
                                        m_cUbo.cellSize / float(particlePerCell.y * particlePerCell.y),
                                        float(k) * m_cUbo.cellSize + float(c) * m_cUbo.cellSize / float(particlePerCell.z) +
                                        m_cUbo.cellSize / float(particlePerCell.z * particlePerCell.z),
                                        0.0f);
                                if (dislocatePos) particles[p-1] += glm::vec4(
                                        randomFloat(0.0f, 0.3f * m_cUbo.cellSize),
                                        randomFloat(0.0f, 0.3f * m_cUbo.cellSize),
                                        randomFloat(0.0f, 0.3f * m_cUbo.cellSize),
                                        0.0f);
                            }
                        }
                    }
                }
            }
        }
    }

//    if (p < numParticles) {
//        throw std::runtime_error("Too many particles to fit in grid");
//    }


    vkb::Buffer::writeVectorToBuffer(m_deviceRef, m_particlePosBuffer, particles);
}

void FlipSolver::initializeKernels(const std::unique_ptr<vkb::DescriptorPool> &globalPool, const std::vector<RigidObject>& sceneObjectBuffers) {
    m_advectParticlesKernel.createPipeline();
    m_collideObjKernel.createPipeline();
    m_resetGridKernel.createPipeline();
    m_particleToGridKernel.createPipeline();
    m_applyWeightsAndGravityKernel.createPipeline();
    m_extendVelocitiesKernel.createPipeline();
    m_applyBoundaryConditionsKernel.createPipeline();
    m_applyObjBoundaryConditionsKernel.createPipeline();
    m_setPrevVelocitiesKernel.createPipeline();
    m_createMatrixAndRhsKernel.createPipeline();
    m_applyPressureKernel.createPipeline();
    m_gridToParticleKernel.createPipeline();

    if (kernelsInitialized) {
        std::vector<VkDescriptorSet> setsToFree = {
                m_advectParticlesKernel.descSets[0], m_resetGridKernel.descSets[0], m_particleToGridKernel.descSets[0],
                m_applyWeightsAndGravityKernel.descSets[0], m_applyBoundaryConditionsKernel.descSets[0],
                m_setPrevVelocitiesKernel.descSets[0], m_createMatrixAndRhsKernel.descSets[0],
                m_applyPressureKernel.descSets[0], m_gridToParticleKernel.descSets[0]
        };
        for (uint32_t k = 0; k < maxExtensions; k++) {
            setsToFree.push_back(m_extendVelocitiesKernel.descSets[k]);
        }

        auto pressureSets = m_pressureSolver.activeDescriptorSets();

        setsToFree.insert(setsToFree.end(), pressureSets.begin(), pressureSets.end());

        globalPool->freeDescriptors(setsToFree);
    }

    kernelsInitialized = true;

    m_advectParticlesKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_advectParticlesKernel.layout, {
            {m_computeUniformBuffer->descriptorInfo()},
            {m_velXBuffer->descriptorInfo()},
            {m_velYBuffer->descriptorInfo()},
            {m_velZBuffer->descriptorInfo()},
            {m_particlePosBuffer->descriptorInfo()},
            {m_particleVelBuffer->descriptorInfo()},
    });

    m_collideObjKernel.descSets.clear();
    for (auto& obj : sceneObjectBuffers) {
        m_collideObjKernel.descSets.push_back(vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_collideObjKernel.layout, {
                {m_computeUniformBuffer->descriptorInfo()},
                {m_particlePosBuffer->descriptorInfo()},
                {obj.getSdfInfo()},
        }));
    }

    m_resetGridKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_resetGridKernel.layout, {
            {m_computeUniformBuffer->descriptorInfo()},
            {m_typesBuffer->descriptorInfo()},
            {m_velXBuffer->descriptorInfo()},
            {m_velYBuffer->descriptorInfo()},
            {m_velZBuffer->descriptorInfo()},
            {m_weightsBuffer->descriptorInfo()},
            {m_pressureBuffer->descriptorInfo()},
            {m_rhsBuffer->descriptorInfo()},
            {m_hasVelBuffer->descriptorInfo()},

    });

    m_particleToGridKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_particleToGridKernel.layout, {
            {m_computeUniformBuffer->descriptorInfo()},
            {m_typesBuffer->descriptorInfo()},
            {m_velXBuffer->descriptorInfo()},
            {m_velYBuffer->descriptorInfo()},
            {m_velZBuffer->descriptorInfo()},
            {m_weightsBuffer->descriptorInfo()},
            {m_particlePosBuffer->descriptorInfo()},
            {m_particleVelBuffer->descriptorInfo()},
            {m_hasVelBuffer->descriptorInfo()},
    });

    m_applyWeightsAndGravityKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_applyWeightsAndGravityKernel.layout, {
            {m_computeUniformBuffer->descriptorInfo()},
            {m_typesBuffer->descriptorInfo()},
            {m_velXBuffer->descriptorInfo()},
            {m_velYBuffer->descriptorInfo()},
            {m_velZBuffer->descriptorInfo()},
            {m_weightsBuffer->descriptorInfo()},
    });

    for (uint32_t k = 0; k < maxExtensions; k++) {
        m_extendVelocitiesKernel.descSets[k] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_extendVelocitiesKernel.layout, {
                {m_extensionUniformBuffers[k]->descriptorInfo()},
                {m_typesBuffer->descriptorInfo()},
                {m_velXBuffer->descriptorInfo()},
                {m_velYBuffer->descriptorInfo()},
                {m_velZBuffer->descriptorInfo()},
                {m_hasVelBuffer->descriptorInfo()},
        });
    }

    m_applyBoundaryConditionsKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_applyBoundaryConditionsKernel.layout, {
            {m_computeUniformBuffer->descriptorInfo()},
            {m_typesBuffer->descriptorInfo()},
            {m_velXBuffer->descriptorInfo()},
            {m_velYBuffer->descriptorInfo()},
            {m_velZBuffer->descriptorInfo()},
    });

    m_applyObjBoundaryConditionsKernel.descSets.clear();
    for (auto& obj : sceneObjectBuffers) {
        m_applyObjBoundaryConditionsKernel.descSets.push_back(vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_applyObjBoundaryConditionsKernel.layout, {
                {m_computeUniformBuffer->descriptorInfo()},
                {m_typesBuffer->descriptorInfo()},
                {m_velXBuffer->descriptorInfo()},
                {m_velYBuffer->descriptorInfo()},
                {m_velZBuffer->descriptorInfo()},
                {obj.getSdfInfo()},
        }));
    }

    m_setPrevVelocitiesKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_setPrevVelocitiesKernel.layout, {
            {m_computeUniformBuffer->descriptorInfo()},
            {m_velXBuffer->descriptorInfo()},
            {m_velYBuffer->descriptorInfo()},
            {m_velZBuffer->descriptorInfo()},
            {m_prevVelXBuffer->descriptorInfo()},
            {m_prevVelYBuffer->descriptorInfo()},
            {m_prevVelZBuffer->descriptorInfo()},
    });

    m_createMatrixAndRhsKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_createMatrixAndRhsKernel.layout, {
            {m_computeUniformBuffer->descriptorInfo()},
            {m_matrixBuffer->descriptorInfo()},
            {m_typesBuffer->descriptorInfo()},
            {m_rhsBuffer->descriptorInfo()},
            {m_velXBuffer->descriptorInfo()},
            {m_velYBuffer->descriptorInfo()},
            {m_velZBuffer->descriptorInfo()},
    });

    m_applyPressureKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_applyPressureKernel.layout, {
            {m_computeUniformBuffer->descriptorInfo()},
            {m_typesBuffer->descriptorInfo()},
            {m_velXBuffer->descriptorInfo()},
            {m_velYBuffer->descriptorInfo()},
            {m_velZBuffer->descriptorInfo()},
            {m_pressureBuffer->descriptorInfo()},
    });

    m_gridToParticleKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_gridToParticleKernel.layout, {
            {m_computeUniformBuffer->descriptorInfo()},
            {m_velXBuffer->descriptorInfo()},
            {m_velYBuffer->descriptorInfo()},
            {m_velZBuffer->descriptorInfo()},
            {m_prevVelXBuffer->descriptorInfo()},
            {m_prevVelYBuffer->descriptorInfo()},
            {m_prevVelZBuffer->descriptorInfo()},
            {m_particlePosBuffer->descriptorInfo()},
            {m_particleVelBuffer->descriptorInfo()},
    });

    m_pressureSolver.initializeKernels(globalPool, m_computeUniformBuffer, m_matrixBuffer, m_typesBuffer, m_rhsBuffer, m_pressureBuffer);

    m_velBarrier = {m_velXBuffer->getBarrierData(), m_velYBuffer->getBarrierData(), m_velZBuffer->getBarrierData()};
    m_velWeightBarrier = {m_velXBuffer->getBarrierData(), m_velYBuffer->getBarrierData(), m_velZBuffer->getBarrierData(), m_weightsBuffer->getBarrierData()};

}

void FlipSolver::createBuffers() {
    for (uint32_t k = 0; k < maxExtensions; k++) {
        std::vector<ExtensionUBO> extUbo = {{m_cUbo.size, k + 1, m_cUbo.dim}};
        m_extensionUniformBuffers[k] = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                                     sizeof(ExtensionUBO),
                                                                     VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                                                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        vkb::Buffer::writeVectorToBuffer(m_deviceRef, m_extensionUniformBuffers[k], extUbo);
    }

    m_computeUniformBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                           sizeof(ComputeUniformBufferObject),
                                                           VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                                           VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    std::vector<ComputeUniformBufferObject> uboVec = {m_cUbo};
    vkb::Buffer::writeVectorToBuffer(m_deviceRef, m_computeUniformBuffer, uboVec);

    m_matrixBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                   7*m_cUbo.size*sizeof(double),
                                                   VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                   VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                   VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                   VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    m_typesBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                  m_cUbo.size*sizeof(uint32_t),
                                                  VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                  VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                  VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                  VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    m_hasVelBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                   m_cUbo.size*sizeof(uint32_t),
                                                   VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                   VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                   VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                   VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);


    m_rhsBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                m_cUbo.size*sizeof(double),
                                                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    m_pressureBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                     m_cUbo.size*sizeof(double),
                                                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                     VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                     VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    m_velXBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                 m_cUbo.size*sizeof(float),
                                                 VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                 VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                 VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    m_velYBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                 m_cUbo.size*sizeof(float),
                                                 VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                 VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                 VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    m_velZBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                 m_cUbo.size*sizeof(float),
                                                 VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                 VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                 VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    m_prevVelXBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                 m_cUbo.size*sizeof(float),
                                                 VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                 VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                 VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    m_prevVelYBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                     m_cUbo.size*sizeof(float),
                                                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                     VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                     VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    m_prevVelZBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                     m_cUbo.size*sizeof(float),
                                                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                     VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                     VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);


    m_weightsBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                     m_cUbo.size*sizeof(glm::vec4),
                                                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                     VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                     VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    m_particlePosBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                        m_cUbo.numParticles * sizeof(glm::vec4),
                                                    VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                    VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                    VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
                                                    VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                                                        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    m_particleVelBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                        m_cUbo.numParticles * sizeof(glm::vec4),
                                                        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                        VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                        VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    m_pressureSolver.createBuffers();
}
