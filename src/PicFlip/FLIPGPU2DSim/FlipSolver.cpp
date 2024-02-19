//
// Created by luc on 25/01/24.
//

#include "FlipSolver.h"

void FlipSolver::updateSimulation(float deltaTime) {

    uint32_t nGrid = m_cUbo.size/m_workGroupSize + 1;
    uint32_t nParticles = m_cUbo.numParticles/m_workGroupSize + 1;
    m_computeHandler.runComputeIsolated(0, [&](VkCommandBuffer commandBuffer) {
        m_advectParticlesKernel.bindAndDispatch(commandBuffer, 0, nParticles, 1, 1);

        vkb::ComputeShaderHandler::computeBarrier(commandBuffer, m_particlePosBuffer);

        m_resetGridKernel.bindAndDispatch(commandBuffer, 0, nGrid, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(commandBuffer,m_velWeightBarrier);

        m_particleToGridKernel.bindAndDispatch(commandBuffer, 0, nParticles, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(commandBuffer,m_velWeightBarrier);

        m_applyWeightsAndGravityKernel.bindAndDispatch(commandBuffer, 0, nGrid, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(commandBuffer,m_velBarrier);

        uint32_t nBound = std::max(numTilesX, numTilesY)/m_workGroupSize + 1;
        m_applyBoundaryConditionsKernel.bindAndDispatch(commandBuffer, 0, nBound, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(commandBuffer,m_velBarrier);

        m_createMatrixAndRhsKernel.bindAndDispatch(commandBuffer, 0, nGrid, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(commandBuffer, {m_matrixBuffer->getBarrierData(), m_rhsBuffer->getBarrierData()});
    });


    int res = m_pressureSolver.solve(1e-4, 500);
    if (res == 1) {
        std::cout << "Pressure solver did not converge\n";
    }

    m_computeHandler.runCompute(0, [&](VkCommandBuffer commandBuffer) {
        vkb::ComputeShaderHandler::computeBarrier(commandBuffer, m_pressureBuffer);

        m_applyPressureKernel.bindAndDispatch(commandBuffer, 0, nGrid, 1, 1);

        vkb::ComputeShaderHandler::computeBarriers(commandBuffer, m_velBarrier);

        m_gridToParticleKernel.bindAndDispatch(commandBuffer, 0, nParticles, 1, 1);
    });
}

void FlipSolver::initialize(const std::unique_ptr<vkb::DescriptorPool> &globalPool)  {
    createBuffers();
    initializeKernels(globalPool);
    initializeParticles();
}

void FlipSolver::initializeParticles() {
    std::vector<glm::vec2> particles{numParticles};
    uint32_t k = 0, na = 3, nb = 3;
    uint32_t s = numTilesX/4, e = 3*numTilesX/4;
    for (uint32_t j = 4 ; j < numTilesY-1; j++) {
        for (uint32_t i = s; i < e; i++) {
            for (uint32_t a = 0; a < na; a++){
                for (uint32_t b = 0; b < nb; b++) {
                    if (k < particles.size()) {
                        particles[k++] = glm::vec2(
                                float(i*cellSize) + float(a*cellSize)/float(na) +
                                float(cellSize)/float(na*na) + randomFloat(0.0f, 0.3f*float(cellSize)),
                                float(j*cellSize) + float(b*cellSize)/float(nb) +
                                        float(cellSize)/float(nb*nb) + randomFloat(0.0f, 0.3f*float(cellSize)));
                    }
                }
            }
        }
    }

    vkb::Buffer::writeVectorToBuffer(m_deviceRef, m_particlePosBuffer, particles);
}

void FlipSolver::initializeKernels(const std::unique_ptr<vkb::DescriptorPool> &globalPool) {
    m_advectParticlesKernel.createPipeline();
    m_resetGridKernel.createPipeline();
    m_particleToGridKernel.createPipeline();
    m_applyWeightsAndGravityKernel.createPipeline();
    m_applyBoundaryConditionsKernel.createPipeline();
    m_createMatrixAndRhsKernel.createPipeline();
    m_applyPressureKernel.createPipeline();
    m_gridToParticleKernel.createPipeline();

    m_advectParticlesKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_advectParticlesKernel.layout, {
            {m_computeUniformBuffer->descriptorInfo()},
            {m_velXBuffer->descriptorInfo()},
            {m_velYBuffer->descriptorInfo()},
            {m_particlePosBuffer->descriptorInfo()},
            {m_particleVelBuffer->descriptorInfo()},
    });

    m_resetGridKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_resetGridKernel.layout, {
            {m_computeUniformBuffer->descriptorInfo()},
            {m_typesBuffer->descriptorInfo()},
            {m_velXBuffer->descriptorInfo()},
            {m_velYBuffer->descriptorInfo()},
            {m_weightsBuffer->descriptorInfo()},
            {m_pressureBuffer->descriptorInfo()},
            {m_rhsBuffer->descriptorInfo()},

    });

    m_particleToGridKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_particleToGridKernel.layout, {
            {m_computeUniformBuffer->descriptorInfo()},
            {m_typesBuffer->descriptorInfo()},
            {m_velXBuffer->descriptorInfo()},
            {m_velYBuffer->descriptorInfo()},
            {m_weightsBuffer->descriptorInfo()},
            {m_particlePosBuffer->descriptorInfo()},
            {m_particleVelBuffer->descriptorInfo()},
    });

    m_applyWeightsAndGravityKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_applyWeightsAndGravityKernel.layout, {
            {m_computeUniformBuffer->descriptorInfo()},
            {m_typesBuffer->descriptorInfo()},
            {m_velXBuffer->descriptorInfo()},
            {m_velYBuffer->descriptorInfo()},
            {m_weightsBuffer->descriptorInfo()},
    });

    m_applyBoundaryConditionsKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_applyBoundaryConditionsKernel.layout, {
            {m_computeUniformBuffer->descriptorInfo()},
            {m_typesBuffer->descriptorInfo()},
            {m_velXBuffer->descriptorInfo()},
            {m_velYBuffer->descriptorInfo()},
    });

    m_createMatrixAndRhsKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_createMatrixAndRhsKernel.layout, {
            {m_computeUniformBuffer->descriptorInfo()},
            {m_matrixBuffer->descriptorInfo()},
            {m_typesBuffer->descriptorInfo()},
            {m_rhsBuffer->descriptorInfo()},
            {m_velXBuffer->descriptorInfo()},
            {m_velYBuffer->descriptorInfo()},
    });

    m_applyPressureKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_applyPressureKernel.layout, {
            {m_computeUniformBuffer->descriptorInfo()},
            {m_typesBuffer->descriptorInfo()},
            {m_velXBuffer->descriptorInfo()},
            {m_velYBuffer->descriptorInfo()},
            {m_pressureBuffer->descriptorInfo()},
    });

    m_gridToParticleKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_gridToParticleKernel.layout, {
            {m_computeUniformBuffer->descriptorInfo()},
            {m_typesBuffer->descriptorInfo()},
            {m_velXBuffer->descriptorInfo()},
            {m_velYBuffer->descriptorInfo()},
            {m_particlePosBuffer->descriptorInfo()},
            {m_particleVelBuffer->descriptorInfo()},
    });

    m_pressureSolver.initializeKernels(globalPool, m_computeUniformBuffer, m_matrixBuffer, m_typesBuffer, m_rhsBuffer, m_pressureBuffer);

    m_velBarrier = {m_velXBuffer->getBarrierData(), m_velYBuffer->getBarrierData()};
    m_velWeightBarrier = {m_velXBuffer->getBarrierData(), m_velYBuffer->getBarrierData(), m_weightsBuffer->getBarrierData()};

}

void FlipSolver::createBuffers() {
    m_computeUniformBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                           sizeof(ComputeUniformBufferObject),
                                                           VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                           VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

    m_computeUniformBuffer->map();
    m_computeUniformBuffer->write(&m_cUbo);

    m_matrixBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                   5*m_cUbo.size*sizeof(double),
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

    m_weightsBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                     m_cUbo.size*sizeof(glm::vec2),
                                                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                     VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                     VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    m_particlePosBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                        numParticles * sizeof(glm::vec2),
                                                    VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                    VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                    VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
                                                    VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                                                        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    m_particleVelBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                        numParticles * sizeof(glm::vec2),
                                                        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                        VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                                        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

}
