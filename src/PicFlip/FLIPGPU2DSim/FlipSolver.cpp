//
// Created by luc on 25/01/24.
//

#include "FlipSolver.h"

void FlipSolver::updateSimulation(float deltaTime) {
    std::vector<glm::vec2> p{particleCount};
    vkb::Buffer::writeBufferToVector(m_deviceRef, m_particlePosBuffer, p);
    for (auto& particle : p) {
//        std::cout << "Particle at " << particle.x << " " << particle.y << "\n";
    }

    HeapMatrix<uint32_t> types(m_cUbo.size, m_cUbo.numTilesX);
    HeapMatrix<float> velY(m_cUbo.size, m_cUbo.numTilesX);

    for (uint32_t j = 0; j < types.nRows(); j++) {
        for (uint32_t i = 0; i < types.nCols(); i++) {
            if (i == 0 || j == 0 || i == types.nCols() - 1 || j == types.nRows() - 1)
                types(i, j) = 0;
            else if (i >= 10 && i < 40 && j >= 10 && j < 40) {
                types(i, j) = 1;
                velY(i, j) = -9.8f*(1.0/60.0);
            } else {
                types(i, j) = 2;
            }
        }
    }

    vkb::Buffer::writeVectorToBuffer(m_deviceRef, m_typesBuffer, types.getVector());
    vkb::Buffer::writeVectorToBuffer(m_deviceRef, m_velYBuffer, velY.getVector());

    m_computeHandler.runComputeIsolated(0, [&](VkCommandBuffer commandBuffer) {
        m_createMatrixAndRhsKernel.bindAndDispatch(commandBuffer, 0, m_cUbo.size/m_workGroupSize + 1, 1, 1);
    });

    auto res = m_pressureSolver.solve(1e-4, 200);
    if (res == 1) {
        std::cout << "Pressure solver did not converge\n";
    }
}

void FlipSolver::initialize(const std::unique_ptr<vkb::DescriptorPool> &globalPool)  {
    createBuffers();
    initializeKernels(globalPool);
    initializeParticles();
}

void FlipSolver::initializeParticles() {
    std::vector<glm::vec2> particles{particleCount};
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
    m_createMatrixAndRhsKernel.createPipeline();

    m_createMatrixAndRhsKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_createMatrixAndRhsKernel.layout, {
            {m_computeUniformBuffer->descriptorInfo()},
            {m_matrixBuffer->descriptorInfo()},
            {m_typesBuffer->descriptorInfo()},
            {m_rhsBuffer->descriptorInfo()},
            {m_velXBuffer->descriptorInfo()},
            {m_velYBuffer->descriptorInfo()},
    });


    m_pressureSolver.initializeKernels(globalPool, m_computeUniformBuffer, m_matrixBuffer, m_typesBuffer, m_rhsBuffer, m_pressureBuffer);

}

void FlipSolver::createBuffers() {
    m_computeUniformBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                           sizeof(m_cUbo),
                                                           VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                           VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    m_computeUniformBuffer->singleWrite(&m_cUbo);

    m_matrixBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                   5*m_cUbo.size*sizeof(double),
                                                   VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                   VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                   VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                   VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    m_typesBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                  m_cUbo.size*sizeof(uint32_t),
                                                  VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                  VK_BUFFER_USAGE_TRANSFER_DST_BIT,
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
                                                    particleCount*sizeof(glm::vec2),
                                                    VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                    VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                    VK_BUFFER_USAGE_TRANSFER_SRC_BIT |
                                                    VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                                                    VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    m_particleVelBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                        particleCount*sizeof(glm::vec2),
                                                        VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                        VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                                        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

}
