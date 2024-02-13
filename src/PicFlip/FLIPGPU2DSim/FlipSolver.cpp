//
// Created by luc on 25/01/24.
//

#include "FlipSolver.h"

void FlipSolver::updateSimulation(float deltaTime, float flipRatio) {
    auto printVec = [](const std::vector<double> &vec) {
        for (auto &v: vec) {
            std::cout << v << ", ";
        }
        std::cout << "\n";
    };
    auto printMatrix = []<typename T>(const HeapMatrix<T> &matrix) {
        for (uint32_t j = 0; j < matrix.nRows(); j++) {
            std::cout << "Row " << j << ": ";
            for (uint32_t i = 0; i < matrix.nCols(); i++) {
                std::cout << matrix(i, j) << ", ";
            }
            std::cout << "\n";
        }
    };

    HeapMatrix<uint32_t> types(m_cUbo.size, m_cUbo.numTilesX);
    HeapMatrix<double> velY(m_cUbo.size, m_cUbo.numTilesX);

    for (uint32_t j = 0; j < types.nRows(); j++) {
        for (uint32_t i = 0; i < types.nCols(); i++) {
            if (i == 0 || j == 0 || i == types.nCols() - 1 || j == types.nRows() - 1)
                types(i, j) = 0;
            else if (i >= 10 && i < 40 && j >= 10 && j < 40) {
                types(i, j) = 1;
                velY(i, j) = -9.8*(1.0/60.0);
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

    vkb::Buffer::writeBufferToVector(m_deviceRef, m_rhsBuffer, velY.getVector());

    auto res = m_pressureSolver.solve(1e-4, 200);
    if (res == 1) {
        std::cout << "Pressure solver did not converge\n";
    }
}

void FlipSolver::initialize(const std::unique_ptr<vkb::DescriptorPool> &globalPool)  {
    m_createMatrixAndRhsKernel.createPipeline();

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
                                                     m_cUbo.size*sizeof(double),
                                                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                     VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                     VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    m_velYBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                     m_cUbo.size*sizeof(double),
                                                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                     VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                     VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);


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
