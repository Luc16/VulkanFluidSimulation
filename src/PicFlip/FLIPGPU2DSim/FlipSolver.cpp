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
            else if (i >= 6 && i < 14 && j >= 6 && j < 14) {
                types(i, j) = 1;
                velY(i, j) = -9.8*(1.0/60.0);
            } else {
                types(i, j) = 2;
            }
        }
    }

    printMatrix(types);
    printMatrix(velY);

    vkb::Buffer::writeVectorToBuffer(m_deviceRef, m_typesBuffer, types.getVector());
    vkb::Buffer::writeVectorToBuffer(m_deviceRef, m_auxBuffer, velY.getVector());

    m_computeHandler.runComputeIsolated(0, [this](VkCommandBuffer commandBuffer) {
        m_createMatrixAndRhsKernel.bindAndDispatch(commandBuffer, 0, m_cUbo.size/256 + 1, 1, 1);
//        vkb::ComputeShaderHandler::computeBarrier(commandBuffer, m_matrixBuffer);
//        m_matrixMultiplyKernel.bindAndDispatch(commandBuffer, 0, 1, 1, 1);
    });

    HeapMatrix<double> result(m_cUbo.size, m_cUbo.numTilesX);
    vkb::Buffer::writeBufferToVector(m_deviceRef, m_rhsBuffer, result.getVector());
    printMatrix(result);

    {
    double errMax = 0;
    long avgTime = 0;
    uint32_t iters = 0;
    for (int k = 0; k < iters; k++) {
        std::vector<double> v1(m_cUbo.size);
        for (uint32_t i = 0; i < m_cUbo.size; i++) {
            v1[i] = randomDouble();
        }
        vkb::Buffer::writeVectorToBuffer(m_deviceRef, m_directionBuffer, v1);
//    printVec(v1);

        std::vector<double> v2(m_cUbo.size);
        for (uint32_t i = 0; i < m_cUbo.size; i++) {
            v2[i] = randomDouble();
        }
        vkb::Buffer::writeVectorToBuffer(m_deviceRef, m_auxBuffer, v2);
//    printVec(v2);

        auto time = std::chrono::high_resolution_clock::now();
        m_computeHandler.runComputeIsolated(0, [this](VkCommandBuffer commandBuffer) {
//        m_addScaledKernel.bindAndDispatch(commandBuffer, 0, 1, 1, 1);
            applyDotProductKernel(commandBuffer, 0, m_dotProductAuxBuffers.size() - 1);
        });
        auto time2 = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(time2 - time).count();
        avgTime += duration;
        std::vector<double> res = {0};
        vkb::Buffer::writeBufferToVector(m_deviceRef, m_alphaBuffer, res);

        double dot = 0;
        for (uint32_t i = 0; i < m_cUbo.size; i++) {
            dot += v1[i] * v2[i];
        }

        double err = std::abs(dot - res[0]) / dot;
        std::cout << "\r" << k << " -> Dot product: CPU: " << dot << ", GPU: " << res[0] << std::scientific << ", err: "
                  << err << ", max err: " << errMax << ", time: " << duration << "us" << std::flush;
        errMax = std::max(errMax, err);
//        if (dot != res[0]) {
//            std::cout << "Dot product not equal, errMax: " << dot - res[0] << "\n";
//        } else {
//            std::cout << "Dot product equal\n";
//        }
    }
//    std::cout << std::scientific << "\nMax errMax: " << errMax << "\n";
//    std::cout << "Average time: " << double(avgTime) / iters << "us \n";

    }
}

void FlipSolver::initializeParticles(const std::unique_ptr<vkb::DescriptorPool> &globalPool)  {
    m_createMatrixAndRhsKernel.createPipeline();
    m_cUbo.size = 20*20;
    m_cUbo.numTilesX = 20;
//    m_cUbo.size = 1'000'000;
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


    m_createMatrixAndRhsKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_createMatrixAndRhsKernel.layout, {
            {m_computeUniformBuffer->descriptorInfo()},
            {m_matrixBuffer->descriptorInfo()},
            {m_typesBuffer->descriptorInfo()},
            {m_rhsBuffer->descriptorInfo()},
//            {m_directionBuffer->descriptorInfo()}, // vel X
//            {m_auxBuffer->descriptorInfo()}, // vel Y
    });

}
