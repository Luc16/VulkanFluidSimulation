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

    std::vector<double> types = {
            0, 0, 0, 0, 0,
            0, 1, 1, 1, 0,
            0, 1, 1, 1, 0,
            0, 1, 1, 1, 0,
            0, 0, 0, 0, 0,
    };
    vkb::Buffer::writeVectorToBuffer(m_deviceRef, m_typesBuffer, types);
    m_computeHandler.runComputeIsolated(0, [this](VkCommandBuffer commandBuffer) {
//        m_addScaledKernel.bindAndDispatch(commandBuffer, 0, 1, 1, 1);
        m_createMatrixKernel.bindAndDispatch(commandBuffer, 0, 1, 1, 1);
    });


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
    m_addScaledKernel.createPipeline();
    m_dotProductKernel.createPipeline();
    m_reduceKernel.createPipeline();
    m_cUbo.size = 25;
    m_cUbo.numTilesX = 5;
    std::cout << "size: " << m_cUbo.size << "\n";
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
                                                      m_cUbo.size*sizeof(double),
                                                      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                      VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                                      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    m_directionBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                 m_cUbo.size*sizeof(double),
                                                 VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                 VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                 VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);


    m_auxBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                      m_cUbo.size*sizeof(double),
                                                      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                      VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                                      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);


    m_alphaBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                      sizeof(double),
                                                      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                      VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                      VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);


    for (uint32_t n = m_cUbo.size/m_workGroupSize + (m_cUbo.size%m_workGroupSize != 0); n > 1; n = n/m_workGroupSize + (n%m_workGroupSize != 0)) {
        m_dotProductAuxBuffers.emplace_back(std::make_unique<vkb::Buffer>(m_deviceRef,
                                                                 n*sizeof(double),
                                                                 VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                                                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT));
    }


    std::vector<double> alpha = {-0.5};
    vkb::Buffer::writeVectorToBuffer(m_deviceRef, m_alphaBuffer, alpha);

    m_addScaledKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_addScaledKernel.layout, {
            {m_computeUniformBuffer->descriptorInfo()},
            {m_directionBuffer->descriptorInfo()},
            {m_directionBuffer->descriptorInfo()},
            {m_alphaBuffer->descriptorInfo()},
            {m_auxBuffer->descriptorInfo()},
    });

    m_dotProductKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_dotProductKernel.layout, {
            {m_computeUniformBuffer->descriptorInfo()},
            {(!m_dotProductAuxBuffers.empty()) ? m_dotProductAuxBuffers[0]->descriptorInfo() : m_alphaBuffer->descriptorInfo()},
            {m_directionBuffer->descriptorInfo()},
            {m_auxBuffer->descriptorInfo()},
    });

    for (uint32_t i = 1; i < m_dotProductAuxBuffers.size(); i++) {
        m_reduceKernel.descSets.push_back(vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_reduceKernel.layout, {
                {m_computeUniformBuffer->descriptorInfo()},
                {m_dotProductAuxBuffers[i]->descriptorInfo()},
                {m_dotProductAuxBuffers[i-1]->descriptorInfo()},
        }));
    }

    if (!m_dotProductAuxBuffers.empty()) {
        m_reduceKernel.descSets.push_back(vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_reduceKernel.layout, {
                {m_computeUniformBuffer->descriptorInfo()},
                {m_alphaBuffer->descriptorInfo()},
                {m_dotProductAuxBuffers[m_dotProductAuxBuffers.size()-1]->descriptorInfo()},
        }));
    }

}

void FlipSolver::applyDotProductKernel(VkCommandBuffer commandBuffer,
                                       uint32_t dotProductDescSetIdx,
                                       uint32_t resDescSetIdx) {
    uint32_t numGroups = m_cUbo.size / m_workGroupSize + (m_cUbo.size % m_workGroupSize != 0);
    m_dotProductKernel.bindAndDispatch(commandBuffer, dotProductDescSetIdx, numGroups, 1, 1);
    for (uint32_t i = 0, n = numGroups; n > 256; n = n / m_workGroupSize + (n % m_workGroupSize != 0), i++) {
        m_reduceKernel.bindAndDispatch(commandBuffer, i, n, 1, 1);
    }
    if (!m_dotProductAuxBuffers.empty()) {
        m_reduceKernel.bindAndDispatch(commandBuffer, resDescSetIdx, 1, 1, 1);
    }

}