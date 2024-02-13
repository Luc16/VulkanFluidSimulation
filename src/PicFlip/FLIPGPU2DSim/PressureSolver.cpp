//
// Created by luc on 23/01/24.
//

#include "PressureSolver.h"

PressureSolver::PressureSolver(const vkb::Device &device, const std::vector<std::string> &shaders, uint32_t size): m_deviceRef(device), m_shaderPaths(shaders), m_size(size) {
    m_pUbos[0] = {size, 0};
    m_pUbos[1] = {size, 1};
    m_pUbos[2] = {size, 2};
    createBuffers();
}

int PressureSolver::solve() {

    std::vector<double> beta = {0.0};
    vkb::Buffer::writeVectorToBuffer(m_deviceRef, m_betaBuffer, beta);
    m_computeHandler.runComputeIsolated(0, [&](VkCommandBuffer commandBuffer){
        uint32_t n = m_size/m_workGroupSize + 1;
        // d = r
        m_addScaledKernel.bindAndDispatch(commandBuffer, 0, n, 1, 1);

        // gamma = dot(r, r) && gamma0 = initial gamma
        applyDotProductKernel(commandBuffer, 0, 0);

        for (uint32_t i = 0; i < 10; i++) {
            vkb::ComputeShaderHandler::computeBarrier(commandBuffer, m_directionBuffer);

            // z = A*d
            m_matrixMultiplyKernel.bindAndDispatch(commandBuffer, 0, n, 1, 1);

            vkb::ComputeShaderHandler::computeBarrier(commandBuffer, m_auxBuffer);

            // alpha = gamma/dot(d, z)
            applyDotProductKernel(commandBuffer, 1, 1);

            vkb::ComputeShaderHandler::computeBarrier(commandBuffer, m_alphaBuffer);

            // p = p + alpha*d
            m_addScaledKernel.bindAndDispatch(commandBuffer, 1, n, 1, 1);

            // r = r - alpha*z
            m_addScaledKernel.bindAndDispatch(commandBuffer, 2, n, 1, 1);

            vkb::ComputeShaderHandler::computeBarriers(commandBuffer, m_externalBarriers);

            // gamma_prev = gramma -> gamma = dot -> beta = gamma/gamma_prev
            applyDotProductKernel(commandBuffer, 0, 2);

            vkb::ComputeShaderHandler::computeBarriers(commandBuffer, {m_gammaBuffer->getBarrierData(), m_betaBuffer->getBarrierData()});

            // d = r + beta*d
            m_addScaledKernel.bindAndDispatch(commandBuffer, 0, n, 1, 1);

        }
    });

    std::vector<double> gamma = {0.0, 0.0, 0.0};
    vkb::Buffer::writeBufferToVector(m_deviceRef, m_gammaBuffer, gamma);
    std::cout << "gamma: " << gamma[0] << ", " << gamma[1] << ", " << gamma[2] << "\n";
    std::cout << "error: " << gamma[0]/gamma[2] << "\n";

    vkb::Buffer::writeBufferToVector(m_deviceRef, m_betaBuffer, gamma);
    std::cout << "beta: " << gamma[0] << "\n";

    vkb::Buffer::writeBufferToVector(m_deviceRef, m_alphaBuffer, gamma);
    std::cout << "alpha: " << gamma[0] << "\n";

    return 0;
}

void PressureSolver::applyDotProductKernel(VkCommandBuffer commandBuffer,
                                       uint32_t dotProductDescSetIdx,
                                       uint32_t resDescSetIdx) {
    uint32_t numGroups = m_size / m_workGroupSize + (m_size % m_workGroupSize != 0);
    m_dotProductKernel.bindAndDispatch(commandBuffer, dotProductDescSetIdx, numGroups, 1, 1);
    vkb::ComputeShaderHandler::computeBarrier(commandBuffer, m_dotProductAuxBuffers[0]);

    for (uint32_t i = 0, n = numGroups; n > m_workGroupSize; n = n / m_workGroupSize + (n % m_workGroupSize != 0), i++) {
        m_reduceKernel.bindAndDispatch(commandBuffer, i, n, 1, 1);
        vkb::ComputeShaderHandler::computeBarrier(commandBuffer, m_dotProductAuxBuffers[i+1]);
    }

    m_finishDotKernel.bindAndDispatch(commandBuffer, resDescSetIdx, 1, 1, 1);


}

void PressureSolver::createBuffers() {
    for (uint32_t i = 0; i < m_pUbos.size(); i++) {
        m_pressureSolverUniformBuffer[i] = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                               sizeof(PressureSolverUniformBufferObject),
                                                               VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                               VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        m_pressureSolverUniformBuffer[i]->singleWrite(&m_pUbos[i]);
    }

    m_directionBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                      m_size*sizeof(double),
                                                      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                      VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                      VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    m_auxBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                m_size*sizeof(double),
                                                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    m_alphaBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                  sizeof(double),
                                                  VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                  VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                  VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                  VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    m_betaBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                  sizeof(double),
                                                  VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                  VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                  VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                  VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    std::vector<double> beta = {0.0};
    vkb::Buffer::writeVectorToBuffer(m_deviceRef, m_betaBuffer, beta);

    m_gammaBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                  3*sizeof(double),
                                                  VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                  VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                  VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                  VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    if (m_size <= m_workGroupSize) throw std::runtime_error("Size of the matrix is too small for the work group size");
    for (uint32_t n = m_size/m_workGroupSize + (m_size%m_workGroupSize != 0); n > 1; n = n/m_workGroupSize + (n%m_workGroupSize != 0)) {
        m_dotProductAuxBuffers.emplace_back(std::make_unique<vkb::Buffer>(m_deviceRef,
                                                                          n*sizeof(double),
                                                                          VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                                                          VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT));
    }
    if (m_dotProductAuxBuffers.empty()) throw std::runtime_error("No dot product aux buffers created");

}

void PressureSolver::initializeKernels(const std::unique_ptr<vkb::DescriptorPool> &globalPool,
                                       const std::unique_ptr<vkb::Buffer>& uniformBuffer,
                                       const std::unique_ptr<vkb::Buffer>& matrixBuffer,
                                       const std::unique_ptr<vkb::Buffer>& typesBuffer,
                                       const std::unique_ptr<vkb::Buffer>& residualBuffer,
                                       const std::unique_ptr<vkb::Buffer>& pressureBuffer) {
    m_matrixMultiplyKernel.createPipeline();
    m_addScaledKernel.createPipeline();
    m_dotProductKernel.createPipeline();
    m_finishDotKernel.createPipeline();
    m_reduceKernel.createPipeline();

    m_externalBarriers = {
            residualBuffer->getBarrierData(),
            pressureBuffer->getBarrierData(),
    };

    // z = A*d
    m_matrixMultiplyKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_matrixMultiplyKernel.layout, {
            {uniformBuffer->descriptorInfo()},
            {matrixBuffer->descriptorInfo()},
            {typesBuffer->descriptorInfo()},
            {m_directionBuffer->descriptorInfo()},
            {m_auxBuffer->descriptorInfo()},
    });

    // d = r + beta*d
    m_addScaledKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_addScaledKernel.layout, {
            {m_pressureSolverUniformBuffer[0]->descriptorInfo()},
            {m_directionBuffer->descriptorInfo()},
            {residualBuffer->descriptorInfo()},
            {m_betaBuffer->descriptorInfo()},
            {m_directionBuffer->descriptorInfo()},
    });

    // p = p + alpha*d
    m_addScaledKernel.descSets[1] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_addScaledKernel.layout, {
            {m_pressureSolverUniformBuffer[0]->descriptorInfo()},
            {pressureBuffer->descriptorInfo()},
            {pressureBuffer->descriptorInfo()},
            {m_alphaBuffer->descriptorInfo()},
            {m_directionBuffer->descriptorInfo()},
    });

    // r = r - alpha*z
    m_addScaledKernel.descSets[2] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_addScaledKernel.layout, {
            {m_pressureSolverUniformBuffer[1]->descriptorInfo()},
            {residualBuffer->descriptorInfo()},
            {residualBuffer->descriptorInfo()},
            {m_alphaBuffer->descriptorInfo()},
            {m_auxBuffer->descriptorInfo()},
    });

    // dot(r, r)
    m_dotProductKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_dotProductKernel.layout, {
            {m_pressureSolverUniformBuffer[0]->descriptorInfo()},
            {m_dotProductAuxBuffers[0]->descriptorInfo()},
            {residualBuffer->descriptorInfo()},
            {residualBuffer->descriptorInfo()},
    });

    // dot(d, z)
    m_dotProductKernel.descSets[1] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_dotProductKernel.layout, {
            {m_pressureSolverUniformBuffer[0]->descriptorInfo()},
            {m_dotProductAuxBuffers[0]->descriptorInfo()},
            {m_auxBuffer->descriptorInfo()},
            {m_directionBuffer->descriptorInfo()},
    });

    for (uint32_t i = 1; i < m_dotProductAuxBuffers.size(); i++) {
        m_reduceKernel.descSets.push_back(vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_reduceKernel.layout, {
                {m_pressureSolverUniformBuffer[0]->descriptorInfo()},
                {m_dotProductAuxBuffers[i]->descriptorInfo()},
                {m_dotProductAuxBuffers[i-1]->descriptorInfo()},
        }));
    }

    // gamma0 = initial gamma
    m_finishDotKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_finishDotKernel.layout, {
            {m_pressureSolverUniformBuffer[0]->descriptorInfo()},
            {m_gammaBuffer->descriptorInfo()},
            {m_dotProductAuxBuffers[m_dotProductAuxBuffers.size()-1]->descriptorInfo()},
            {m_alphaBuffer->descriptorInfo()},
    });

    // alpha = gamma/dot
    m_finishDotKernel.descSets[1] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_finishDotKernel.layout, {
            {m_pressureSolverUniformBuffer[1]->descriptorInfo()},
            {m_alphaBuffer->descriptorInfo()},
            {m_dotProductAuxBuffers[m_dotProductAuxBuffers.size()-1]->descriptorInfo()},
            {m_gammaBuffer->descriptorInfo()},
    });

    // gamma_prev == gramma -> gamma = dot -> beta = gamma/gamma_prev
    m_finishDotKernel.descSets[2] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_finishDotKernel.layout, {
            {m_pressureSolverUniformBuffer[2]->descriptorInfo()},
            {m_gammaBuffer->descriptorInfo()},
            {m_dotProductAuxBuffers[m_dotProductAuxBuffers.size()-1]->descriptorInfo()},
            {m_betaBuffer->descriptorInfo()},
    });
}
