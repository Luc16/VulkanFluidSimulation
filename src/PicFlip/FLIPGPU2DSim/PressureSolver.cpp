//
// Created by luc on 23/01/24.
//

#include "PressureSolver.h"

PressureSolver::PressureSolver(const vkb::Device &device, const std::vector<std::string> &shaders): m_deviceRef(device), m_shaderPaths(shaders) {
    createBuffers();
}

void PressureSolver::applyDotProductKernel(VkCommandBuffer commandBuffer,
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

void PressureSolver::createBuffers() {
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

}

void PressureSolver::initializeKernels(const std::unique_ptr<vkb::DescriptorPool> &globalPool,
                                       const std::unique_ptr<vkb::Buffer>& matrixBuffer,
                                       const std::unique_ptr<vkb::Buffer>& typesBuffer,
                                       const std::unique_ptr<vkb::Buffer>& residualBuffer,
                                       const std::unique_ptr<vkb::Buffer>& pressureBuffer) {
    m_matrixMultiplyKernel.createPipeline();
    m_addScaledKernel.createPipeline();
    m_dotProductKernel.createPipeline();
    m_reduceKernel.createPipeline();


    m_matrixMultiplyKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_matrixMultiplyKernel.layout, {
            {m_computeUniformBuffer->descriptorInfo()},
            {matrixBuffer->descriptorInfo()},
            {typesBuffer->descriptorInfo()},
            {m_auxBuffer->descriptorInfo()},
            {m_directionBuffer->descriptorInfo()},
    });

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
