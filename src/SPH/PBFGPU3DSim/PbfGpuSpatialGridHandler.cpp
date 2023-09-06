//
// Created by luc on 16/07/23.
//

#include "PbfGpuSpatialGridHandler.h"
#include "../../lib/descriptors/DescriptorWriter.h"
#include "../../lib/ComputeShaderHandler.h"

#include <utility>

namespace vkb {

    PbfGpuSpatialGridHandler::PbfGpuSpatialGridHandler(const Device &device, uint32_t workGroupSize,
                                                 std::string resetGridShader,
                                                 std::string insertParticlesShader, std::string scanShader,
                                                 std::string scanAddShader, std::string sortShader):
                                                    m_deviceRef(device),
                                                    m_workGroupSize(workGroupSize),
                                                    m_resetGridComputeSystem(device, std::move(resetGridShader)),
                                                    m_insertParticlesComputeSystem(device, std::move(insertParticlesShader)),
                                                    m_scanComputeSystem(device, std::move(scanShader)),
                                                    m_scanAddComputeSystem(device, std::move(scanAddShader)),
                                                    m_countingSortComputeSystem(device, std::move(sortShader)){}

    void PbfGpuSpatialGridHandler::createSystems() {
        m_insertParticlesComputeSystem.createPipelineWithLayout(m_insertDescriptorLayout.descriptorSetLayout());
        m_scanComputeSystem.createPipelineWithLayout(m_scanDescriptorLayout.descriptorSetLayout());
        m_scanAddComputeSystem.createPipelineWithLayout(m_scanDescriptorLayout.descriptorSetLayout());

        m_resetGridComputeSystem.createPipelineWithLayout(m_resetDescriptorLayout.descriptorSetLayout());

        m_countingSortComputeSystem.createPipelineWithLayout(m_sortDescriptorLayout.descriptorSetLayout());
    }

    void PbfGpuSpatialGridHandler::createDescriptorsAndBuffers(const std::unique_ptr<DescriptorPool>& globalPool,
                                                            uint32_t gridSize, GridParticleUniformBufferObject uboData,
                                                            const std::unique_ptr<Buffer>& gridIdxBuffer,
                                                            const std::array<std::unique_ptr<Buffer>, 2>& particlePosBuffers,
                                                            const std::array<std::unique_ptr<Buffer>, 2>& particleVelBuffers,
                                                            const std::array<std::unique_ptr<Buffer>, 2>& particlePredPosBuffers
                                                            ) {
        if (!m_created){
            m_gridUniformBuffer = std::make_unique<Buffer>(m_deviceRef, sizeof(GridUniformBufferObject), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                                VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
            m_gridUniformBuffer->map();

            m_gridParticleUniformBuffer = std::make_unique<Buffer>(m_deviceRef, sizeof(GridParticleUniformBufferObject), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                                        VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
            m_gridParticleUniformBuffer->map();
        }
        if (m_created) {
            // free descriptor sets to create new ones
//            m_scanDescriptorSets.push_back(m_resetDescriptorSet);
//            m_scanDescriptorSets.push_back(m_insertDescriptorSet);
//            m_scanDescriptorSets.push_back(m_sortDescriptorSet);
//            globalPool->freeDescriptors(m_scanDescriptorSets);
        }

        m_gridParticleUbo = uboData;
        if (gridSize != m_gridUbo.gridSize) {
            m_gridUbo.gridSize = gridSize;
            createScanData(globalPool);
        }

        m_gridUniformBuffer->write(&m_gridUbo);
        m_gridParticleUniformBuffer->write(&m_gridParticleUbo);

        m_resetDescriptorSet = DescriptorWriter::createSingleDescriptorSet(globalPool, m_resetDescriptorLayout, {
                {m_gridUniformBuffer->descriptorInfo()},
                {m_gridBuffer->descriptorInfo()}
        });

        for (uint32_t i = 0; i < particlePosBuffers.size(); i++){
            m_insertDescriptorSets[i] = DescriptorWriter::createSingleDescriptorSet(globalPool, m_insertDescriptorLayout, {
                    {m_gridParticleUniformBuffer->descriptorInfo()},
                    {m_gridBuffer->descriptorInfo()},
//                    {particlePredPosBuffers[i]->descriptorInfo()}
                    {particlePosBuffers[i]->descriptorInfo()} // TODO change later
            });

            m_sortDescriptorSets[i] = DescriptorWriter::createSingleDescriptorSet(globalPool, m_sortDescriptorLayout, {
                    {m_gridParticleUniformBuffer->descriptorInfo()},
                    {m_gridBuffer->descriptorInfo()},
                    {particlePosBuffers[i]->descriptorInfo()},
                    {particlePosBuffers[(i + 1) % particlePosBuffers.size()]->descriptorInfo()},
                    {particleVelBuffers[i]->descriptorInfo()},
                    {particleVelBuffers[(i + 1) % particleVelBuffers.size()]->descriptorInfo()},
                    {particlePredPosBuffers[i]->descriptorInfo()},
                    {particlePredPosBuffers[(i + 1) % particlePredPosBuffers.size()]->descriptorInfo()},
                    {gridIdxBuffer->descriptorInfo()}
            });
        }

        m_created = true;
    }

    void PbfGpuSpatialGridHandler::createScanData(const std::unique_ptr<DescriptorPool> &globalPool) {
        m_gridBuffer = std::make_unique<Buffer>(m_deviceRef, m_gridUbo.gridSize * sizeof(uint32_t),
                                                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                                VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        m_partialSums.clear();
        for (uint32_t n = m_gridUbo.gridSize; n > 1; n = (n + m_workGroupSize) / m_workGroupSize) {
            uint32_t next = (n + m_workGroupSize) / m_workGroupSize;
            m_partialSums.emplace_back(std::make_unique<Buffer>(m_deviceRef, next * sizeof(uint32_t),
                                                                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                                                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT));
        }

        m_scanDescriptorSets = {
                DescriptorWriter::createSingleDescriptorSet(globalPool, m_scanDescriptorLayout, {
                        m_gridUniformBuffer->descriptorInfo(),
                        m_gridBuffer->descriptorInfo(),
                        m_partialSums[0]->descriptorInfo()
                })
        };

        m_scanBarrierData.clear();
        m_scanBarrierData.resize(m_partialSums.size());
        for (uint32_t i = 0; i < m_partialSums.size(); i++){
            if (i > 0) {
                m_scanDescriptorSets.emplace_back(DescriptorWriter::createSingleDescriptorSet(globalPool, m_scanDescriptorLayout, {
                        m_gridUniformBuffer->descriptorInfo(),
                        m_partialSums[i - 1]->descriptorInfo(),
                        m_partialSums[i]->descriptorInfo()
                }));
            }

            std::pair<VkBuffer, VkDeviceSize> b1 = (i == 0) ? m_gridBuffer->getBarrierData() : m_partialSums[i-1]->getBarrierData();
            std::pair<VkBuffer, VkDeviceSize> b2 = m_partialSums[i]->getBarrierData();

            m_scanBarrierData[i] = {b1, b2};
        }
    }

    void PbfGpuSpatialGridHandler::resetGrid(VkCommandBuffer commandBuffer) {
        m_resetGridComputeSystem.bindAndDispatch(commandBuffer,
                                                 &m_resetDescriptorSet,
                                                 m_gridUbo.gridSize / 256 + (1 - (m_gridUbo.gridSize % 256 == 0)), 1, 1);
    }

    void PbfGpuSpatialGridHandler::insertParticles(u_char frameIdx, VkCommandBuffer commandBuffer) {
        m_insertParticlesComputeSystem.bindAndDispatch(commandBuffer,
                                                       &m_insertDescriptorSets[frameIdx],
                                                       m_gridParticleUbo.numParticles / 256 + (1 - (m_gridParticleUbo.numParticles % 256 == 0)), 1, 1);
    }

    void PbfGpuSpatialGridHandler::prefixSum(VkCommandBuffer commandBuffer) {
        for (int i = 0; i < m_partialSums.size(); i++) {
            m_scanComputeSystem.bindAndDispatch(commandBuffer,
                                                &m_scanDescriptorSets[i],
                                                m_gridUbo.gridSize / 256 + (1 - (m_gridUbo.gridSize % 256 == 0)), 1, 1);
            ComputeShaderHandler::computeBarriers(commandBuffer, m_scanBarrierData[i]);
        }

        for (int i = int(m_partialSums.size()) - 2; i >= 0; i--) {
            m_scanAddComputeSystem.bindAndDispatch(commandBuffer,
                                                   &m_scanDescriptorSets[i],
                                                   m_gridUbo.gridSize / 256 + (1 - (m_gridUbo.gridSize % 256 == 0)), 1, 1);

            ComputeShaderHandler::computeBarriers(commandBuffer, m_scanBarrierData[i]);
        }
    }

    void PbfGpuSpatialGridHandler::countingSort(u_char frameIdx, VkCommandBuffer commandBuffer) {
        m_countingSortComputeSystem.bindAndDispatch(commandBuffer,
                                                    &m_sortDescriptorSets[frameIdx],
                                                    m_gridParticleUbo.numParticles / 256 + (1 - (m_gridParticleUbo.numParticles % 256 == 0)), 1, 1);
    }

    void PbfGpuSpatialGridHandler::gridBarrier(VkCommandBuffer commandBuffer) {
        ComputeShaderHandler::computeBarrier(commandBuffer, m_gridBuffer);
    }
} // vkb