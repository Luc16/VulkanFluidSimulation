//
// Created by luc on 16/07/23.
//

#include "GpuSpatialGridHandler.h"
#include "../descriptors/DescriptorWriter.h"
#include "../ComputeShaderHandler.h"

#include <utility>

namespace vkb {

    GpuSpatialGridHandler::GpuSpatialGridHandler(const Device &device, uint32_t workGroupSize,
                                                 std::string resetGridShader,
                                                 std::string insertParticlesShader, std::string scanShader,
                                                 std::string scanAddShader, std::string sortShader):
                                                    m_deviceRef(device),
                                                    m_workGroupSize(workGroupSize),
                                                    m_resetGridComputeSystem(device, std::move(resetGridShader)),
                                                    m_insertParticlesComputeSystem(device, std::move(insertParticlesShader)),
                                                    m_scanComputeSystem(device, std::move(scanShader)),
                                                    m_scanAddComputeSystem(device, std::move(scanAddShader)),
                                                    m_countingSortComputeSystem(device, std::move(sortShader))
                                                 {

        m_insertParticlesComputeSystem.createPipelineWithLayout(m_generalDescriptorLayout.descriptorSetLayout());
        m_scanComputeSystem.createPipelineWithLayout(m_generalDescriptorLayout.descriptorSetLayout());
        m_scanAddComputeSystem.createPipelineWithLayout(m_generalDescriptorLayout.descriptorSetLayout());

        m_resetGridComputeSystem.createPipelineWithLayout(m_resetDescriptorLayout.descriptorSetLayout());

        m_countingSortComputeSystem.createPipelineWithLayout(m_sortDescriptorLayout.descriptorSetLayout());

        m_uniformBuffer = std::make_unique<vkb::Buffer>(m_deviceRef, sizeof(UniformBufferObject), VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                     VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        m_uniformBuffer->map();
    }

    void GpuSpatialGridHandler::createDescriptorsAndBuffers(const std::unique_ptr<vkb::DescriptorPool>& globalPool,
                                                            uint32_t gridSize, uint32_t numParticles,
                                                            const std::unique_ptr<vkb::Buffer>& particleBuffer,
                                                            const std::unique_ptr<vkb::Buffer>& sortedParticleBuffer) {
        m_ubo.numParticles = numParticles;
        if (gridSize != m_ubo.gridSize) {
            m_ubo.gridSize = gridSize;
            createScanData(globalPool);
        }

        m_uniformBuffer->write(&m_ubo);

        m_resetDescriptorSet = createSingleDescriptorSet(globalPool, m_resetDescriptorLayout, {
                {m_uniformBuffer->descriptorInfo()},
                {m_gridBuffer->descriptorInfo()}
        });

        m_insertDescriptorSet = createSingleDescriptorSet(globalPool, m_resetDescriptorLayout, {
                {m_uniformBuffer->descriptorInfo()},
                {m_gridBuffer->descriptorInfo()},
                {particleBuffer->descriptorInfo()}
        });

        m_sortDescriptorSet = createSingleDescriptorSet(globalPool, m_sortDescriptorLayout, {
                {m_uniformBuffer->descriptorInfo()},
                {m_gridBuffer->descriptorInfo()},
                {particleBuffer->descriptorInfo()},
                {sortedParticleBuffer->descriptorInfo()},
        });
    }

    void GpuSpatialGridHandler::createScanData(const std::unique_ptr<vkb::DescriptorPool> &globalPool) {
        m_gridBuffer = std::make_unique<Buffer>(m_deviceRef, m_ubo.gridSize * sizeof(uint32_t),
                                                VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                                VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        m_partialSums.clear();
        for (uint32_t n = m_ubo.gridSize; n > 1; n = (n + m_workGroupSize) / m_workGroupSize) {
            uint32_t next = (n + m_workGroupSize) / m_workGroupSize;
            m_partialSums.emplace_back(std::make_unique<vkb::Buffer>(m_deviceRef, next * sizeof(uint32_t),
                                                                     VK_BUFFER_USAGE_STORAGE_BUFFER_BIT,
                                                                     VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT));
        }

        m_scanDescriptorSets = {
                createSingleDescriptorSet(globalPool, m_generalDescriptorLayout, {
                        m_uniformBuffer->descriptorInfo(),
                        m_gridBuffer->descriptorInfo(),
                        m_partialSums[0]->descriptorInfo()
                })
        };

        m_scanBarrierDatas.resize(m_partialSums.size());
        for (uint32_t i = 0; i < m_partialSums.size(); i++){
            if (i > 0) {
                m_scanDescriptorSets.emplace_back(createSingleDescriptorSet(globalPool, m_generalDescriptorLayout, {
                        m_uniformBuffer->descriptorInfo(),
                        m_partialSums[i - 1]->descriptorInfo(),
                        m_partialSums[i]->descriptorInfo()
                }));
            }

            std::pair<VkBuffer, VkDeviceSize> b1 = (i == 0) ? std::make_pair(m_gridBuffer->getBuffer(), m_gridBuffer->getSize()) :
                                                   std::make_pair(m_partialSums[i-1]->getBuffer(), m_partialSums[i-1]->getSize());
            std::pair<VkBuffer, VkDeviceSize> b2 = std::make_pair(m_partialSums[i]->getBuffer(), m_partialSums[i]->getSize());

            m_scanBarrierDatas[i] = {b1, b2};
        }
    }


    VkDescriptorSet
    GpuSpatialGridHandler::createSingleDescriptorSet(const std::unique_ptr<vkb::DescriptorPool> &globalPool,
                                                     DescriptorSetLayout &layout,
                                                     std::vector<VkDescriptorBufferInfo> bufferInfos) {
        VkDescriptorSetLayout setLayout = layout.descriptorSetLayout();
        VkDescriptorSetAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
        allocInfo.descriptorPool = globalPool->descriptorPool();
        allocInfo.descriptorSetCount = 1;
        allocInfo.pSetLayouts = &setLayout;

        VkDescriptorSet set;
        if (vkAllocateDescriptorSets(m_deviceRef.device(), &allocInfo, &set) != VK_SUCCESS) {
            throw std::runtime_error("failed to allocate descriptor sets!");
        }

        auto writer = vkb::DescriptorWriter(layout, *globalPool);
        for (uint32_t i = 0; i < bufferInfos.size(); i++){
            writer.writeBuffer(i, &bufferInfos[i]);
        }

        writer.build(set, false);

        return set;
    }

    void GpuSpatialGridHandler::resetGrid(VkCommandBuffer commandBuffer) {
        m_resetGridComputeSystem.bindAndDispatch(commandBuffer,
                                                 &m_resetDescriptorSet,
                                                 m_ubo.gridSize/m_workGroupSize, 1, 1);
    }

    void GpuSpatialGridHandler::insertParticles(VkCommandBuffer commandBuffer) {
        m_insertParticlesComputeSystem.bindAndDispatch(commandBuffer,
                                                 &m_insertDescriptorSet,
                                                 m_ubo.numParticles/m_workGroupSize, 1, 1);
    }

    void GpuSpatialGridHandler::prefixSum(VkCommandBuffer commandBuffer) {
        for (int i = 0; i < m_partialSums.size(); i++) {
            m_scanComputeSystem.bindAndDispatch(commandBuffer,
                                              &m_scanDescriptorSets[i],
                                              m_ubo.gridSize/m_workGroupSize, 1, 1);
            vkb::ComputeShaderHandler::computeBarriers(commandBuffer, m_scanBarrierDatas[i]);
        }

        for (int i = int(m_partialSums.size()) - 2; i >= 0; i--) {
            m_scanAddComputeSystem.bindAndDispatch(commandBuffer,
                                                 &m_scanDescriptorSets[i],
                                                 m_ubo.gridSize/m_workGroupSize, 1, 1);

            vkb::ComputeShaderHandler::computeBarriers(commandBuffer, m_scanBarrierDatas[i]);
        }
    }

    void GpuSpatialGridHandler::countingSort(VkCommandBuffer commandBuffer) {
        m_countingSortComputeSystem.bindAndDispatch(commandBuffer,
                                                       &m_sortDescriptorSet,
                                                       m_ubo.numParticles/m_workGroupSize, 1, 1);
    }

    void GpuSpatialGridHandler::gridBarrier(VkCommandBuffer commandBuffer) {
        ComputeShaderHandler::computeBarrier(commandBuffer, m_gridBuffer);
    }
} // vkb