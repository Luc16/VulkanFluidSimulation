//
// Created by luc on 16/07/23.
//

#ifndef VULKANFLUIDSIMULATION_GPUSPATIALGRIDHANDLER_H
#define VULKANFLUIDSIMULATION_GPUSPATIALGRIDHANDLER_H

#include <vector>
#include "../utils.h"
#include "../ComputeSystem.h"
#include "../descriptors/DescriptorSetLayout.h"
#include "../Buffer.h"
#include "../descriptors/DescriptorPool.h"

namespace vkb {

    class GpuSpatialGridHandler {
    public:
        GpuSpatialGridHandler(
                const Device& device,
                uint32_t workGroupSize,
                std::string resetGridShader,
                std::string insertParticlesShader,
                std::string scanShader,
                std::string scanAddShader,
                std::string sortShader);
        GpuSpatialGridHandler(const GpuSpatialGridHandler &) = delete;
        GpuSpatialGridHandler &operator=(const GpuSpatialGridHandler &) = delete;
        ~GpuSpatialGridHandler() = default;

        void createDescriptorsAndBuffers(const std::unique_ptr<vkb::DescriptorPool>& globalPool, uint32_t gridSize, uint32_t numParticles,
                                         const std::unique_ptr<vkb::Buffer>& particleBuffer,
                                         const std::unique_ptr<vkb::Buffer>& sortedParticleBuffer);

        void resetGrid(VkCommandBuffer commandBuffer);
        void insertParticles(VkCommandBuffer commandBuffer);
        void prefixSum(VkCommandBuffer commandBuffer);
        void countingSort(VkCommandBuffer commandBuffer);
        void gridBarrier(VkCommandBuffer commandBuffer);


    private:

        VkDescriptorSet createSingleDescriptorSet(const std::unique_ptr<vkb::DescriptorPool>& globalPool, vkb::DescriptorSetLayout &layout, std::vector<VkDescriptorBufferInfo> bufferInfos);
        void createScanData(const std::unique_ptr<vkb::DescriptorPool>& globalPool);

        struct UniformBufferObject {
            uint32_t gridSize;
            uint32_t numParticles;
        };

        const Device& m_deviceRef;
        uint32_t m_workGroupSize;
        UniformBufferObject m_ubo{0, 0};

        std::vector<std::unique_ptr<Buffer>> m_partialSums;
        std::unique_ptr<Buffer> m_gridBuffer;
        std::unique_ptr<Buffer> m_uniformBuffer;


        vkb::DescriptorSetLayout m_resetDescriptorLayout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // grid
                .build();
        vkb::DescriptorSetLayout m_generalDescriptorLayout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                .addBinding({2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                .build();
        vkb::DescriptorSetLayout m_sortDescriptorLayout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // grid
                .addBinding({2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // particles in
                .addBinding({3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER,1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // particles out
                .build();

        VkDescriptorSet m_resetDescriptorSet{};
        VkDescriptorSet m_insertDescriptorSet{};
        std::vector<VkDescriptorSet> m_scanDescriptorSets{};
        std::vector<std::vector<std::pair<VkBuffer, VkDeviceSize>>> m_scanBarrierDatas{};
        VkDescriptorSet m_sortDescriptorSet{};

        vkb::ComputeSystem m_resetGridComputeSystem;
        vkb::ComputeSystem m_insertParticlesComputeSystem;
        vkb::ComputeSystem m_scanComputeSystem;
        vkb::ComputeSystem m_scanAddComputeSystem;
        vkb::ComputeSystem m_countingSortComputeSystem;

    };

} // vkb

#endif //VULKANFLUIDSIMULATION_GPUSPATIALGRIDHANDLER_H
