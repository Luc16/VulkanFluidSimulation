//
// Created by luc on 16/07/23.
//

#ifndef VULKANFLUIDSIMULATION_PBFGPUSPATIALGRIDHANDLER_H
#define VULKANFLUIDSIMULATION_PBFGPUSPATIALGRIDHANDLER_H

#include <vector>
#include "../../lib/utils.h"
#include "../../lib/ComputeSystem.h"
#include "../../lib/descriptors/DescriptorSetLayout.h"
#include "../../lib/Buffer.h"
#include "../../lib/descriptors/DescriptorPool.h"

namespace vkb {

    class PbfGpuSpatialGridHandler {
    public:
        struct GridUniformBufferObject {
            uint32_t gridSize;
        };

        struct GridParticleUniformBufferObject {
            uint32_t numParticles;
            float spacing;
            alignas(16) glm::vec3 boundSize;
        };

        PbfGpuSpatialGridHandler(
                const Device& device,
                uint32_t workGroupSize,
                std::string resetGridShader,
                std::string insertParticlesShader,
                std::string scanShader,
                std::string scanAddShader,
                std::string sortShader);
        PbfGpuSpatialGridHandler(const PbfGpuSpatialGridHandler &) = delete;
        PbfGpuSpatialGridHandler &operator=(const PbfGpuSpatialGridHandler &) = delete;
        ~PbfGpuSpatialGridHandler() = default;

        void createSystems();

        void createDescriptorsAndBuffers(const std::unique_ptr<DescriptorPool>& globalPool,
                                                                     uint32_t gridSize, GridParticleUniformBufferObject uboData,
                                                                     const std::unique_ptr<Buffer>& particleIdxBuffer,
                                                                     const std::array<std::unique_ptr<Buffer>, 2>& particlePosBuffers,
                                                                     const std::array<std::unique_ptr<Buffer>, 2>& particleVelBuffers);

        void resetGrid(VkCommandBuffer commandBuffer);
        void insertParticles(u_char frameIdx, VkCommandBuffer commandBuffer);
        void prefixSum(VkCommandBuffer commandBuffer);
        void countingSort(u_char frameIdx, VkCommandBuffer commandBuffer);
        void gridBarrier(VkCommandBuffer commandBuffer);

        [[nodiscard]] VkDescriptorBufferInfo gridDescriptorInfo(
                VkDeviceSize size = VK_WHOLE_SIZE,
                VkDeviceSize offset = 0) const { return m_gridBuffer->descriptorInfo(size, offset); }



    private:

        void createScanData(const std::unique_ptr<vkb::DescriptorPool>& globalPool);

        const Device& m_deviceRef;
        uint32_t m_workGroupSize;
        GridUniformBufferObject m_gridUbo{0};
        GridParticleUniformBufferObject m_gridParticleUbo{};

        std::vector<std::unique_ptr<Buffer>> m_partialSums;
        std::unique_ptr<Buffer> m_gridBuffer;
        std::unique_ptr<Buffer> m_gridUniformBuffer;
        std::unique_ptr<Buffer> m_gridParticleUniformBuffer;


        vkb::DescriptorSetLayout m_resetDescriptorLayout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // grid
                .build();
        vkb::DescriptorSetLayout m_insertDescriptorLayout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // grid
                .addBinding({2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // positions
                .build();
        vkb::DescriptorSetLayout m_scanDescriptorLayout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                .addBinding({2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                .build();
        vkb::DescriptorSetLayout m_sortDescriptorLayout = vkb::DescriptorSetLayout::Builder(m_deviceRef)
                .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr})
                .addBinding({1, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // grid
                .addBinding({2, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // particles pos in
                .addBinding({3, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // particles vel in
                .addBinding({4, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // particles pos out
                .addBinding({5, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // particles vel out
                .addBinding({6, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_COMPUTE_BIT, nullptr}) // particles vel out
                .build();

        VkDescriptorSet m_resetDescriptorSet{};
        std::array<VkDescriptorSet, 2> m_insertDescriptorSets{};
        std::vector<VkDescriptorSet> m_scanDescriptorSets{};
        std::vector<std::vector<std::pair<VkBuffer, VkDeviceSize>>> m_scanBarrierData{};
        std::array<VkDescriptorSet, 2> m_sortDescriptorSets{};
        bool m_created = false;

        vkb::ComputeSystem m_resetGridComputeSystem;
        vkb::ComputeSystem m_insertParticlesComputeSystem;
        vkb::ComputeSystem m_scanComputeSystem;
        vkb::ComputeSystem m_scanAddComputeSystem;
        vkb::ComputeSystem m_countingSortComputeSystem;

    };

} // vkb

#endif //VULKANFLUIDSIMULATION_PBFGPUSPATIALGRIDHANDLER_H
