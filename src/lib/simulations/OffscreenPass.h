//
// Created by luc on 09/10/23.
//

#ifndef VULKANFLUIDSIMULATION_OFFSCREENPASS_H
#define VULKANFLUIDSIMULATION_OFFSCREENPASS_H

#include "../utils.h"
#include "../Device.h"
#include "../Image.h"
#include "../RenderSystem.h"


namespace vkb {
    class OffscreenPass {
    public:
        OffscreenPass(const Device& device, VkExtent2D extent, bool isDepthOnly = false, bool hasMultipleImages = false);
        ~OffscreenPass();


        [[nodiscard]] VkDescriptorImageInfo descriptorInfo() const { return VkDescriptorImageInfo {
                    m_sampler,
                    m_image->view(),
                    (m_isDepthOnly) ? VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
            };
        }

        [[nodiscard]] VkRenderPass renderPass() const { return m_renderPass;}

        [[nodiscard]] VkDescriptorImageInfo additionalImageDescriptorInfo() const {
            if (!m_hasMultipleImages) throw std::runtime_error("cant get additional image descriptor without multiple images");

            return VkDescriptorImageInfo {
                    m_sampler,
                    m_additionalImage->view(),
                    (m_isDepthOnly) ? VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
            };
        }

        void bindRenderSystem(VkCommandBuffer commandBuffer,VkDescriptorSet* descriptorSet) { m_renderSystem.bind(commandBuffer, descriptorSet); }

        void createPass(VkDescriptorSetLayout globalSetLayout, const RenderSystem::ShaderPaths& shaderPaths,
                        const std::function<void(GraphicsPipeline::PipelineConfigInfo&)>& configurePipeline = {}, uint32_t PushConstantSize = 0);

        void changeImageSize(VkExtent2D extent);

        void run(VkCommandBuffer commandBuffer, VkDescriptorSet *descriptorSet,
                 const std::function<void(VkCommandBuffer &)> &function);



        void runAlternating(VkCommandBuffer commandBuffer, const std::array<VkDescriptorSet*, 2>& descriptorSet,
                 const std::function<void(VkCommandBuffer &)> &function);

    private:

        void createRenderPass();
        void createFrameBuffer();
        void createDepthRenderPass();
        void createDepthFrameBuffer();
        void destroyObjects();
        void runWithFrameBuffer(VkCommandBuffer commandBuffer, VkFramebuffer frameBuffer, VkDescriptorSet *descriptorSet,
                                const std::function<void(VkCommandBuffer &)> &function);

        // Depth bias (and slope) are used to avoid shadowing artifacts
        // Constant depth bias factor (always applied)
        static constexpr float DEPTH_BIAS_CONSTANT = 1.25f;
        // Slope depth bias factor, applied depending on polygon's slope
        static constexpr float DEPTH_BIAS_SLOPE = 1.75f;

        std::unique_ptr<Image> m_image;
        std::unique_ptr<Image> m_additionalImage;
        RenderSystem m_renderSystem;
        const bool m_isDepthOnly;
        const bool m_hasMultipleImages;
        bool m_created = false;
        const Device& m_deviceRef;
        VkExtent2D m_extent;
        VkFramebuffer m_frameBuffer{};
        VkFramebuffer m_additionalFrameBuffer{};
        VkRenderPass m_renderPass{};
        VkSampler m_sampler{};
        VkDescriptorImageInfo m_descriptor{};

        uint32_t m_alternatingIdx = 0;
    };
}



#endif //VULKANFLUIDSIMULATION_OFFSCREENPASS_H
