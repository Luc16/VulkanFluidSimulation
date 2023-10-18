//
// Created by luc on 09/10/23.
//

#ifndef VULKANFLUIDSIMULATION_OFFSCREENPASS_H
#define VULKANFLUIDSIMULATION_OFFSCREENPASS_H

#include "../../lib/utils.h"
#include "../../lib/Device.h"
#include "../../lib/Image.h"
#include "../../lib/RenderSystem.h"


namespace vkb {
    class OffscreenPass {
    public:
        OffscreenPass(const Device& device, VkExtent2D extent, bool isDepthOnly = false);
        ~OffscreenPass();


        [[nodiscard]] VkDescriptorImageInfo descriptorInfo() const { return VkDescriptorImageInfo {
                    m_sampler,
                    m_image->view(),
                    (m_isDepthOnly) ? VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL
            };
        }

        void createPass(VkDescriptorSetLayout globalSetLayout, const RenderSystem::ShaderPaths& shaderPaths,
                        const std::function<void(GraphicsPipeline::PipelineConfigInfo&)>& configurePipeline);

        void run(VkCommandBuffer commandBuffer, VkDescriptorSet *descriptorSet,
                 const std::function<void(VkCommandBuffer &)> &function) const;
    private:

        void createRenderPass();
        void createFrameBuffer();
        void createDepthRenderPass();
        void createDepthFrameBuffer();

        // Depth bias (and slope) are used to avoid shadowing artifacts
        // Constant depth bias factor (always applied)
        static constexpr float DEPTH_BIAS_CONSTANT = 1.25f;
        // Slope depth bias factor, applied depending on polygon's slope
        static constexpr float DEPTH_BIAS_SLOPE = 1.75f;

        std::unique_ptr<Image> m_image;
        RenderSystem m_renderSystem;
        const bool m_isDepthOnly;
        const Device& m_deviceRef;
        VkExtent2D m_extent;
        VkFramebuffer m_frameBuffer{};
        VkRenderPass m_renderPass{};
        VkSampler m_sampler{};
        VkDescriptorImageInfo m_descriptor{};
    };
}



#endif //VULKANFLUIDSIMULATION_OFFSCREENPASS_H
