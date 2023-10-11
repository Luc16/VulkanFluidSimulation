//
// Created by luc on 09/10/23.
//

#ifndef VULKANFLUIDSIMULATION_OFFSCREENPASS_H
#define VULKANFLUIDSIMULATION_OFFSCREENPASS_H

#include "../../lib/utils.h"
#include "../../lib/Device.h"
#include "../../lib/Image.h"


namespace vkb {
    class OffscreenPass {
    public:
        OffscreenPass(const Device& device, VkExtent2D extent);
        ~OffscreenPass();


        [[nodiscard]] VkRenderPass renderPass() const { return m_renderPass; }
        [[nodiscard]] VkDescriptorImageInfo descriptorInfo() const { return VkDescriptorImageInfo {
                    m_depthSampler, m_depthImage->view(), VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL
            };
        }

        void createRenderPass();
        void createFrameBuffer();
        void runRenderPass(VkCommandBuffer commandBuffer, const std::function<void(VkCommandBuffer&)> &function) const;

    private:
        // Depth bias (and slope) are used to avoid shadowing artifacts
        // Constant depth bias factor (always applied)
        static constexpr float DEPTH_BIAS_CONSTANT = 1.25f;
        // Slope depth bias factor, applied depending on polygon's slope
        static constexpr float DEPTH_BIAS_SLOPE = 1.75f;
        std::unique_ptr<Image> m_depthImage;

        const Device& m_deviceRef;
        VkExtent2D m_extent;
        VkFramebuffer m_frameBuffer{};
        VkRenderPass m_renderPass{};
        VkSampler m_depthSampler{};
        VkDescriptorImageInfo m_descriptor{};
    };
}



#endif //VULKANFLUIDSIMULATION_OFFSCREENPASS_H
