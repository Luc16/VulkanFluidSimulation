//
// Created by luc on 09/10/23.
//

#include "OffscreenPass.h"


namespace vkb {
    OffscreenPass::OffscreenPass(const vkb::Device &device, VkExtent2D extent): m_deviceRef(device), m_extent(extent) {}

    OffscreenPass::~OffscreenPass() {
        vkDestroyFramebuffer(m_deviceRef.device(), m_frameBuffer, nullptr);
        vkDestroyRenderPass(m_deviceRef.device(), m_renderPass, nullptr);
        vkDestroySampler(m_deviceRef.device(), m_depthSampler, nullptr);
    }

    void OffscreenPass::createRenderPass() {
        VkAttachmentDescription attachmentDescription{};
        attachmentDescription.format = VK_FORMAT_D16_UNORM;
        attachmentDescription.samples = VK_SAMPLE_COUNT_1_BIT;
        attachmentDescription.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;                            // Clear depth at beginning of the render pass
        attachmentDescription.storeOp = VK_ATTACHMENT_STORE_OP_STORE;                        // We will read from depth, so it's important to store the depth attachment results
        attachmentDescription.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        attachmentDescription.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        attachmentDescription.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;                    // We don't care about initial layout of the attachment
        attachmentDescription.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;// Attachment will be transitioned to shader read at render pass end

        VkAttachmentReference depthReference = {};
        depthReference.attachment = 0;
        depthReference.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;            // Attachment will be used as depth/stencil during render pass

        VkSubpassDescription subpass = {};
        subpass.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
        subpass.colorAttachmentCount = 0;                                                    // No color attachments
        subpass.pDepthStencilAttachment = &depthReference;                                    // Reference to our depth attachment

        // Use subpass dependencies for layout transitions
        std::array<VkSubpassDependency, 2> dependencies{};

        dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
        dependencies[0].dstSubpass = 0;
        dependencies[0].srcStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        dependencies[0].dstStageMask = VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
        dependencies[0].srcAccessMask = VK_ACCESS_SHADER_READ_BIT;
        dependencies[0].dstAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
        dependencies[0].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

        dependencies[1].srcSubpass = 0;
        dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
        dependencies[1].srcStageMask = VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
        dependencies[1].dstStageMask = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
        dependencies[1].srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
        dependencies[1].dstAccessMask = VK_ACCESS_SHADER_READ_BIT;
        dependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

        VkRenderPassCreateInfo renderPassCreateInfo{};
        renderPassCreateInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
        renderPassCreateInfo.attachmentCount = 1;
        renderPassCreateInfo.pAttachments = &attachmentDescription;
        renderPassCreateInfo.subpassCount = 1;
        renderPassCreateInfo.pSubpasses = &subpass;
        renderPassCreateInfo.dependencyCount = static_cast<uint32_t>(dependencies.size());
        renderPassCreateInfo.pDependencies = dependencies.data();

        if (vkCreateRenderPass(m_deviceRef.device(), &renderPassCreateInfo, nullptr, &m_renderPass) != VK_SUCCESS) {
            throw std::runtime_error("failed to create render pass!");
        }
    }

    void OffscreenPass::createFrameBuffer() {
        // For shadow mapping we only need a depth attachment
        m_depthImage = std::make_unique<Image>(
                m_deviceRef,
                m_extent.width,
                m_extent.height,
                1,
                VK_SAMPLE_COUNT_1_BIT,
                VK_FORMAT_D16_UNORM,
                VK_IMAGE_TILING_OPTIMAL,
                VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
                VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
                VK_IMAGE_ASPECT_DEPTH_BIT
                );


        // Create sampler to sample from to depth attachment
        // Used to sample in the fragment shader for shadowed rendering
        VkFormatProperties formatProps;
        vkGetPhysicalDeviceFormatProperties(m_deviceRef.physicalDevice(), VK_FORMAT_D16_UNORM, &formatProps);

        VkFilter shadowmap_filter = formatProps.optimalTilingFeatures & VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT ?
                                    VK_FILTER_LINEAR :
                                    VK_FILTER_NEAREST;
        VkSamplerCreateInfo samplerInfo{};
        samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
        samplerInfo.maxAnisotropy = 1.0f;
        samplerInfo.magFilter = shadowmap_filter;
        samplerInfo.minFilter = shadowmap_filter;
        samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
        samplerInfo.addressModeV = samplerInfo.addressModeU;
        samplerInfo.addressModeW = samplerInfo.addressModeU;
        samplerInfo.mipLodBias = 0.0f;
        samplerInfo.maxAnisotropy = 1.0f;
        samplerInfo.minLod = 0.0f;
        samplerInfo.maxLod = 1.0f;
        samplerInfo.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
        if (vkCreateSampler(m_deviceRef.device(), &samplerInfo, nullptr, &m_depthSampler) != VK_SUCCESS) {
            throw std::runtime_error("failed to create texture sampler!");
        }

        createRenderPass();

        // Create frame buffer
        VkImageView depthImageView = m_depthImage->view();
        VkFramebufferCreateInfo fbufCreateInfo{};
        fbufCreateInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        fbufCreateInfo.renderPass = m_renderPass;
        fbufCreateInfo.attachmentCount = 1;
        fbufCreateInfo.pAttachments = &depthImageView;
        fbufCreateInfo.width = m_extent.width;
        fbufCreateInfo.height = m_extent.height;
        fbufCreateInfo.layers = 1;

        if (vkCreateFramebuffer(m_deviceRef.device(), &fbufCreateInfo, nullptr, &m_frameBuffer) != VK_SUCCESS) {
            throw std::runtime_error("failed to create framebuffer!");
        }
    }

    void OffscreenPass::runRenderPass(VkCommandBuffer commandBuffer, const std::function<void(VkCommandBuffer&)> &function) const {
        VkRenderPassBeginInfo renderPassInfo{};
        renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
        renderPassInfo.renderPass = m_renderPass;
        renderPassInfo.framebuffer = m_frameBuffer;
        renderPassInfo.renderArea.offset = {0, 0};
        renderPassInfo.renderArea.extent = m_extent;

        std::array<VkClearValue, 1> clearValues{};
        clearValues[0].depthStencil = {1.0f, 0};
        renderPassInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
        renderPassInfo.pClearValues = clearValues.data();

        vkCmdBeginRenderPass(commandBuffer, &renderPassInfo, VK_SUBPASS_CONTENTS_INLINE);

        VkViewport viewport{};
        viewport.x = 0.0f;
        viewport.y = 0.0f;
        viewport.width = static_cast<float>(m_extent.width);
        viewport.height = static_cast<float>(m_extent.height);
        viewport.minDepth = 0.0f;
        viewport.maxDepth = 1.0f;
        vkCmdSetViewport(commandBuffer, 0, 1, &viewport);

        VkRect2D scissor{};
        scissor.offset = {0, 0};
        scissor.extent = m_extent;
        vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

        // Set depth bias (aka "Polygon offset")
        // Required to avoid shadow mapping artifacts
        vkCmdSetDepthBias(
                commandBuffer,
                DEPTH_BIAS_CONSTANT,
                0.0f,
                DEPTH_BIAS_SLOPE);

        function(commandBuffer);

        vkCmdEndRenderPass(commandBuffer);

    }
}
