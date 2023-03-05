//
// Created by luc on 22/10/22.
//

#ifndef VULKANBASE_SWAPCHAIN_H
#define VULKANBASE_SWAPCHAIN_H

#include <vulkan/vulkan.h>
#include <vector>
#include "utils.h"
#include "Device.h"
#include "Image.h"
#include <GLFW/glfw3.h>


namespace vkb {
    class SwapChain {
    public:
        static constexpr int MAX_FRAMES_IN_FLIGHT = 2;

        SwapChain(const Device& device, VkExtent2D windowExtent);
        SwapChain(const Device& device, VkExtent2D windowExtent, std::shared_ptr<SwapChain> oldSwapChain);

        ~SwapChain();

        SwapChain(const SwapChain &) = delete;

        SwapChain &operator=(const SwapChain &) = delete;


        [[nodiscard]] VkSwapchainKHR swapChain() const { return m_swapChain; }
        [[nodiscard]] uint32_t imageCount() const { return m_imgCount; }
        [[nodiscard]] uint32_t numImages() const { return m_swapChainImages.size(); }
        [[nodiscard]] uint32_t queueFamily() const { return m_queueFamily; }
        [[nodiscard]] VkFormat format() const { return m_imageFormat; }
        [[nodiscard]] VkFormat depthFormat() const { return m_depthFormat; }
        [[nodiscard]] VkRenderPass renderPass() const { return m_renderPass; }
        [[nodiscard]] VkExtent2D extent() const { return m_swapChainExtent; }
        [[nodiscard]] uint32_t width() const { return m_swapChainExtent.width; }
        [[nodiscard]] uint32_t height() const { return m_swapChainExtent.height; }
        [[nodiscard]] std::vector<VkImageView> imageViews() const { return m_swapChainImageViews; }
        [[nodiscard]] VkImageView imageViews(uint32_t i) const { return m_swapChainImageViews[i]; }
        [[nodiscard]] size_t numImageViews() const { return m_swapChainImageViews.size(); }
        [[nodiscard]] std::vector<VkFramebuffer> frameBuffers() const { return m_swapChainFramebuffers; }
        [[nodiscard]] VkFramebuffer frameBuffers(uint32_t i) const { return m_swapChainFramebuffers[i]; }
        [[nodiscard]] size_t numFrameBuffers() const { return m_swapChainFramebuffers.size(); }
        [[nodiscard]] inline bool compareSwapFormats(const SwapChain& swapChain) const {
            return m_imageFormat == swapChain.format() && m_depthFormat == swapChain.depthFormat();
        }

        VkResult acquireNextImage(uint32_t* imageIndex);
        VkResult submitCommandBuffers(const VkCommandBuffer* buffers, const uint32_t* imageIndex, const std::vector<VkSemaphore>& additionalSemaphores = {}, const std::vector<VkPipelineStageFlags>& additionalStages = {});
        void beginRenderPass(VkCommandBuffer commandBuffer, uint32_t imageIndex) const;
        void endRenderPass(VkCommandBuffer commandBuffer) const;

    private:
        void init();
        void createSwapChain();
        void createImageViews();
        void createRenderPass();
        void createDepthResources();
        void createColorResources();
        void createFrameBuffers();
        void createSyncObjects();

        static VkSurfaceFormatKHR chooseSwapSurfaceFormat(const std::vector<VkSurfaceFormatKHR> &availableFormats);

        static VkPresentModeKHR chooseSwapPresentMode(const std::vector<VkPresentModeKHR> &availablePresentModes);

        VkExtent2D chooseSwapExtent(const VkSurfaceCapabilitiesKHR &capabilities);

        VkFormat findDepthFormat();

        VkFormat findSupportedFormat(const std::vector<VkFormat> &candidates, VkImageTiling tiling,
                            VkFormatFeatureFlags features);


        VkSwapchainKHR m_swapChain{};
        std::shared_ptr<SwapChain> m_oldSwapChain;

        uint32_t m_imgCount{};
        uint32_t m_queueFamily{};
        VkFormat m_imageFormat{};
        VkExtent2D m_swapChainExtent{};
        std::vector<VkImage> m_swapChainImages;
        std::vector<VkImageView> m_swapChainImageViews;
        std::vector<VkFramebuffer> m_swapChainFramebuffers;

        const Device& m_deviceRef;
        VkExtent2D m_windowExtent;
        VkRenderPass m_renderPass{};

        std::unique_ptr<Image> m_colorImage;
        VkFormat m_depthFormat{};
        std::unique_ptr<Image> m_depthImage;

        std::vector<VkSemaphore> m_imageAvailableSemaphores{};
        std::vector<VkSemaphore> m_renderFinishedSemaphores{};
        std::vector<VkFence> m_inFlightFences{};
        uint32_t currentFrame{};
    };
}

#endif //VULKANBASE_SWAPCHAIN_H
