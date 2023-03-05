//
// Created by luc on 03/11/22.
//

#ifndef VULKANBASE_RENDERER_H
#define VULKANBASE_RENDERER_H

#include <vulkan/vulkan.h>
#include <vector>
#include "SwapChain.h"

namespace vkb {
    class Renderer {
    public:
        Renderer(Window& window, const Device& device);

        ~Renderer();
        Renderer(const Renderer &) = delete;
        Renderer &operator=(const Renderer &) = delete;

        [[nodiscard]] uint32_t imageCount() const { return m_swapChain->imageCount(); }
        [[nodiscard]] uint32_t numImages() const { return m_swapChain->numImages(); }
        [[nodiscard]] uint32_t queueFamily() const { return m_swapChain->queueFamily(); }

        [[nodiscard]] uint32_t currentFrame() const { return m_currentFrame; }
        [[nodiscard]] VkRenderPass renderPass() const { return m_swapChain->renderPass(); }
        [[nodiscard]] float getSwapChainAspectRatio() const {return (float) m_swapChain->width() / (float) m_swapChain->height(); }
        [[nodiscard]] VkCommandBuffer getCurrentCommandBuffer() const {
            if (!m_isFrameStarted) throw std::runtime_error("Cannot getBuffer command buffer when frame not in progress");
            return m_commandBuffers[m_currentFrame];
        }

        void runRenderPass(const std::function<void(VkCommandBuffer&)>& function) const;
        void runFrame(const std::function<void(VkCommandBuffer&)>& function, const std::vector<VkSemaphore>& additionalSemaphores = {}, const std::vector<VkPipelineStageFlags>& additionalStages = {});
        void activateImGui(VkDescriptorPool descriptorPool);
    private:
        void recreateSwapChain();
        void createCommandBuffers();
        void freeCommandBuffers();
        void initImGuiVulkan(VkDescriptorPool descriptorPool);
        static void checkVkResultImGui(VkResult err);

        std::unique_ptr<vkb::SwapChain> m_swapChain;
        std::vector<VkCommandBuffer> m_commandBuffers;
        uint32_t m_currentFrame{0};
        uint32_t m_imageIndex{};
        bool m_isFrameStarted{false};

        const Device& m_deviceRef;
        Window& m_windowRef;

        bool m_imGuiActivated = false;
    };
}



#endif //VULKANBASE_RENDERER_H
