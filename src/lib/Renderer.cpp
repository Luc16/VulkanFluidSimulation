//
// Created by luc on 03/11/22.
//

#include <stdexcept>
#include "Renderer.h"
#include "../../external/imgui/imgui_impl_vulkan.h"
#include "../../external/imgui/imgui_impl_glfw.h"
#include "../../external/imgui/imgui_internal.h"

namespace vkb {

    Renderer::Renderer(Window &window, const Device &device): m_windowRef(window) , m_deviceRef(device) {
        recreateSwapChain();
        createCommandBuffers();
    }

    Renderer::~Renderer() {
        freeCommandBuffers();

        if (m_imGuiActivated) {
            ImGui_ImplGlfw_Shutdown();
            ImGui_ImplVulkan_Shutdown();
            ImGui::DestroyContext();
        }
    }

    void Renderer::activateImGui(VkDescriptorPool descriptorPool){
        if (m_imGuiActivated) throw std::runtime_error("Can't start ImGui more than once");

        initImGuiVulkan(descriptorPool);
        m_imGuiActivated = true;
    }

    void Renderer::runFrame(const std::function<void(VkCommandBuffer&)>& function) {
        if (m_isFrameStarted) throw std::runtime_error("Can't run frame while another frame is running");

        VkResult result = m_swapChain->acquireNextImage(&m_imageIndex);
        if (result == VK_ERROR_OUT_OF_DATE_KHR){
            recreateSwapChain();
            return;
        } else if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR){
            throw std::runtime_error("failed to acquire swap chain image!");
        }

        if (m_imGuiActivated) {
            ImGui_ImplVulkan_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();
        }

        m_isFrameStarted = true;
        auto commandBuffer = getCurrentCommandBuffer();
        vkResetCommandBuffer(commandBuffer, /*VkCommandBufferResetFlagBits*/ 0);

        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = 0;
        beginInfo.pInheritanceInfo = nullptr;

        if (vkBeginCommandBuffer(commandBuffer, &beginInfo) != VK_SUCCESS) {
            throw std::runtime_error("failed to begin recording command m_buffer!");
        }

        function(commandBuffer);

        if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS) {
            throw std::runtime_error("failed to record command m_buffer!");
        }

        result = m_swapChain->submitCommandBuffers(&commandBuffer, &m_imageIndex);
        if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR || m_windowRef.frameBufferResized()) {
            m_windowRef.frameBufferNotResized();
            recreateSwapChain();
        } else if (result != VK_SUCCESS) {
            throw std::runtime_error("failed to present swap chain image!");
        }
        if (m_imGuiActivated) ImGui::EndFrame();

        m_currentFrame = (m_currentFrame + 1) % vkb::SwapChain::MAX_FRAMES_IN_FLIGHT;
        m_isFrameStarted = false;
    }

    void Renderer::runRenderPass(const std::function<void(VkCommandBuffer&)> &function) const {
        if (!m_isFrameStarted) throw std::runtime_error("Can't run renderPass with no frames running");

        auto commandBuffer = getCurrentCommandBuffer();
        m_swapChain->beginRenderPass(commandBuffer, m_imageIndex);

        function(commandBuffer);

        if (m_imGuiActivated) {
            ImGui::Render();
            ImGui_ImplVulkan_RenderDrawData(ImGui::GetDrawData(), commandBuffer);
        }

        m_swapChain->endRenderPass(commandBuffer);

    }

    void Renderer::recreateSwapChain() {
        int width = 0, height = 0;
        glfwGetFramebufferSize(m_windowRef.window(), &width, &height);
        if (width == 0 && height == 0){
            glfwGetFramebufferSize(m_windowRef.window(), &width, &height);
            glfwWaitEvents();
        }

        vkDeviceWaitIdle(m_deviceRef.device());

        if (m_swapChain == nullptr){
            m_swapChain = std::make_unique<vkb::SwapChain>(m_deviceRef, m_windowRef.extent());
            return;
        }
        std::shared_ptr<SwapChain> oldSwapChain = std::move(m_swapChain);
        m_swapChain = std::make_unique<SwapChain>(m_deviceRef, m_windowRef.extent(), oldSwapChain);

        if (!oldSwapChain->compareSwapFormats(*m_swapChain)) {
            throw std::runtime_error("Swap chain image(or depth) format has changed!");
        }

        if (m_imGuiActivated) ImGui_ImplVulkan_SetMinImageCount(m_swapChain->imageCount());

    }

    void Renderer::createCommandBuffers(){
        m_commandBuffers.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);

        VkCommandBufferAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.commandPool = m_deviceRef.graphicsCommandPool();
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        allocInfo.commandBufferCount = static_cast<uint32_t>(m_commandBuffers.size());

        if (vkAllocateCommandBuffers(m_deviceRef.device(), &allocInfo, m_commandBuffers.data()) != VK_SUCCESS) {
            throw std::runtime_error("failed to allocate command buffers!");
        }

    }

    void Renderer::freeCommandBuffers(){
        vkFreeCommandBuffers(m_deviceRef.device(), m_deviceRef.graphicsCommandPool(),
                             static_cast<uint32_t>(m_commandBuffers.size()), m_commandBuffers.data());
        m_commandBuffers.clear();

    }

    void Renderer::checkVkResultImGui(VkResult err) {
        if (err == 0)
            return;
        fprintf(stderr, "[vulkan] Error: VkResult = %d\n", err);
        if (err < 0)
            abort();
    }

    void Renderer::initImGuiVulkan(VkDescriptorPool descriptorPool) {
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO(); (void)io;
        ImGui_ImplGlfw_InitForVulkan(m_windowRef.window(), true);
        ImGui::StyleColorsDark();


        ImGui_ImplVulkan_InitInfo initInfo = {};
        initInfo.Instance = m_deviceRef.instance();
        initInfo.PhysicalDevice = m_deviceRef.physicalDevice();
        initInfo.Device = m_deviceRef.device();
        initInfo.QueueFamily = queueFamily();
        initInfo.Queue = m_deviceRef.graphicsQueue();
        initInfo.PipelineCache = nullptr;
        initInfo.DescriptorPool = descriptorPool;
        initInfo.Subpass = 0;
        initInfo.MinImageCount = imageCount();
        initInfo.ImageCount = static_cast<uint32_t>(numImages());
        initInfo.MSAASamples = m_deviceRef.msaaSamples();
        initInfo.Allocator = nullptr;
        initInfo.CheckVkResultFn = checkVkResultImGui;

        ImGui_ImplVulkan_Init(&initInfo, renderPass());

        {
            VkResult err;
            // Use any command queue
            VkCommandPool command_pool = m_deviceRef.graphicsCommandPool();

            err = vkResetCommandPool(m_deviceRef.device(), command_pool, 0);
            checkVkResultImGui(err);
            m_deviceRef.executeSingleCommand([](VkCommandBuffer commandBuffer) {
                ImGui_ImplVulkan_CreateFontsTexture(commandBuffer);
            }, true);

            ImGui_ImplVulkan_DestroyFontUploadObjects();
        }
    }

}