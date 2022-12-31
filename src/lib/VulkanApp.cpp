//
// Created by luc on 14/11/22.
//

#include "VulkanApp.h"

namespace vkb {

    VulkanApp::VulkanApp(int width, int height, const std::string &appName, Device::PhysicalDeviceType type):
    window(width, height, appName), device(window, type), renderer(window, device), m_appName(appName) {
        globalDescriptorPool = vkb::DescriptorPool::Builder(device)
                .addPoolSize({ VK_DESCRIPTOR_TYPE_SAMPLER, 1000 })
                .addPoolSize({ VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, 1000 })
                .addPoolSize({ VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE, 1000 })
                .addPoolSize({ VK_DESCRIPTOR_TYPE_STORAGE_IMAGE, 1000 })
                .addPoolSize({ VK_DESCRIPTOR_TYPE_UNIFORM_TEXEL_BUFFER, 1000 })
                .addPoolSize({ VK_DESCRIPTOR_TYPE_STORAGE_TEXEL_BUFFER, 1000 })
                .addPoolSize({ VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1000 })
                .addPoolSize({ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1000 })
                .addPoolSize({ VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER_DYNAMIC, 1000 })
                .addPoolSize({ VK_DESCRIPTOR_TYPE_STORAGE_BUFFER_DYNAMIC, 1000 })
                .addPoolSize({ VK_DESCRIPTOR_TYPE_INPUT_ATTACHMENT, 1000 })
                .setFlags(VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT)
                .setMaxSetsTimesSizes(1000)
                .build();

        renderer.activateImGui(globalDescriptorPool->descriptorPool());

    }

    void VulkanApp::run() {
        onCreate();

        auto currentTime = std::chrono::high_resolution_clock::now();
        float deltaTime, accTime = 0;
        uint32_t frames = 0;

        while (!window.shouldClose()) {
            auto newTime = std::chrono::high_resolution_clock::now();
            deltaTime =
                    std::chrono::duration<float, std::chrono::seconds::period>(newTime - currentTime).count();
            currentTime = newTime;
            accTime += deltaTime;
            frames++;

            glfwPollEvents();

            if (window.frameBufferResized())
                onResize(window.width(), window.height());

            mainLoop(deltaTime);

            if (accTime > 0.5f) {
                double fps = double(frames) / accTime;

                std::stringstream ss;
                ss << m_appName <<" " << "[" << fps << " FPS]";

                glfwSetWindowTitle(window.window(), ss.str().c_str());

                frames = 0;
                accTime = 0.0f;
            }
        }
        vkDeviceWaitIdle(device.device());

        onDestroy();

    }

    std::vector<VkDescriptorSet> VulkanApp::createDescriptorSets(vkb::DescriptorSetLayout& layout,
                                                      std::vector<VkDescriptorBufferInfo> bufferInfos,
                                                      std::vector<VkDescriptorImageInfo> imageInfos) {
        std::vector<VkDescriptorSet> descriptorSets(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);

        for (auto & descriptorSet : descriptorSets) {

            auto writer = vkb::DescriptorWriter(layout, *globalDescriptorPool);

            for (auto& bufferInfo : bufferInfos){
                writer.writeBuffer(0, &bufferInfo);
            }

            for (auto& imageInfo : imageInfos){
                writer.writeImage(1, &imageInfo);
            }

            writer.build(descriptorSet);
        }

        return descriptorSets;
    }
}