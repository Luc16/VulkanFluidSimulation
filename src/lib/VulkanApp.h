//
// Created by luc on 14/11/22.
//

#ifndef VULKANBASE_VULKANAPP_H
#define VULKANBASE_VULKANAPP_H

#include <sstream>
#include <fstream>
#include "../../external/imgui/imgui.h"
#include "../../external/objloader/tiny_obj_loader.h"
#include "SwapChain.h"
#include "Buffer.h"
#include "Model.h"
#include "utils.h"
#include "Texture.h"
#include "Pipeline.h"
#include "Renderer.h"
#include "descriptors/DescriptorSetLayout.h"
#include "descriptors/DescriptorPool.h"
#include "descriptors/DescriptorWriter.h"
#include "Camera.h"
#include "CameraMovementController.h"
#include "RenderSystem.h"
#include "DrawableObject.h"

namespace vkb{
    class VulkanApp {
    public:
        VulkanApp(int width, int height, const std::string &appName, Device::PhysicalDeviceType type = Device::INTEL);
        VulkanApp(const VulkanApp &) = delete;
        VulkanApp &operator=(const VulkanApp &) = delete;
        ~VulkanApp() = default;

        void run();

    private:
        std::string m_appName;

    protected:
        std::vector<VkDescriptorSet> createDescriptorSets(DescriptorSetLayout &layout,
                                                          std::vector<VkDescriptorBufferInfo> bufferInfos,
                                                          std::vector<VkDescriptorImageInfo> imageInfos = {});

        virtual void mainLoop(float deltaTime) = 0;
        virtual void onCreate() = 0;
        virtual void onDestroy() {};
        virtual void onResize(int width, int height) {};

        vkb::Window window;
        vkb::Device device;

        std::unique_ptr<vkb::DescriptorPool> globalDescriptorPool;

        vkb::Renderer renderer;
    };
}



#endif //VULKANBASE_VULKANAPP_H
