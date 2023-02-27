//
// Created by luc on 26/10/22.
//

#include "Device.h"

namespace vkb {

    static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
            VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity,
            VkDebugUtilsMessageTypeFlagsEXT messageType,
            const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData,
            void* pUserData) {

        std::cerr << "validation layer: " << pCallbackData->pMessage << std::endl;

        return VK_FALSE;
    }

    VkResult CreateDebugUtilsMessengerEXT(VkInstance instance, const VkDebugUtilsMessengerCreateInfoEXT* pCreateInfo,
                                          const VkAllocationCallbacks* pAllocator, VkDebugUtilsMessengerEXT* pDebugMessenger) {
        auto func = (PFN_vkCreateDebugUtilsMessengerEXT) vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT");
        if (func != nullptr) {
            return func(instance, pCreateInfo, pAllocator, pDebugMessenger);
        } else {
            return VK_ERROR_EXTENSION_NOT_PRESENT;
        }
    }

    void DestroyDebugUtilsMessengerEXT(VkInstance instance, VkDebugUtilsMessengerEXT debugMessenger, const VkAllocationCallbacks* pAllocator) {
        auto func = (PFN_vkDestroyDebugUtilsMessengerEXT) vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT");
        if (func != nullptr) {
            func(instance, debugMessenger, pAllocator);
        }
    }

    Device::Device(const Window& window, PhysicalDeviceType type): m_windowRef(window), m_deviceType(type) {
        createInstance();
        setupDebugMessenger();
        createSurface();
        pickPhysicalDevice();
        createLogicalDevice();
        createCommandPools();
    }

    Device::~Device() {
        vkDestroyCommandPool(m_device, m_graphicsCommandPool, nullptr);
        vkDestroyCommandPool(m_device, m_computeCommandPool, nullptr);
        vkDestroyDevice(m_device, nullptr);
        if (enableValidationLayers) {
            DestroyDebugUtilsMessengerEXT(m_instance, m_debugMessenger, nullptr);
        }
        vkDestroySurfaceKHR(m_instance, m_surface, nullptr);
        vkDestroyInstance(m_instance, nullptr);
    }

    void Device::createCommandPools(){
        QueueFamilyIndices queueFamilyIndices = findQueueFamilies(m_physicalDevice);

        VkCommandPoolCreateInfo poolInfo{};
        poolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
        poolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        poolInfo.queueFamilyIndex = queueFamilyIndices.graphicsFamily.value();

        if (vkCreateCommandPool(m_device, &poolInfo, nullptr, &m_graphicsCommandPool) != VK_SUCCESS){
            throw std::runtime_error("failed to create graphics command pool!");
        }

        poolInfo.queueFamilyIndex = queueFamilyIndices.computeFamily.value();

        if (vkCreateCommandPool(m_device, &poolInfo, nullptr, &m_computeCommandPool) != VK_SUCCESS){
            throw std::runtime_error("failed to create compute/transfer command pool!");
        }
    }

    void Device::createLogicalDevice() {
        QueueFamilyIndices familyIndices = findQueueFamilies(m_physicalDevice);

        std::vector<VkDeviceQueueCreateInfo> queueCreateInfos;
        std::set<uint32_t> uniqueQueueFamilies = {familyIndices.graphicsFamily.value(), familyIndices.presentFamily.value(), familyIndices.computeFamily.value()};

        float queuePriority = 1.0f;
        for (uint32_t uQueueFamily : uniqueQueueFamilies) {
            VkDeviceQueueCreateInfo queueCreateInfo{};
            queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
            queueCreateInfo.queueFamilyIndex = uQueueFamily;
            queueCreateInfo.queueCount = 1;
            queueCreateInfo.pQueuePriorities = &queuePriority;
            queueCreateInfos.push_back(queueCreateInfo);
        }

        VkPhysicalDeviceFeatures deviceFeatures{};
        deviceFeatures.samplerAnisotropy = VK_TRUE;
        deviceFeatures.fillModeNonSolid = VK_TRUE;
//        deviceFeatures.sampleRateShading = VK_TRUE; // Activate for more image quality

        VkDeviceCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
        createInfo.queueCreateInfoCount = static_cast<uint32_t>(queueCreateInfos.size());
        createInfo.pQueueCreateInfos = queueCreateInfos.data();
        createInfo.pEnabledFeatures = &deviceFeatures;

        createInfo.enabledExtensionCount = static_cast<uint32_t>(deviceExtensions.size());
        createInfo.ppEnabledExtensionNames = deviceExtensions.data();

        if (enableValidationLayers) {
            createInfo.enabledLayerCount = static_cast<uint32_t>(validationLayers.size());
            createInfo.ppEnabledLayerNames = validationLayers.data();
        } else {
            createInfo.enabledLayerCount = 0;
        }

        if (vkCreateDevice(m_physicalDevice, &createInfo, nullptr, &m_device) != VK_SUCCESS) {
            throw std::runtime_error("failed to create logical m_deviceRef!");
        }
        vkGetDeviceQueue(m_device, familyIndices.graphicsFamily.value(), 0, &m_graphicsQueue);
        vkGetDeviceQueue(m_device, familyIndices.presentFamily.value(), 0, &m_presentQueue);
        vkGetDeviceQueue(m_device, familyIndices.computeFamily.value(), 0, &m_computeQueue);
    }

    void Device::pickPhysicalDevice() {
        uint32_t deviceCount = 0;
        vkEnumeratePhysicalDevices(m_instance, &deviceCount, nullptr);

        if (deviceCount == 0) throw std::runtime_error("failed to find GPUs with Vulkan support!");

        std::vector<VkPhysicalDevice> devices(deviceCount);
        vkEnumeratePhysicalDevices(m_instance, &deviceCount, devices.data());

        std::cout << "Physical device list:\n";
        bool deviceChosen = false;
        for (const auto& pDevice : devices){
            VkPhysicalDeviceProperties properties;
            vkGetPhysicalDeviceProperties(pDevice, &properties);
            std::cout << '\t' << properties.deviceName << " ID: " << properties.deviceID << '\n';


            if (isDeviceSuitable(pDevice) && !deviceChosen && properties.deviceID == m_deviceType) {
                deviceChosen = true;
                m_physicalDevice = pDevice;
                m_msaaSamples = getMaxUsableSampleCount();
            }
        }

        if (m_physicalDevice == VK_NULL_HANDLE) {
            throw std::runtime_error("failed to find a suitable GPU!");
        }

        VkPhysicalDeviceProperties properties;
        vkGetPhysicalDeviceProperties(m_physicalDevice, &properties);
        std::cout << "\nUsing physical device: " << properties.deviceName << '\n';
    }

    void Device::createSurface(){
        if (glfwCreateWindowSurface(m_instance, m_windowRef.window(), nullptr, &m_surface) != VK_SUCCESS){
            throw std::runtime_error("failed to create m_windowRef surface!");
        }
    }

    void Device::setupDebugMessenger() {
        if (!enableValidationLayers) return;

        VkDebugUtilsMessengerCreateInfoEXT createInfo;
        populateDebugMessengerCreateInfo(createInfo);

        if (CreateDebugUtilsMessengerEXT(m_instance, &createInfo, nullptr, &m_debugMessenger) != VK_SUCCESS) {
            throw std::runtime_error("failed to set up debug messenger!");
        }

    }

    void Device::createInstance() {
        if (enableValidationLayers && !checkValidationLayerSupport()) {
            throw std::runtime_error("validation layers requested, but not available!");
        }

        VkApplicationInfo appInfo{};
        appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
        appInfo.pApplicationName = "Hello Triangle";
        appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
        appInfo.pEngineName = "No Engine";
        appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
        appInfo.apiVersion = VK_API_VERSION_1_0;

        VkInstanceCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
        createInfo.pApplicationInfo = &appInfo;

        auto extensions = getRequiredExtension();
        createInfo.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
        createInfo.ppEnabledExtensionNames = extensions.data();

        VkDebugUtilsMessengerCreateInfoEXT debugCreateInfo{};
        if (enableValidationLayers) {
            createInfo.enabledLayerCount = static_cast<uint32_t>(validationLayers.size());
            createInfo.ppEnabledLayerNames = validationLayers.data();

            populateDebugMessengerCreateInfo(debugCreateInfo);
            createInfo.pNext = (VkDebugUtilsMessengerCreateInfoEXT*) &debugCreateInfo;
        } else {
            createInfo.enabledLayerCount = 0;

            createInfo.pNext = nullptr;
        }

        if (vkCreateInstance(&createInfo, nullptr, &m_instance) != VK_SUCCESS) {
            throw std::runtime_error("failed to create instance!");
        }
    }

    bool Device::isDeviceSuitable(VkPhysicalDevice pDevice) {
        QueueFamilyIndices familyIndices = findQueueFamilies(pDevice);

        bool extensionsSupported = checkDeviceExtensionSupport(pDevice);

        VkPhysicalDeviceFeatures supportedFeatures;
        vkGetPhysicalDeviceFeatures(pDevice, &supportedFeatures);

        bool swapChainAdequate = false;
        if (extensionsSupported) {
            SwapChainSupportDetails swapChainSupport = querySwapChainSupport(pDevice);
            swapChainAdequate = !swapChainSupport.formats.empty() && !swapChainSupport.presentModes.empty();
        }

        return familyIndices.isComplete() && extensionsSupported && swapChainAdequate && supportedFeatures.samplerAnisotropy;
    }

    VkSampleCountFlagBits Device::getMaxUsableSampleCount() {
        VkPhysicalDeviceProperties physicalDeviceProperties{};
        vkGetPhysicalDeviceProperties(m_physicalDevice, &physicalDeviceProperties);

        VkSampleCountFlags counts = physicalDeviceProperties.limits.framebufferColorSampleCounts & physicalDeviceProperties.limits.framebufferDepthSampleCounts;
        std::vector<VkSampleCountFlagBits> sampleCounts = {
                VK_SAMPLE_COUNT_64_BIT,
                VK_SAMPLE_COUNT_32_BIT,
                VK_SAMPLE_COUNT_16_BIT,
                VK_SAMPLE_COUNT_8_BIT,
                VK_SAMPLE_COUNT_4_BIT,
                VK_SAMPLE_COUNT_2_BIT,
        };
        bool validCandidate = false;
        for (auto & sampleCount : sampleCounts) {
            if (counts & sampleCount) validCandidate = true;
            if (validCandidate && sampleCount <= MAX_SAMPLES) return sampleCount;
        }

        return VK_SAMPLE_COUNT_1_BIT;
    }

    bool Device::checkDeviceExtensionSupport(VkPhysicalDevice pDevice){
        uint32_t extensionCount = 0;
        vkEnumerateDeviceExtensionProperties(pDevice, nullptr, &extensionCount, nullptr);

        std::vector<VkExtensionProperties> availableExtensions(extensionCount);
        vkEnumerateDeviceExtensionProperties(pDevice, nullptr, &extensionCount, availableExtensions.data());

        std::set<std::string> requiredExtensions(deviceExtensions.begin(), deviceExtensions.end());

        for (const auto& extension : availableExtensions) {
            requiredExtensions.erase(extension.extensionName);
        }
        if (!requiredExtensions.empty()){
            for (const auto& extension : requiredExtensions) {
                std::cerr << "Extension: " << extension << " is unavailable\n";
            }
        }
        return requiredExtensions.empty();
    }

    bool Device::checkValidationLayerSupport(){
        uint32_t layerCount;
        vkEnumerateInstanceLayerProperties(&layerCount, nullptr);

        std::vector<VkLayerProperties> availableLayers(layerCount);
        vkEnumerateInstanceLayerProperties(&layerCount, availableLayers.data());

        for (const char* layerName: validationLayers){
            bool layerFound = false;
            for (VkLayerProperties& layerProperties : availableLayers){
                if (strcmp(layerName, layerProperties.layerName) == 0){
                    layerFound = true;
                    break;
                }
            }
            if (!layerFound) return false;
        }
        return true;
    }

    std::vector<const char*> Device::getRequiredExtension() const{
        uint32_t glfwExtensionCount = 0;
        const char** glfwExtensions;
        glfwExtensions = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);
        checkNecessaryExtensions(glfwExtensions, glfwExtensionCount);

        std::vector<const char *> extensions(glfwExtensions, glfwExtensions + glfwExtensionCount);

        if (enableValidationLayers){
            extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);
        }

        return extensions;
    }

    void Device::checkNecessaryExtensions(const char** glfwExtensions, uint32_t count) {
        uint32_t extensionCount = 0;
        vkEnumerateInstanceExtensionProperties(nullptr, &extensionCount, nullptr);
        std::vector<VkExtensionProperties> extensions(extensionCount);
        vkEnumerateInstanceExtensionProperties(nullptr, &extensionCount, extensions.data());

        for (int i = 0; i < count; i++){
            bool foundExtension = false;
            for (const auto& extension : extensions) {
                if (strcmp(extension.extensionName, glfwExtensions[i]) == 0){
                    foundExtension = true;
                    break;
                }
            }
            if (!foundExtension) {
                std::string errorMessage = "extension " + std::string(glfwExtensions[i]) + " not found!";
                throw std::runtime_error(errorMessage);
            }
        }
    }

    void Device::populateDebugMessengerCreateInfo(VkDebugUtilsMessengerCreateInfoEXT& createInfo) {
        createInfo = {};
        createInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
        createInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_VERBOSE_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
        createInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
        createInfo.pfnUserCallback = debugCallback;
        createInfo.pUserData = nullptr; // Optional
    }

    Device::QueueFamilyIndices Device::findQueueFamilies(VkPhysicalDevice physicalDevice) const {
        Device::QueueFamilyIndices familyIndices;
        uint32_t queueFamilyCount = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &queueFamilyCount, nullptr);


        std::vector<VkQueueFamilyProperties> queueFamilies(queueFamilyCount);
        vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &queueFamilyCount, queueFamilies.data());

        int i = 0;
        for (const auto& iQueueFamily : queueFamilies){
            if (iQueueFamily.queueFlags & VK_QUEUE_GRAPHICS_BIT){
                familyIndices.graphicsFamily = i;
            } else if (iQueueFamily.queueCount & VK_QUEUE_COMPUTE_BIT) {
                familyIndices.computeFamily = i;
            }

            VkBool32 presentSupport = false;
            vkGetPhysicalDeviceSurfaceSupportKHR(physicalDevice, i, m_surface, &presentSupport);
            if (presentSupport) familyIndices.presentFamily = i;

            if (familyIndices.isComplete()) break;

            i++;
        }

        return familyIndices;
    }

    uint32_t Device::findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags properties) const {
        VkPhysicalDeviceMemoryProperties memProperties;
        vkGetPhysicalDeviceMemoryProperties(m_physicalDevice, &memProperties);
        for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
            if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties) {
                return i;
            }
        }

        throw std::runtime_error("failed to find suitable m_memory type!");
    }


    void Device::executeSingleCommand(const std::function<void(VkCommandBuffer&)>& function, bool useGraphicsQueue) const {
        VkCommandBufferAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
        allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        if (useGraphicsQueue)
            allocInfo.commandPool = m_graphicsCommandPool;
        else
            allocInfo.commandPool = m_computeCommandPool;
        allocInfo.commandBufferCount = 1;

        VkCommandBuffer commandBuffer;
        vkAllocateCommandBuffers(m_device, &allocInfo, &commandBuffer);

        VkCommandBufferBeginInfo beginInfo{};
        beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
        beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

        vkBeginCommandBuffer(commandBuffer, &beginInfo);

        function(commandBuffer);

        vkEndCommandBuffer(commandBuffer);

        VkSubmitInfo submitInfo{};
        submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
        submitInfo.commandBufferCount = 1;
        submitInfo.pCommandBuffers = &commandBuffer;

        if (useGraphicsQueue) {
            vkQueueSubmit(m_graphicsQueue,1, &submitInfo, VK_NULL_HANDLE);
            vkQueueWaitIdle(m_graphicsQueue);
        } else {
            vkQueueSubmit(m_computeQueue,1, &submitInfo, VK_NULL_HANDLE);
            vkQueueWaitIdle(m_computeQueue);
        }

        vkFreeCommandBuffers(m_device, allocInfo.commandPool, 1, &commandBuffer);

    }

    void Device::copyBufferToImage(VkBuffer buffer, VkImage image, uint32_t width, uint32_t height) const {
        executeSingleCommand([buffer, image, width, height](VkCommandBuffer commandBuffer) {
            VkBufferImageCopy region{};
            region.bufferOffset = 0;
            region.bufferRowLength = 0;
            region.bufferImageHeight = 0;

            region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
            region.imageSubresource.mipLevel = 0;
            region.imageSubresource.baseArrayLayer = 0;
            region.imageSubresource.layerCount = 1;

            region.imageOffset = {0, 0, 0};
            region.imageExtent = {
                    width,
                    height,
                    1
            };

            vkCmdCopyBufferToImage(
                    commandBuffer,
                    buffer,
                    image,
                    VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                    1,
                    &region
            );
        });
    }

    void Device::createBuffer(VkDeviceSize size, VkBufferUsageFlags usage, VkMemoryPropertyFlags properties,
                              VkBuffer& buffer, VkDeviceMemory& bufferMemory) const {
        auto familyIndices = findQueueFamilies(m_physicalDevice);
        uint32_t queueFamilyIndices[] = {familyIndices.graphicsFamily.value(), familyIndices.computeFamily.value()};

        VkBufferCreateInfo bufferInfo{};
        bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
        bufferInfo.size = size;
        bufferInfo.usage = usage;
        bufferInfo.sharingMode = VK_SHARING_MODE_CONCURRENT;
        bufferInfo.queueFamilyIndexCount = 2;
        bufferInfo.pQueueFamilyIndices = queueFamilyIndices;

        if (vkCreateBuffer(m_device, &bufferInfo, nullptr, &buffer) != VK_SUCCESS){
            throw std::runtime_error("failed to create vertex m_buffer!");
        }

        VkMemoryRequirements memRequirements{};
        vkGetBufferMemoryRequirements(m_device, buffer, &memRequirements);

        VkMemoryAllocateInfo allocInfo{};
        allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
        allocInfo.allocationSize = memRequirements.size;
        allocInfo.memoryTypeIndex = findMemoryType(memRequirements.memoryTypeBits, properties);

        if (vkAllocateMemory(m_device, &allocInfo, nullptr, &bufferMemory) != VK_SUCCESS) {
            throw std::runtime_error("failed to allocate vertex m_buffer m_memory!");
        }

        vkBindBufferMemory(m_device, buffer, bufferMemory, 0);
    }

    void Device::copyBuffer(VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size) const {
        executeSingleCommand([srcBuffer, dstBuffer, size](VkCommandBuffer commandBuffer) {
            VkBufferCopy copyRegion{};
            copyRegion.srcOffset = 0;
            copyRegion.dstOffset = 0;
            copyRegion.size = size;
            vkCmdCopyBuffer(commandBuffer, srcBuffer, dstBuffer, 1, &copyRegion);
        });
    }

    Device::SwapChainSupportDetails Device::querySwapChainSupport(VkPhysicalDevice physicalDevice) const {
        SwapChainSupportDetails details;

        vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physicalDevice, m_surface, &details.capabilities);

        uint32_t formatCount = 0;
        vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, m_surface, &formatCount, nullptr);

        if (formatCount != 0) {
            details.formats.resize(formatCount);
            vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, m_surface, &formatCount, details.formats.data());

        }

        uint32_t presentModeCount;
        vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, m_surface, &presentModeCount, nullptr);

        if (presentModeCount != 0) {
            details.presentModes.resize(presentModeCount);
            vkGetPhysicalDeviceSurfacePresentModesKHR(physicalDevice, m_surface, &presentModeCount, details.presentModes.data());
        }

        return details;
    }
}