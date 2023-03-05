//
// Created by luc on 02/11/22.
//

#ifndef VULKANBASE_PIPELINE_H
#define VULKANBASE_PIPELINE_H

#include "Device.h"

namespace vkb {
    class GraphicsPipeline {
    public:
        struct PipelineConfigInfo {
            PipelineConfigInfo() = default;
//            PipelineConfigInfo(const PipelineConfigInfo&) = delete;
//            PipelineConfigInfo& operator=(const PipelineConfigInfo&) = delete;

            VkPipelineInputAssemblyStateCreateInfo inputAssembly{};
            VkPipelineViewportStateCreateInfo viewportState{};
            VkPipelineRasterizationStateCreateInfo rasterizer{};
            VkPipelineMultisampleStateCreateInfo multisampling{};
            VkPipelineDepthStencilStateCreateInfo depthStencilInfo{};
            VkPipelineColorBlendAttachmentState colorBlendAttachment{};
            VkPipelineColorBlendStateCreateInfo colorBlending{};
            std::vector<VkDynamicState> dynamicStateEnables{};
            VkPipelineDynamicStateCreateInfo dynamicState{};
            std::vector<VkVertexInputBindingDescription> bindingDescription{};
            std::vector<VkVertexInputAttributeDescription> attributeDescription{};

            VkPipelineLayout pipelineLayout = VK_NULL_HANDLE;
            VkRenderPass renderPass = VK_NULL_HANDLE;
            uint32_t subpass = 0;

            void setPipelineLayout(VkPipelineLayout layout) { pipelineLayout = layout; }
            void setRenderPass(VkRenderPass vkRenderPass) { renderPass = vkRenderPass; }

            void enablePolygonLineMode() { rasterizer.polygonMode = VK_POLYGON_MODE_LINE; }
            void enableAlphaBlending();

        };

        GraphicsPipeline(const Device& device, const std::string& vertShaderPath, const std::string& fragShaderPath, PipelineConfigInfo& configInfo);
        ~GraphicsPipeline();
        GraphicsPipeline(const GraphicsPipeline &) = delete;
        GraphicsPipeline &operator=(const GraphicsPipeline &) = delete;

        void createGraphicsPipeline(const std::string& vertShaderPath, const std::string& fragShaderPath, PipelineConfigInfo& configInfo);
        void bind(VkCommandBuffer commandBuffer);

        static PipelineConfigInfo defaultConfigInfo(VkPipelineLayout layout, VkRenderPass renderPass);
        static PipelineConfigInfo defaultConfigInfo();

        static std::vector<char> readFile(const std::string& filename);
        static VkShaderModule createShaderModule(const Device& device, const std::vector<char>& code);

    private:

        const Device& m_deviceRef;
        VkPipeline m_pipeline{};
    };
}

#endif //VULKANBASE_PIPELINE_H
