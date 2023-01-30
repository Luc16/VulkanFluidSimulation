//
// Created by luc on 02/11/22.
//

#include <fstream>
#include "Pipeline.h"
#include "Model.h"


namespace vkb {

    Pipeline::Pipeline(const Device &device, const std::string &vertShaderPath, const std::string &fragShaderPath,
                       Pipeline::PipelineConfigInfo &configInfo): m_deviceRef(device) {
        createGraphicsPipeline(vertShaderPath, fragShaderPath, configInfo);

    }

    Pipeline::~Pipeline() {
        vkDestroyPipeline(m_deviceRef.device(), m_pipeline, nullptr);
    }

    void Pipeline::createGraphicsPipeline(const std::string &vertShaderPath, const std::string &fragShaderPath,
                                          Pipeline::PipelineConfigInfo &configInfo) {

        if (configInfo.pipelineLayout == VK_NULL_HANDLE) throw std::runtime_error("Trying to initialize graphics pipeline with no layout");
        if (configInfo.renderPass == VK_NULL_HANDLE) throw std::runtime_error("Trying to initialize graphics pipeline with no renderPass");
        auto vertShaderCode = readFile(vertShaderPath);
        auto fragShaderCode = readFile(fragShaderPath);

        VkShaderModule vertShaderModule = createShaderModule(vertShaderCode);
        VkShaderModule fragShaderModule = createShaderModule(fragShaderCode);

        VkPipelineShaderStageCreateInfo vertShaderStageCreateInfo{};
        vertShaderStageCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        vertShaderStageCreateInfo.stage = VK_SHADER_STAGE_VERTEX_BIT;
        vertShaderStageCreateInfo.module = vertShaderModule;
        vertShaderStageCreateInfo.pName = "main";

        VkPipelineShaderStageCreateInfo fragShaderStageInfo{};
        fragShaderStageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
        fragShaderStageInfo.stage = VK_SHADER_STAGE_FRAGMENT_BIT;
        fragShaderStageInfo.module = fragShaderModule;
        fragShaderStageInfo.pName = "main";

        VkPipelineShaderStageCreateInfo shaderStages[] = {vertShaderStageCreateInfo, fragShaderStageInfo};

        VkPipelineVertexInputStateCreateInfo vertexInputInfo{};
        vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
        vertexInputInfo.vertexBindingDescriptionCount = static_cast<uint32_t>(configInfo.bindingDescription.size());
        vertexInputInfo.pVertexBindingDescriptions = configInfo.bindingDescription.data();
        vertexInputInfo.vertexAttributeDescriptionCount = static_cast<uint32_t>(configInfo.attributeDescription.size());
        vertexInputInfo.pVertexAttributeDescriptions = configInfo.attributeDescription.data();

        // inicializa o multisampling
        configInfo.multisampling.rasterizationSamples = m_deviceRef.msaaSamples();

        VkGraphicsPipelineCreateInfo pipelineInfo{};
        pipelineInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
        pipelineInfo.stageCount = 2;
        pipelineInfo.pStages = shaderStages;
        pipelineInfo.pVertexInputState = &vertexInputInfo;
        pipelineInfo.pInputAssemblyState = &configInfo.inputAssembly;
        pipelineInfo.pViewportState = &configInfo.viewportState;
        pipelineInfo.pRasterizationState = &configInfo.rasterizer;
        pipelineInfo.pMultisampleState = &configInfo.multisampling;
        pipelineInfo.pDepthStencilState = &configInfo.depthStencilInfo;
        pipelineInfo.pColorBlendState = &configInfo.colorBlending;
        pipelineInfo.pDynamicState = &configInfo.dynamicState;
        pipelineInfo.layout = configInfo.pipelineLayout;
        pipelineInfo.renderPass = configInfo.renderPass;
        pipelineInfo.subpass = configInfo.subpass;
        pipelineInfo.basePipelineHandle = VK_NULL_HANDLE;
        pipelineInfo.basePipelineIndex = -1;

        if (vkCreateGraphicsPipelines(m_deviceRef.device(), VK_NULL_HANDLE, 1,
                                      &pipelineInfo, nullptr, &m_pipeline) != VK_SUCCESS) {
            throw std::runtime_error("failed to create graphics pipeline!");
        }


        vkDestroyShaderModule(m_deviceRef.device(), vertShaderModule, nullptr);
        vkDestroyShaderModule(m_deviceRef.device(), fragShaderModule, nullptr);
    }

    VkShaderModule Pipeline::createShaderModule(const std::vector<char>& code){
        VkShaderModuleCreateInfo createInfo{};
        createInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
        createInfo.codeSize = code.size();
        createInfo.pCode = reinterpret_cast<const uint32_t*>(code.data());

        VkShaderModule shaderModule;
        if (vkCreateShaderModule(m_deviceRef.device(), &createInfo, nullptr, &shaderModule) != VK_SUCCESS){
            throw std::runtime_error("failed to create shader module!");
        }

        return shaderModule;
    }

    void Pipeline::bind(VkCommandBuffer commandBuffer) {
        vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pipeline);
    }

    std::vector<char> Pipeline::readFile(const std::string& filename){
        std::ifstream file;
        file.open(filename, std::ios::ate | std::ios::binary);

        if (!file.is_open()) {
            throw std::runtime_error("failed to open file: " + filename);
        }

        long fileSize = (long) file.tellg();
        std::vector<char> buffer(fileSize);
        file.seekg(0);
        file.read(buffer.data(), fileSize);
        file.close();

        return buffer;
    }

    Pipeline::PipelineConfigInfo Pipeline::defaultConfigInfo() {
        PipelineConfigInfo configInfo;

        configInfo.inputAssembly.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
        configInfo.inputAssembly.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
        configInfo.inputAssembly.primitiveRestartEnable = VK_FALSE;

        configInfo.viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
        configInfo.viewportState.viewportCount = 1;
        configInfo.viewportState.scissorCount = 1;

        configInfo.rasterizer.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
        configInfo.rasterizer.depthClampEnable = VK_FALSE;
        configInfo.rasterizer.rasterizerDiscardEnable = VK_FALSE;
        configInfo.rasterizer.polygonMode = VK_POLYGON_MODE_FILL;
        configInfo.rasterizer.lineWidth = 1.0f;
        configInfo.rasterizer.cullMode = VK_CULL_MODE_BACK_BIT;
        configInfo.rasterizer.frontFace = VK_FRONT_FACE_COUNTER_CLOCKWISE;
        configInfo.rasterizer.depthBiasEnable = VK_FALSE;
        configInfo.rasterizer.depthBiasConstantFactor = 0.0f; // Optional
        configInfo.rasterizer.depthBiasClamp = 0.0f; // Optional
        configInfo.rasterizer.depthBiasSlopeFactor = 0.0f; // Optional

        configInfo.multisampling.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
        configInfo.multisampling.sampleShadingEnable = VK_FALSE; // VK_TRUE for more image quality
        configInfo.multisampling.minSampleShading = 1.0f; // 0.2f for more image quality
        configInfo.multisampling.pSampleMask = nullptr; // Optional
        configInfo.multisampling.alphaToCoverageEnable = VK_FALSE; // Optional
        configInfo.multisampling.alphaToOneEnable = VK_FALSE; // Optional

        configInfo.depthStencilInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
        configInfo.depthStencilInfo.depthTestEnable = VK_TRUE;
        configInfo.depthStencilInfo.depthWriteEnable = VK_TRUE;
        configInfo.depthStencilInfo.depthCompareOp = VK_COMPARE_OP_LESS;
        configInfo.depthStencilInfo.depthBoundsTestEnable = VK_FALSE;
        configInfo.depthStencilInfo.minDepthBounds = 0.0f; // Optional
        configInfo.depthStencilInfo.maxDepthBounds = 1.0f; // Optional
        configInfo.depthStencilInfo.stencilTestEnable = VK_FALSE;
        configInfo.depthStencilInfo.front = {}; // Optional
        configInfo.depthStencilInfo.back = {}; // Optional

        configInfo.colorBlendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;
        configInfo.colorBlendAttachment.blendEnable = VK_FALSE;
        configInfo.colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_ONE; // Optional
        configInfo.colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ZERO; // Optional
        configInfo.colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD; // Optional
        configInfo.colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE; // Optional
        configInfo.colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO; // Optional
        configInfo.colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD; // Optional

        configInfo.colorBlending.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
        configInfo.colorBlending.logicOpEnable = VK_FALSE;
        configInfo.colorBlending.logicOp = VK_LOGIC_OP_COPY; // Optional
        configInfo.colorBlending.attachmentCount = 1;
        configInfo.colorBlending.pAttachments = &configInfo.colorBlendAttachment;
        configInfo.colorBlending.blendConstants[0] = 0.0f; // Optional
        configInfo.colorBlending.blendConstants[1] = 0.0f; // Optional
        configInfo.colorBlending.blendConstants[2] = 0.0f; // Optional
        configInfo.colorBlending.blendConstants[3] = 0.0f; // Optional

        configInfo.dynamicStateEnables.push_back(VK_DYNAMIC_STATE_VIEWPORT);
        configInfo.dynamicStateEnables.push_back(VK_DYNAMIC_STATE_SCISSOR);

        configInfo.dynamicState.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
        configInfo.dynamicState.dynamicStateCount = static_cast<uint32_t>(configInfo.dynamicStateEnables.size());
        configInfo.dynamicState.pDynamicStates = configInfo.dynamicStateEnables.data();

        configInfo.bindingDescription.push_back(vkb::Model::Vertex::getBindingDescription());
        auto attributeDescription = vkb::Model::Vertex::getAttributeDescriptions();
        configInfo.attributeDescription.insert(configInfo.attributeDescription.end(), attributeDescription.begin(), attributeDescription.end());

        return configInfo;
    }

    Pipeline::PipelineConfigInfo Pipeline::defaultConfigInfo(VkPipelineLayout layout, VkRenderPass renderPass) {
        PipelineConfigInfo configInfo = defaultConfigInfo();
        configInfo.setPipelineLayout(layout);
        configInfo.setRenderPass(renderPass);
        return configInfo;
    }

    void Pipeline::PipelineConfigInfo::enableAlphaBlending() {
        colorBlendAttachment.blendEnable = VK_TRUE;
        colorBlendAttachment.colorWriteMask =
                VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT |
                VK_COLOR_COMPONENT_A_BIT;
        colorBlendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
        colorBlendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
        colorBlendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
        colorBlendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
        colorBlendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
        colorBlendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;
    }
}