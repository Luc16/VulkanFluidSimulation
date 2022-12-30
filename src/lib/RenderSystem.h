//
// Created by luc on 13/11/22.
//

#ifndef VULKANBASE_RENDERSYSTEM_H
#define VULKANBASE_RENDERSYSTEM_H

#include <utility>

#include "utils.h"
#include "Device.h"
#include "Pipeline.h"

namespace vkb {
    class RenderSystem {
    public:
        struct ShaderPaths {
            ShaderPaths(std::string  vertShaderPath, std::string  fragShaderPath): vertPath(std::move(vertShaderPath)), fragPath(std::move(fragShaderPath)){}
            const std::string fragPath;
            const std::string vertPath;
        };

        explicit RenderSystem(const Device &device);
        ~RenderSystem();
        RenderSystem(const RenderSystem &) = delete;
        RenderSystem &operator=(const RenderSystem &) = delete;

        [[nodiscard]] VkPipelineLayout pipelineLayout() { return m_pipelineLayout; }
        [[nodiscard]] uint32_t pushConstantSize() const { return m_pushConstantSize; }
        void bind(VkCommandBuffer commandBuffer, VkDescriptorSet* descriptorSet);

        void createPipelineLayout(VkDescriptorSetLayout globalSetLayout, uint32_t pushConstantSize = 0);
        void createPipeline(VkRenderPass renderPass, const ShaderPaths& shaderPaths, const std::function<void(Pipeline::PipelineConfigInfo&)>& configurePipeline = {});
    private:

        const Device& m_deviceRef;

        std::unique_ptr<Pipeline> m_pipeline;
        VkPipelineLayout m_pipelineLayout{};

        bool m_layoutCreated = false;
        uint32_t m_pushConstantSize;
    };
}



#endif //VULKANBASE_RENDERSYSTEM_H
