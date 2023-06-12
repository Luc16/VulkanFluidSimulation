//
// Created by luc on 12/12/22.
//

#ifndef VULKANBASE_INSTANCEDOBJECTS_H
#define VULKANBASE_INSTANCEDOBJECTS_H

#include "DrawableObject.h"

namespace vkb {
    template<typename InstanceData>
    class InstancedObjects : public DrawableObject {
    public:
        InstancedObjects(const Device &device, size_t initialSize, std::shared_ptr<vkb::Model> model,
                         std::shared_ptr<vkb::Texture> texture = nullptr, VkBufferUsageFlags customUsage = 0);

        void render(vkb::RenderSystem &renderSystem, VkCommandBuffer commandBuffer) override;
        void updateBuffer();
        void resizeBuffer(size_t new_size);

        VkVertexInputBindingDescription getBindingDescription();
        [[nodiscard]] VkDescriptorBufferInfo descriptorInfo(VkDeviceSize size = VK_WHOLE_SIZE, VkDeviceSize offset = 0) const;
        [[nodiscard]] const std::unique_ptr<vkb::Buffer>& getBuffer() { return m_instanceBuffer; };
        [[nodiscard]] std::vector<InstanceData> getVector() { return m_instanceVector; };
        [[nodiscard]] size_t size() { return m_instanceVector.size(); };
        [[nodiscard]] constexpr const InstanceData& operator[](uint32_t i) const{ return m_instanceVector[i];}
        [[nodiscard]] constexpr InstanceData& operator[](uint32_t i) { return m_instanceVector[i];}
        [[nodiscard]] constexpr auto begin() { return m_instanceVector.begin();}
        [[nodiscard]] constexpr auto end() { return m_instanceVector.end();}
        void swapVector(std::vector<InstanceData>& other) { m_instanceVector.swap(other); }

    private:
        void createInstanceBuffer();

        std::unique_ptr<vkb::Buffer> m_instanceBuffer;
        std::vector<InstanceData> m_instanceVector;
        const Device& m_deviceRef;
        VkBufferUsageFlags m_customUsage = 0;
    };

    template<typename InstanceData>
    void InstancedObjects<InstanceData>::resizeBuffer(size_t new_size) {
        m_instanceVector.resize(new_size);
        createInstanceBuffer();
    }


    template<typename InstanceData>
    InstancedObjects<InstanceData>::InstancedObjects(const Device &device, size_t initialSize,
                                                     std::shared_ptr<vkb::Model> model,
                                                     std::shared_ptr<vkb::Texture> texture,
                                                     VkBufferUsageFlags customUsage):
                                                 m_deviceRef(device),
                                                 DrawableObject(model, texture),
                                                 m_instanceVector(initialSize),
                                                 m_customUsage(customUsage){
                if (initialSize > 0) createInstanceBuffer();
    }

    template<typename InstanceData>
    void InstancedObjects<InstanceData>::updateBuffer() {

        vkb::Buffer stagingBuffer(m_deviceRef, m_instanceBuffer->getSize(), VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                  VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        stagingBuffer.singleWrite(m_instanceVector.data());

        m_deviceRef.copyBuffer(stagingBuffer.getBuffer(), m_instanceBuffer->getBuffer(), m_instanceBuffer->getSize());
    }

    template<typename InstanceData>
    void InstancedObjects<InstanceData>::render(vkb::RenderSystem &renderSystem, VkCommandBuffer commandBuffer) {
        if (renderSystem.pushConstantSize() > 0) {
            PushConstantData push{};
            push.modelMatrix = modelMatrix();
            vkCmdPushConstants(
                    commandBuffer,
                    renderSystem.pipelineLayout(),
                    VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                    0,
                    sizeof(PushConstantData),
                    &push);
        }
        m_model->bind(commandBuffer);

        VkBuffer instanceBuffer = {m_instanceBuffer->getBuffer()};
        VkDeviceSize offsets[] = {0};

        vkCmdBindVertexBuffers(commandBuffer, 1, 1, &instanceBuffer, offsets);

        m_model->draw(commandBuffer, m_instanceVector.size());
    }

    template<typename InstanceData>
    void InstancedObjects<InstanceData>::createInstanceBuffer() {
        VkDeviceSize bufferSize = sizeof(InstanceData) * m_instanceVector.size();

        vkb::Buffer stagingBuffer(m_deviceRef, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                  VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        stagingBuffer.singleWrite(m_instanceVector.data());

        m_instanceBuffer = std::make_unique<vkb::Buffer>(m_deviceRef, bufferSize,
                                                         VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                         VK_BUFFER_USAGE_VERTEX_BUFFER_BIT |
                                                         m_customUsage,
                                                         VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        m_deviceRef.copyBuffer(stagingBuffer.getBuffer(), m_instanceBuffer->getBuffer(), bufferSize);
    }

    template<typename InstanceData>
    VkVertexInputBindingDescription InstancedObjects<InstanceData>::getBindingDescription() {
        VkVertexInputBindingDescription bindingDescription{};
        bindingDescription.binding = 1;
        bindingDescription.stride = sizeof(InstanceData);
        bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_INSTANCE;

        return bindingDescription;
    }

    template<typename InstanceData>
    VkDescriptorBufferInfo InstancedObjects<InstanceData>::descriptorInfo(VkDeviceSize size, VkDeviceSize offset) const {
        return m_instanceBuffer->descriptorInfo(size, offset);
    }



}
#endif //VULKANBASE_INSTANCEDOBJECTS_H
