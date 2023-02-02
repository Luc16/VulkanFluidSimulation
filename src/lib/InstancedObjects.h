//
// Created by luc on 12/12/22.
//

#ifndef VULKANBASE_INSTANCEDOBJECTS_H
#define VULKANBASE_INSTANCEDOBJECTS_H

#include "DrawableObject.h"

namespace vkb {
    template<typename InstanceData>
    class InstancedObjects : public DrawableObject, private std::vector<InstanceData> {
    public:
        InstancedObjects(const Device &device, size_t initialSize, std::shared_ptr<vkb::Model> model,
                         std::shared_ptr<vkb::Texture> texture = nullptr);

        void render(vkb::RenderSystem &renderSystem, VkCommandBuffer commandBuffer) override;
        void updateBuffer();
        void resizeBuffer(size_t new_size);

        VkVertexInputBindingDescription getBindingDescription();

        using std::vector<InstanceData>::size;
        using std::vector<InstanceData>::operator[];
        using std::vector<InstanceData>::resize;
        using std::vector<InstanceData>::begin;
        using std::vector<InstanceData>::end;
        using std::vector<InstanceData>::swap;

    private:
        void createInstanceBuffer();

        std::unique_ptr<vkb::Buffer> m_instanceBuffer;
        const Device& m_deviceRef;
    };

    template<typename InstanceData>
    void InstancedObjects<InstanceData>::resizeBuffer(size_t new_size) {
        this->resize(new_size);
        createInstanceBuffer();
    }


    template<typename InstanceData>
    InstancedObjects<InstanceData>::InstancedObjects(const Device &device, size_t initialSize,
                                                     std::shared_ptr<vkb::Model> model,
                                                     std::shared_ptr<vkb::Texture> texture):
            m_deviceRef(device), DrawableObject(model, texture), std::vector<InstanceData>(initialSize) {
                if (initialSize > 0) createInstanceBuffer();
    }

    template<typename InstanceData>
    void InstancedObjects<InstanceData>::updateBuffer() {

        vkb::Buffer stagingBuffer(m_deviceRef, m_instanceBuffer->getSize(), VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                  VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        stagingBuffer.singleWrite(this->data());

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

        m_model->draw(commandBuffer, this->size());
    }

    template<typename InstanceData>
    void InstancedObjects<InstanceData>::createInstanceBuffer() {
        VkDeviceSize bufferSize = sizeof(InstanceData) * this->size();

        vkb::Buffer stagingBuffer(m_deviceRef, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                  VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        stagingBuffer.singleWrite(this->data());

        m_instanceBuffer = std::make_unique<vkb::Buffer>(m_deviceRef, bufferSize,
                                                         VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                         VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
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


}
#endif //VULKANBASE_INSTANCEDOBJECTS_H
