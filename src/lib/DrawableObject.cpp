//
// Created by luc on 13/11/22.
//

#include "DrawableObject.h"

namespace vkb {

    void DrawableObject::render(vkb::RenderSystem& renderSystem, VkCommandBuffer commandBuffer) {
        if (renderSystem.pushConstantSize() > 0){
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
        m_model->draw(commandBuffer);
    }

    glm::mat4 DrawableObject::modelMatrix() const {
        return glm::translate(glm::mat4(1.0f), m_translation)*glm::scale(glm::mat4(1.0f), m_scale)
               * glm::rotate(glm::mat4(1.0f), m_rotation.z,glm::vec3(0.0f, 0.0f, 1.0f))
               * glm::rotate(glm::mat4(1.0f), m_rotation.y,glm::vec3(0.0f, 1.0f, 0.0f))
               * glm::rotate(glm::mat4(1.0f), m_rotation.x,glm::vec3(1.0f, 0.0f, 0.0f));
    }
}