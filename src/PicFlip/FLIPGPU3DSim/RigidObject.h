//
// Created by luc on 20/11/23.
//

#ifndef VULKANFLUIDSIMULATION_RIGIDOBJECT_H
#define VULKANFLUIDSIMULATION_RIGIDOBJECT_H


#include "../../lib/DrawableObject.h"
#include "../../lib/utils.h"

class RigidObject {
public:

    RigidObject(const vkb::Device& device, const std::string& modelFile, const std::shared_ptr<vkb::Texture>& tex, float scale, float particleRadius);
    RigidObject(RigidObject&& other) noexcept : m_particlePositions(std::move(other.m_particlePositions)), m_object(std::move(other.m_object)){};
    RigidObject& operator=(RigidObject&& other) noexcept {
        m_particlePositions = std::move(other.m_particlePositions);
        m_object = std::move(other.m_object);
        return *this;
    }


    void render(vkb::RenderSystem& renderSystem, VkCommandBuffer commandBuffer) const {m_object->render(renderSystem, commandBuffer);};

    [[nodiscard]] glm::vec3 getTranslation() const { return m_object->translation; }

    void translate(const glm::vec3& move);

private:
    std::unique_ptr<vkb::DrawableObject> m_object;
};


#endif //VULKANFLUIDSIMULATION_RIGIDOBJECT_H
