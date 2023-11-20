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

    void render(vkb::RenderSystem& renderSystem, VkCommandBuffer commandBuffer) const {m_object->render(renderSystem, commandBuffer);};

    [[nodiscard]] const std::vector<glm::vec3>& getPositions() const { return m_particlePositions; }
    [[nodiscard]] size_t numParticles() const { return m_particlePositions.size(); }
    [[nodiscard]] glm::vec3 getTranslation() const { return m_object->translation; }

    void translate(const glm::vec3& move);

private:
    std::unique_ptr<vkb::DrawableObject> m_object;
    std::vector<glm::vec3> m_particlePositions;
    const vkb::Device& m_deviceRef;
};


#endif //VULKANFLUIDSIMULATION_RIGIDOBJECT_H
