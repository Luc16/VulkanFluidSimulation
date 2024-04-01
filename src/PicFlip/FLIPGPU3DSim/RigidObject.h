//
// Created by luc on 20/11/23.
//

#ifndef VULKANFLUIDSIMULATION_RIGIDOBJECT_H
#define VULKANFLUIDSIMULATION_RIGIDOBJECT_H


#include "../../lib/DrawableObject.h"
#include "../../lib/utils.h"

class RigidObject {
public:

    RigidObject(const vkb::Device& device, const std::string& modelFile, const std::shared_ptr<vkb::Texture>& tex,
                             float scale);
    RigidObject(RigidObject&& other) noexcept: m_object(std::move(other.m_object)), m_deviceRef(other.m_deviceRef){};
    RigidObject& operator=(RigidObject&& other) noexcept {
        m_object = std::move(other.m_object);
        return *this;
    }


    void createSdf(float cellSize, const glm::vec3& gridDimensions);
    void render(vkb::RenderSystem& renderSystem, VkCommandBuffer commandBuffer) const {m_object->render(renderSystem, commandBuffer);};
    const std::unique_ptr<vkb::Buffer>& getSDF() { return m_sdf; }

    [[nodiscard]] glm::vec3 getTranslation() const { return m_object->translation; }

    void translate(const glm::vec3& move);

private:
    const vkb::Device& m_deviceRef;
    std::unique_ptr<vkb::DrawableObject> m_object;
    std::unique_ptr<vkb::Buffer> m_sdf;
    std::vector<glm::vec3> m_vertices;
    std::vector<uint32_t> m_indices;

};


#endif //VULKANFLUIDSIMULATION_RIGIDOBJECT_H
