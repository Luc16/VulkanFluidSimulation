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
    RigidObject(RigidObject&& other) noexcept:
        m_object(std::move(other.m_object)),
        m_sdf(std::move(other.m_sdf)),
        m_sdfPos(other.m_sdfPos),
        m_scale(other.m_scale),
        m_vertices(other.m_vertices),
        m_indices(other.m_indices){};
    RigidObject& operator=(RigidObject&& other) noexcept {
        m_object = std::move(other.m_object);
        m_sdf = std::move(other.m_sdf);
        m_sdfPos = other.m_sdfPos;
        m_scale = other.m_scale;
        m_vertices = other.m_vertices;
        m_indices = other.m_indices;
        return *this;
    }


    void createSdf(const vkb::Device& device, float cellSize, const glm::vec3& gridDimensions);
    void render(vkb::RenderSystem& renderSystem, VkCommandBuffer commandBuffer) const {m_object->render(renderSystem, commandBuffer);};
    const std::unique_ptr<vkb::Buffer>& getSdf() { return m_sdf; }
    [[nodiscard]] VkDescriptorBufferInfo getSdfInfo() const { return m_sdf->descriptorInfo(); }

    [[nodiscard]] glm::vec3 getTranslation() const { return m_object->translation; }
    [[nodiscard]] bool sdfCreated() const { return m_sdfPos == m_object->translation; }
    [[nodiscard]] float getScale() const { return m_scale; }
    [[nodiscard]] std::string getModelPath() const { return m_name; }

    void translate(const glm::vec3& move);

private:
    std::unique_ptr<vkb::DrawableObject> m_object;
    std::unique_ptr<vkb::Buffer> m_sdf;
    glm::vec3 m_sdfPos{};
    std::string m_name;
    float m_scale;
    std::vector<glm::vec3> m_vertices;
    std::vector<uint32_t> m_indices;

};


#endif //VULKANFLUIDSIMULATION_RIGIDOBJECT_H
