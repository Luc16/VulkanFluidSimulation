//
// Created by luc on 13/11/22.
//

#ifndef VULKANBASE_CAMERA_H
#define VULKANBASE_CAMERA_H

#include "utils.h"

// TODO revisar a matematica
namespace vkb {
    class Camera {
    public:
        void setOrthographicProjection(float left, float right, float top, float bottom, float near, float far);
        void setPerspectiveProjection(float fovy, float aspect, float near, float far);

        void setViewDirection(glm::vec3 position, glm::vec3 direction, glm::vec3 up = glm::vec3{0.f, -1.f, 0.f});

        void setViewTarget(glm::vec3 position, glm::vec3 target, glm::vec3 up = glm::vec3{0.f, -1.f, 0.f});

        void updateView();

        void setViewPos(glm::vec3 pos) { m_translation = pos; }
        [[nodiscard]] const glm::mat4& getProjection() const { return m_proj; }
        [[nodiscard]] const glm::mat4& getView() const { return m_view; }
//        const glm::mat4& getInverseView() const { return inverseViewMatrix; }
//        const glm::vec3 getPosition() const { return glm::vec3(inverseViewMatrix[3]); }

        glm::vec3 m_translation{}, m_rotation{};
    private:
        glm::mat4 m_view{1.0f}, m_proj{1.0f};

        friend class CameraMovementController;
    };
}



#endif //VULKANBASE_CAMERA_H
