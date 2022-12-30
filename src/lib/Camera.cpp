//
// Created by luc on 13/11/22.
//

#include "Camera.h"

#include <cmath>

namespace vkb {

    void Camera::setOrthographicProjection(float left, float right, float top, float bottom, float near, float far) {
        m_proj = glm::mat4{1.0f};
        m_proj[0][0] = 2.f / (right - left);
        m_proj[1][1] = 2.f / (bottom - top);
        m_proj[2][2] = 1.f / (far - near);
        m_proj[3][0] = -(right + left) / (right - left);
        m_proj[3][1] = -(bottom + top) / (bottom - top);
        m_proj[3][2] = -near / (far - near);
    }

    void Camera::setPerspectiveProjection(float fovy, float aspect, float near, float far) {
        const float tanHalfFovy = std::tan(fovy / 2.f);
        m_proj = glm::mat4{0.0f};
        m_proj[0][0] = 1.f / (aspect * tanHalfFovy);
        m_proj[1][1] = 1.f / (tanHalfFovy);
        m_proj[2][2] = far / (far - near);
        m_proj[2][3] = 1.f;
        m_proj[3][2] = -(far * near) / (far - near);
    }

    void Camera::setViewDirection(glm::vec3 position, glm::vec3 direction, glm::vec3 up) {
        m_translation = position;
        const glm::vec3 w{glm::normalize(direction)};
        const glm::vec3 u{glm::normalize(glm::cross(w, up))};
        const glm::vec3 v{glm::cross(w, u)};

        m_view = glm::mat4{1.f};
        m_view[0][0] = u.x;
        m_view[1][0] = u.y;
        m_view[2][0] = u.z;
        m_view[0][1] = v.x;
        m_view[1][1] = v.y;
        m_view[2][1] = v.z;
        m_view[0][2] = w.x;
        m_view[1][2] = w.y;
        m_view[2][2] = w.z;
        m_view[3][0] = -glm::dot(u, position);
        m_view[3][1] = -glm::dot(v, position);
        m_view[3][2] = -glm::dot(w, position);

//        inverseViewMatrix = glm::mat4{1.f};
//        inverseViewMatrix[0][0] = u.x;
//        inverseViewMatrix[0][1] = u.y;
//        inverseViewMatrix[0][2] = u.z;
//        inverseViewMatrix[1][0] = v.x;
//        inverseViewMatrix[1][1] = v.y;
//        inverseViewMatrix[1][2] = v.z;
//        inverseViewMatrix[2][0] = w.x;
//        inverseViewMatrix[2][1] = w.y;
//        inverseViewMatrix[2][2] = w.z;
//        inverseViewMatrix[3][0] = position.x;
//        inverseViewMatrix[3][1] = position.y;
//        inverseViewMatrix[3][2] = position.z;
    }

    void Camera::setViewTarget(glm::vec3 position, glm::vec3 target, glm::vec3 up) {
        setViewDirection(position, target - position, up);
    }

    void Camera::updateView() {
        const float c3 = glm::cos(m_rotation.z);
        const float s3 = glm::sin(m_rotation.z);
        const float c2 = glm::cos(m_rotation.x);
        const float s2 = glm::sin(m_rotation.x);
        const float c1 = glm::cos(m_rotation.y);
        const float s1 = glm::sin(m_rotation.y);
        const glm::vec3 u{(c1 * c3 + s1 * s2 * s3), (c2 * s3), (c1 * s2 * s3 - c3 * s1)};
        const glm::vec3 v{(c3 * s1 * s2 - c1 * s3), (c2 * c3), (c1 * c3 * s2 + s1 * s3)};
        const glm::vec3 w{(c2 * s1), (-s2), (c1 * c2)};
        m_view = glm::mat4{1.f};
        m_view[0][0] = u.x;
        m_view[1][0] = u.y;
        m_view[2][0] = u.z;
        m_view[0][1] = v.x;
        m_view[1][1] = v.y;
        m_view[2][1] = v.z;
        m_view[0][2] = w.x;
        m_view[1][2] = w.y;
        m_view[2][2] = w.z;
        m_view[3][0] = -glm::dot(u, m_translation);
        m_view[3][1] = -glm::dot(v, m_translation);
        m_view[3][2] = -glm::dot(w, m_translation);

//        inverseViewMatrix = glm::mat4{1.f};
//        inverseViewMatrix[0][0] = u.x;
//        inverseViewMatrix[0][1] = u.y;
//        inverseViewMatrix[0][2] = u.z;
//        inverseViewMatrix[1][0] = v.x;
//        inverseViewMatrix[1][1] = v.y;
//        inverseViewMatrix[1][2] = v.z;
//        inverseViewMatrix[2][0] = w.x;
//        inverseViewMatrix[2][1] = w.y;
//        inverseViewMatrix[2][2] = w.z;
//        inverseViewMatrix[3][0] = position.x;
//        inverseViewMatrix[3][1] = position.y;
//        inverseViewMatrix[3][2] = position.z;
    }
}