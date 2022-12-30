//
// Created by luc on 13/11/22.
//

#ifndef VULKANBASE_CAMERAMOVEMENTCONTROLLER_H
#define VULKANBASE_CAMERAMOVEMENTCONTROLLER_H

#include "Window.h"
#include "Camera.h"

namespace vkb {
    class CameraMovementController {
    public:
        struct KeyMappings {
            int moveLeft = GLFW_KEY_A;
            int moveRight = GLFW_KEY_D;
            int moveForward = GLFW_KEY_W;
            int moveBackward = GLFW_KEY_S;
            int moveUp = GLFW_KEY_E;
            int moveDown = GLFW_KEY_Q;
            int lookLeft = GLFW_KEY_LEFT;
            int lookRight = GLFW_KEY_RIGHT;
            int lookUp = GLFW_KEY_UP;
            int lookDown = GLFW_KEY_DOWN;
        };

        void moveCamera(GLFWwindow* window, float dt, Camera& camera) const;

        KeyMappings keys{};
        float moveSpeed{30.f};
        float lookSpeed{1.5f};

    };
}


#endif //VULKANBASE_CAMERAMOVEMENTCONTROLLER_H
