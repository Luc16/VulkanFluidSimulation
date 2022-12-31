//
// Created by luc on 26/10/22.
//

#ifndef VULKANBASE_WINDOW_H
#define VULKANBASE_WINDOW_H

#include <GLFW/glfw3.h>
#include <vulkan/vulkan.h>
#include <string>

namespace vkb {
    class Window {
    public:
        Window(int width, int height, const std::string& title);
        Window(const Window &) = delete;
        Window &operator=(const Window &) = delete;
        Window(Window &&) = delete;
        Window &operator=(Window &&) = delete;

        ~Window();

        [[nodiscard]] GLFWwindow* window() const { return m_window; }
        [[nodiscard]] bool shouldClose() const { return glfwWindowShouldClose(m_window); }
        [[nodiscard]] bool frameBufferResized() const { return m_frameBufferResized; }
        void frameBufferNotResized() { m_frameBufferResized = false; }
        [[nodiscard]] VkExtent2D extent() const {return {static_cast<uint32_t>(m_width), static_cast<uint32_t>(m_height)};}
        [[nodiscard]] int width() const {return m_width;}
        [[nodiscard]] int height() const {return m_height;}

    private:
        static void framebufferResizeCallback(GLFWwindow* window, int width, int height);

        bool m_frameBufferResized = false;
        int m_width;
        int m_height;

        GLFWwindow* m_window{};
    };
}



#endif //VULKANBASE_WINDOW_H
