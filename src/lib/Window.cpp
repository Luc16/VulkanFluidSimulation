//
// Created by luc on 26/10/22.
//

#include <string>
#include "Window.h"

namespace vkb {
    Window::Window(int width, int height, const std::string& title, bool resizable): m_width(width), m_height(height) {
        glfwInit();

        glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
        if (!resizable) glfwWindowHint(GLFW_RESIZABLE, GLFW_FALSE);

        m_window = glfwCreateWindow(m_width, m_height, title.c_str(), nullptr, nullptr);
        glfwSetWindowUserPointer(m_window, this);
        glfwSetFramebufferSizeCallback(m_window, framebufferResizeCallback);
    }

    Window::~Window() {
        glfwDestroyWindow(m_window);
        glfwTerminate();
    }

    void Window::framebufferResizeCallback(GLFWwindow* window, int width, int height) {
        auto app = reinterpret_cast<Window *>(glfwGetWindowUserPointer(window));
        app->m_frameBufferResized = true;
        app->m_width = width;
        app->m_height = height;
    }
}