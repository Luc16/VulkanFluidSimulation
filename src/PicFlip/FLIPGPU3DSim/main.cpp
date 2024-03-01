#define GLFW_INCLUDE_VULKAN
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_ENABLE_EXPERIMENTAL

#include "FLIPGPU3DSim.h"

const std::string APP_NAME = "Vulkan GPU PIC/FLIP 3D fluid simulation";

int main() {
    FLIPGPU3DSim app{APP_NAME};

    app.compileShaders();
    try {
        app.run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
