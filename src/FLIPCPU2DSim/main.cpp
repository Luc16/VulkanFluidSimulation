#define GLFW_INCLUDE_VULKAN
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_ENABLE_EXPERIMENTAL

#include "FLIPCPU2DSim.h"

const std::string APP_NAME = "Vulkan CPU PIC/FLIP 2D fluid simulation";

int main() {
    FLIPCPU2DSim app{APP_NAME, vkb::Device::INTEL};

    try {
        app.run(nullptr, 0, <#initializer#>);
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
