#define GLFW_INCLUDE_VULKAN
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_ENABLE_EXPERIMENTAL

#include "Grid3DSim.h"

const uint32_t WIDTH = 1000;
const uint32_t HEIGHT = 1000;
const std::string APP_NAME = "Vulkan 3D Grid Fluid Simulation";

int main() {
    Grid3DSim app{WIDTH, HEIGHT, APP_NAME, vkb::Device::NVIDIA};

    try {
        app.run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
