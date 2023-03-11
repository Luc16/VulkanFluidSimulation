#define GLFW_INCLUDE_VULKAN
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_ENABLE_EXPERIMENTAL

#include "Grid2DSim.h"

const std::string APP_NAME = "Vulkan 2D Grid Fluid Simulation";

int main() {
    Grid2DSim app{Grid2DSim::WIDTH, Grid2DSim::HEIGHT, APP_NAME, vkb::Device::NVIDIA};

    try {
        app.run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
