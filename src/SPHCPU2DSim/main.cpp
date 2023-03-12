#define GLFW_INCLUDE_VULKAN
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_ENABLE_EXPERIMENTAL

#include "SPHCPU2DSim.h"

const uint32_t WIDTH = 1000;
const uint32_t HEIGHT = 1000;
const std::string APP_NAME = "Vulkan CPU SPH 2D fluid simulation";

int main() {
    SPHCPU2DSim app{WIDTH, HEIGHT, APP_NAME, vkb::Device::NVIDIA};

    system("glslc ../src/SPHCPU2DSim/Shaders/default.vert -o ../src/SPHCPU2DSim/Shaders/default.vert.spv");
    system("glslc ../src/SPHCPU2DSim/Shaders/default.frag -o ../src/SPHCPU2DSim/Shaders/default.frag.spv");
    try {
        app.run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
