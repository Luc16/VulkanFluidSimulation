#define GLFW_INCLUDE_VULKAN
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_ENABLE_EXPERIMENTAL

#include "ComputeShaderTest.h"

const uint32_t WIDTH = 1000;
const uint32_t HEIGHT = 1000;
const std::string APP_NAME = "Vulkan";

int main() {
    ComputeShaderTest app{WIDTH, HEIGHT, APP_NAME, vkb::Device::NVIDIA};

    system("glslc ../src/ComputeShaderTest/Shaders/default.vert -o ../src/ComputeShaderTest/Shaders/default.vert.spv");
    system("glslc ../src/ComputeShaderTest/Shaders/default.frag -o ../src/ComputeShaderTest/Shaders/default.frag.spv");
    system("glslc ../src/ComputeShaderTest/Shaders/move_particles.comp -o ../src/ComputeShaderTest/Shaders/move_particles.comp.spv");
    system("glslc ../src/ComputeShaderTest/Shaders/calculate_forces.comp -o ../src/ComputeShaderTest/Shaders/calculate_forces.comp.spv");
    try {
        app.run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
