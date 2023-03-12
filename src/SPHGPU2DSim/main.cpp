#define GLFW_INCLUDE_VULKAN
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_ENABLE_EXPERIMENTAL

#include "SPHGPU2DSim.h"

const uint32_t WIDTH = 1000;
const uint32_t HEIGHT = 1000;
const std::string APP_NAME = "Vulkan GPU SPH 2D fluid simulation";

int main() {
    system("glslc ../src/SPHGPU2DSim/Shaders/default.vert -o ../src/SPHGPU2DSim/Shaders/default.vert.spv");
    system("glslc ../src/SPHGPU2DSim/Shaders/default.frag -o ../src/SPHGPU2DSim/Shaders/default.frag.spv");
    system("glslc ../src/SPHGPU2DSim/Shaders/integrate.comp -o ../src/SPHGPU2DSim/Shaders/integrate.comp.spv");
    system("glslc ../src/SPHGPU2DSim/Shaders/calculate_forces.comp -o ../src/SPHGPU2DSim/Shaders/calculate_forces.comp.spv");
    system("glslc ../src/SPHGPU2DSim/Shaders/calculate_density_pressure.comp -o ../src/SPHGPU2DSim/Shaders/calculate_density_pressure.comp.spv");


    SPHGPU2DSim app{WIDTH, HEIGHT, APP_NAME, vkb::Device::NVIDIA};

    try {
        app.run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
