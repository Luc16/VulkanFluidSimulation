#define GLFW_INCLUDE_VULKAN
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_ENABLE_EXPERIMENTAL

#include "SPHGPU3DSim.h"

const uint32_t WIDTH = 1000;
const uint32_t HEIGHT = 1000;
const std::string APP_NAME = "Vulkan GPU SPH 2D fluid simulation";

int main() {
    system("glslc ../src/SPH/SPHGPU3DSim/Shaders/default.vert -o ../src/SPH/SPHGPU3DSim/Shaders/default.vert.spv");
    system("glslc ../src/SPH/SPHGPU3DSim/Shaders/default.frag -o ../src/SPH/SPHGPU3DSim/Shaders/default.frag.spv");
    system("glslc ../src/SPH/SPHGPU3DSim/Shaders/instancing.vert -o ../src/SPH/SPHGPU3DSim/Shaders/instancing.vert.spv");
    system("glslc ../src/SPH/SPHGPU3DSim/Shaders/instancing.frag -o ../src/SPH/SPHGPU3DSim/Shaders/instancing.frag.spv");
    system("glslc ../src/SPH/SPHGPU3DSim/Shaders/integrate.comp -o ../src/SPH/SPHGPU3DSim/Shaders/integrate.comp.spv");
    system("glslc ../src/SPH/SPHGPU3DSim/Shaders/calculate_forces.comp -o ../src/SPH/SPHGPU3DSim/Shaders/calculate_forces.comp.spv");
    system("glslc ../src/SPH/SPHGPU3DSim/Shaders/calculate_density_pressure.comp -o ../src/SPH/SPHGPU3DSim/Shaders/calculate_density_pressure.comp.spv");


    SPHGPU3DSim app{WIDTH, HEIGHT, APP_NAME, vkb::Device::INTEL};

    try {
        app.run();
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
