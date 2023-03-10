cmake_minimum_required(VERSION 3.24)
project(VulkanFluidSimulation)
set(CMAKE_CXX_STANDARD 23)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++23")

find_package(Vulkan REQUIRED)
find_package(glfw3 REQUIRED )

file(GLOB imgui CONFIGURE_DEPENDS "external/imgui/*.h" "external/imgui/*.cpp")
add_library( VkFluidSim
        ${imgui}
        external/stbimage/stb_image.h
        external/objloader/tiny_obj_loader.h
        src/lib/SwapChain.cpp src/lib/SwapChain.h
        src/lib/utils.cpp src/lib/utils.h
        src/lib/Buffer.cpp src/lib/Buffer.h
        src/lib/Device.cpp src/lib/Device.h
        src/lib/Window.cpp src/lib/Window.h
        src/lib/Image.cpp src/lib/Image.h
        src/lib/Model.cpp src/lib/Model.h
        src/lib/Texture.cpp src/lib/Texture.h
        src/lib/GraphicsPipeline.cpp src/lib/GraphicsPipeline.h
        src/lib/Renderer.cpp src/lib/Renderer.h
        src/lib/descriptors/DescriptorSetLayout.cpp src/lib/descriptors/DescriptorSetLayout.h
        src/lib/descriptors/DescriptorPool.cpp src/lib/descriptors/DescriptorPool.h
        src/lib/descriptors/DescriptorWriter.cpp src/lib/descriptors/DescriptorWriter.h
        src/lib/CameraMovementController.cpp src/lib/CameraMovementController.h
        src/lib/Camera.cpp src/lib/Camera.h
        src/lib/DrawableObject.cpp src/lib/DrawableObject.h
        src/lib/RenderSystem.cpp src/lib/RenderSystem.h
        src/lib/VulkanApp.cpp src/lib/VulkanApp.h
        src/lib/InstancedObjects.cpp src/lib/InstancedObjects.h
        src/Instancing/InstancingApp.cpp src/Instancing/InstancingApp.h
        src/Grid2DSim/Grid2DSim.cpp src/Grid2DSim/Grid2DSim.h
        src/Grid2DSim/Grid2D.cpp src/Grid2DSim/Grid2D.h
        src/Grid3DSim/Grid3DSim.cpp src/Grid3DSim/Grid3DSim.h
        src/ComputeShaderTest/ComputeShaderTest.cpp src/ComputeShaderTest/ComputeShaderTest.h
        src/lib/ComputeSystem.cpp src/lib/ComputeSystem.h src/lib/ComputeShaderHandler.cpp src/lib/ComputeShaderHandler.h
        src/SPHCPU2DSim/SPHCPU2DSim.cpp src/SPHCPU2DSim/SPHCPU2DSim.h
        src/SPHGPU2DSim/SPHGPU2DSim.cpp src/SPHGPU2DSim/SPHGPU2DSim.h)

set( APPS
        "Instancing"
        "Grid2DSim"
        "Grid3DSim"
        "ComputeShaderTest"
        "SPHCPU2DSim"
        "SPHGPU2DSim"
        )

foreach( testsourcefolder ${APPS} )
    add_executable( ${testsourcefolder} src/${testsourcefolder}/main.cpp)
    target_link_libraries( ${testsourcefolder} VkFluidSim GL glfw Vulkan::Vulkan)

    # Make sure YourLib is linked to each app
endforeach( testsourcefolder ${APPS} )

