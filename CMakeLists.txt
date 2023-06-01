cmake_minimum_required(VERSION 3.24)
project(VulkanFluidSimulation)
set(CMAKE_CXX_STANDARD 23)

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
        src/grid/Grid2DSim/Grid2DSim.cpp src/grid/Grid2DSim/Grid2DSim.h
        src/grid/Grid2DSim/Grid2D.cpp src/grid/Grid2DSim/Grid2D.h
        src/grid/Grid3DSim/Grid3DSim.cpp src/grid/Grid3DSim/Grid3DSim.h
        src/ComputeShaderTest/ComputeShaderTest.cpp src/ComputeShaderTest/ComputeShaderTest.h
        src/lib/ComputeSystem.cpp src/lib/ComputeSystem.h src/lib/ComputeShaderHandler.cpp src/lib/ComputeShaderHandler.h
        src/SPH/SPHCPU2DSim/SPHCPU2DSim.cpp src/SPH/SPHCPU2DSim/SPHCPU2DSim.h
        src/SPH/SPHGPU2DSim/SPHGPU2DSim.cpp src/SPH/SPHGPU2DSim/SPHGPU2DSim.h
        src/SPH/SPHCPU3DSim/SPHCPU3DSim.cpp src/SPH/SPHCPU3DSim/SPHCPU3DSim.h
        src/FLIPCPU2DSim/FLIPCPU2DSim.cpp src/FLIPCPU2DSim/FLIPCPU2DSim.h
        src/lib/graphicsDataStructures/Matrices.h src/lib/graphicsDataStructures/SpatialHash.h src/lib/graphicsDataStructures/SpatialHash.cpp)

set( GENERAL_APPS
        "Instancing"
        "ComputeShaderTest"
        "FLIPCPU2DSim"
        )

foreach( testsourcefolder ${GENERAL_APPS} )
    add_executable( ${testsourcefolder} src/${testsourcefolder}/main.cpp)
    target_link_libraries( ${testsourcefolder} VkFluidSim GL glfw Vulkan::Vulkan)
endforeach( testsourcefolder ${GENERAL_APPS} )

set( SPH_APPS
        "SPHCPU2DSim"
        "SPHGPU2DSim"
        "SPHCPU3DSim"
        )

foreach( testsourcefolder ${SPH_APPS} )
    add_executable( ${testsourcefolder} src/SPH/${testsourcefolder}/main.cpp)
    target_link_libraries( ${testsourcefolder} VkFluidSim GL glfw Vulkan::Vulkan)
endforeach( testsourcefolder ${SPH_APPS} )

set( GRID_APPS
        "Grid2DSim"
        "Grid3DSim"
        )

foreach( testsourcefolder ${GRID_APPS} )
    add_executable( ${testsourcefolder} src/grid/${testsourcefolder}/main.cpp)
    target_link_libraries( ${testsourcefolder} VkFluidSim GL glfw Vulkan::Vulkan)
endforeach( testsourcefolder ${GRID_APPS} )
