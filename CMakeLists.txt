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
        src/grid/Grid2DSim/Grid2D.cpp src/grid/Grid2DSim/Grid2D.h
        src/lib/ComputeSystem.cpp src/lib/ComputeSystem.h src/lib/ComputeShaderHandler.cpp src/lib/ComputeShaderHandler.h
        src/lib/graphicsDataStructures/Matrices.h src/lib/graphicsDataStructures/SpatialHash.h src/lib/graphicsDataStructures/SpatialHash.cpp
        src/lib/graphicsDataStructures/SpatialGrid.h src/lib/graphicsDataStructures/SpatialGrid.cpp
        src/SPH/SPHGPU3DSim/GpuSpatialGridHandler.cpp src/SPH/SPHGPU3DSim/GpuSpatialGridHandler.h
        src/SPH/PBFGPU3DSim/PbfGpuSpatialGridHandler.cpp src/SPH/PBFGPU3DSim/PbfGpuSpatialGridHandler.h
        src/lib/simulations/OffscreenPass.cpp
        src/lib/simulations/OffscreenPass.h
        src/lib/CubeMapModel.cpp
        src/lib/CubeMapModel.h
        external/imgui/imgui_stdlib.h
)

set( GENERAL_APPS
        "Instancing"
        "PointSpheres"
        "ComputeShaderTest"
        "SineWaveFluidSim"
        )

foreach( testsourcefolder ${GENERAL_APPS} )
    add_executable( ${testsourcefolder} src/${testsourcefolder}/main.cpp src/${testsourcefolder}/${testsourcefolder}.h src/${testsourcefolder}/${testsourcefolder}.cpp)
    target_link_libraries( ${testsourcefolder} VkFluidSim GL glfw Vulkan::Vulkan)
endforeach( testsourcefolder ${GENERAL_APPS} )

set( GRID_APPS
        "Grid2DSim"
        "Grid3DSim"
        )

foreach( testsourcefolder ${GRID_APPS} )
    add_executable( ${testsourcefolder} src/grid/${testsourcefolder}/main.cpp src/grid/${testsourcefolder}/${testsourcefolder}.h src/grid/${testsourcefolder}/${testsourcefolder}.cpp)
    target_link_libraries( ${testsourcefolder} VkFluidSim GL glfw Vulkan::Vulkan)
endforeach( testsourcefolder ${GRID_APPS} )

set( SPH_APPS
        "SPHCPU2DSim"
        "SPHGPU2DSim"
        "SPHCPU3DSim"
        "SPHGPU3DSim"
        "PBFCPU3DSim"
        "PBFGPU3DSim"
        )

foreach( testsourcefolder ${SPH_APPS} )
    add_executable( ${testsourcefolder} src/SPH/${testsourcefolder}/main.cpp src/SPH/${testsourcefolder}/${testsourcefolder}.h src/SPH/${testsourcefolder}/${testsourcefolder}.cpp
            src/SPH/PBFGPU3DSim/particles_and_ubos.h
            src/SPH/PBFGPU3DSim/RigidObject.cpp
            src/SPH/PBFGPU3DSim/RigidObject.h
            src/SPH/PBFGPU3DSim/PbfInitializer.cpp
            src/SPH/PBFGPU3DSim/PbfInitializer.h)
    target_link_libraries( ${testsourcefolder} VkFluidSim GL glfw Vulkan::Vulkan)
endforeach( testsourcefolder ${SPH_APPS} )

set( FLIP_APPS
        "FLIPCPU2DSim"
        "FLIPGPU2DSim"
        "FLIPGPU3DSim"
)

foreach( testsourcefolder ${FLIP_APPS} )
    add_executable( ${testsourcefolder} src/PicFlip/${testsourcefolder}/main.cpp src/PicFlip/${testsourcefolder}/${testsourcefolder}.h src/PicFlip/${testsourcefolder}/${testsourcefolder}.cpp
            src/PicFlip/${testsourcefolder}/structs.h
            src/PicFlip/${testsourcefolder}/PressureSolver.h
            src/PicFlip/${testsourcefolder}/PressureSolver.cpp
            src/PicFlip/${testsourcefolder}/FlipSolver.cpp
            src/PicFlip/${testsourcefolder}/FlipSolver.h
            src/PicFlip/FLIPGPU3DSim/FlipRenderer.cpp
            src/PicFlip/FLIPGPU3DSim/FlipRenderer.h
            src/PicFlip/FLIPGPU3DSim/RigidObject.cpp
            src/PicFlip/FLIPGPU3DSim/RigidObject.h
    )
    target_link_libraries( ${testsourcefolder} VkFluidSim GL glfw Vulkan::Vulkan)
endforeach( testsourcefolder ${FLIP_APPS} )

