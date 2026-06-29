# Fluid simulations in Vulkan
[![pt-br](https://img.shields.io/badge/LEIAME-PT--BR-A3BE8C.svg?style=for-the-badge)](README-ptbr.md)

This repository has many different fluid simulations algorithms implemented using the Vulkan API. 
The most relevant simulations have separate READMEs explaining their particularities. The core of the
Vulkan code can be found in the [lib](src/lib) folder.

## Relevant Simulations

### [Grid Simulations](src/grid)

These are fluid simulations where a grid buffer is used to store fluid attributes, such as density and velocity.

<p align="center">
  <img width="50%" height="50%" src="https://github.com/Luc16/VulkanFluidSimulation/assets/33912482/d2a46d67-23d1-4e15-a922-cae68596c805" alt="Grid Simulation Example">
</p>

### Particle Fluid Simulations

These are Lagrangian simulations that use discrete particles to approximate the behavior of fluids. This repository implements three different particle-based solvers:
* [**SPH**](/src/SPH/) (Smoothed Particle Hydrodynamics)
* [**PBF**](src/SPH) (Position Based Fluids)
* [**PIC/FLIP**](/src/PicFlip/) (Particle-In-Cell / Fluid-Implicit-Particle)

For more details, visual examples, and reference papers on these algorithms, check the [Particle Simulations README](src/SPH/README.md). Here is an example of a PBF simulation:

<p align="center">
  <img width="70%" src="https://github.com/user-attachments/assets/f68ae8e7-7c6d-42d9-81a0-8e6dc26a9112" alt="PBF Bunny Bath Simulation">
</p>

## Dependencies

To build the project and all the executables, these dependencies are required:

- Vulkan
- glfw3
- CMake >= 3.24.0

## Building

The project can be built using CMake. On Linux, these commands can be run from the
project folder to create the build files:

```
cmake -Bbuild
cd build
make
```

Sometimes is useful to create a build in release mode as well. This makes the 
code faster and is better for most simulations, although it takes longer to compile.
An example of this:

```
cmake -Bbuild-release -DCMAKE_BUILD_TYPE=Release
cd build-release
make
```

All the executable files can be seen by running the `ls` command. To run and executable
just use this command:

```
./executable
```


## License
This project is distributed under the [MIT license](LICENSE.md).
