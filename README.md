# Fluid simulations in Vulkan
[![pt-br](https://img.shields.io/badge/LEIAME-PT--BR-A3BE8C.svg?style=for-the-badge)](README-ptbr.md)


This repository has many different fluid simulations algorithms implemented using the vulkan API. 
The most relevant simulations have separate READMEs explaining its particularities. The core of the
vulkan code can be found in the [lib](src/lib) folder.

## Relevant Simulations

### [Grid Simulations](src/grid)

These are fluid simulations where a grid is used to store fluid attributes, such as density and velocity. 

### [SPH Simulations](src/SPH)

These are simulations based on the Smoothed Particle Hydrodynamics technique, which uses
particles to approximate the behavior of fluids.

## Dependencies
[//]: # (TODO: Explain how to setup each of the dependencies or link where they can be aquired  )
To build the project and all the executables, these dependencies are required:

- Vulkan
- glfw3
- CMake >= 3.24.0

## Building

The project can be built using CMake. On linux, these commands can be run from the
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
cmake -Bbuild-release
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