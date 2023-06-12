# Simple Grid Fluid Simulation
[![pt-br](https://img.shields.io/badge/LEIAME-PT--BR-A3BE8C.svg?style=for-the-badge)](README-ptbr.md)

This is a simple grid fluid simulation implementation. The main idea is to use a grid to simulate
fluid attributes, such as density and velocity. Then the fluid can be updated using 
algorithms based on the Navier-Stokes equations.

There are two simulations in this folder one being 2D and the other 3D. The 2D version is more polished,
to test it a release build is recommended.

## Video demo

[![SPH Demo](https://i9.ytimg.com/vi_webp/Bzj_LLRlu3w/mq2.webp?sqp=CMyHnqQG-oaymwEmCMACELQB8quKqQMa8AEB-AH-CYAC0AWKAgwIABABGEYgWChlMA8=&rs=AOn4CLCP_AY2ZgvYkWMZBBteyKkHLUIeSA)](https://www.youtube.com/watch?v=Bzj_LLRlu3w)

## Controls

For the 2D simulation the mouse can be pressed to create fluid and it can be moved to change the velocities. 
In wall mode pressing and moving the mouse creates walls.

For the 3D simulations:

| Key          | Action            |
|--------------|-------------------|
| `w`          | move forwards     |
| `s`          | move backwards    |
| `a`          | move left         |
| `d`          | move right        |
| `q`          | move up           |
| `e`          | move down         |
| `Arrow Keys` | rotate the camera |


## References

The ideas and algorithm used in this simulation were based on these papers:

- **Stable Fluids** by Jos Stam 
<br>https://pages.cs.wisc.edu/~chaol/data/cs777/stam-stable_fluids.pdf
- **Real-Time Fluid Dynamics for Games** by Jos Stam 
<br> http://graphics.cs.cmu.edu/nsp/course/15-464/Fall09/papers/StamFluidforGames.pdf

## Building

For building the project checkout the [project README](../../README.md)


## License
This project is distributed under the [MIT license](../../LICENSE.md).