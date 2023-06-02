# Simple Grid Fluid Simulation

This is a simple grid fluid simulation implementation. The main idea is to use a grid to simulate
fluid attributes, such as density and velocity. Then the fluid can be updated using 
algorithms based on the Navier-Stokes equations.

There are two simulations in this folder one being 2D and the other 3D. The 2D version is more polished,
to test it a release build is recommended.

## Controls

For the 2D simulation you can press to create fluid and move to change the velocities. 
In wall mode you can press and move the mouse to create walls.

For the 3D simulations:
- W: forward
- S: back
- A: left
- D: right
- Q: up
- E: down
- Arrow Keys: move the camera

## References

The ideas and algorithm used in this simulation were based on these papers:

- **Stable Fluids** by Jos Stam 
<br>https://pages.cs.wisc.edu/~chaol/data/cs777/stam-stable_fluids.pdf
- **Real-Time Fluid Dynamics for Games** by Jos Stam 
<br> http://graphics.cs.cmu.edu/nsp/course/15-464/Fall09/papers/StamFluidforGames.pdf

## Building:

For building the project checkout the [project README](../../README.md)


## License
This project is distributed under the [MIT license](../../LICENSE.md).