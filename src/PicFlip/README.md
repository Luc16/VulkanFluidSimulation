# PIC / FLIP Fluid Simulations

This directory contains implementations of hybrid Eulerian-Lagrangian fluid solvers utilizing the **Particle-In-Cell (PIC)** and **Fluid-Implicit-Particle (FLIP)** methods. Implementations are provided for 2D CPU, 2D GPU, and 3D GPU environments.

## How it Works

Pure grid-based (Eulerian) methods suffer from numerical dissipation, making fluids look viscous and losing turbulent details over time. Pure particle-based (Lagrangian) methods preserve details but struggle with efficient, stable pressure solves. 

The PIC/FLIP method combines the strengths of both:
1. **Particles (Lagrangian):** Used to track fluid quantities and advect velocities through space without numerical smoothing.
2. **Background Grid (Eulerian):** Used to efficiently solve the pressure projection equation to enforce incompressibility.

During the simulation step, velocities are gathered from the particles to the grid, pressure is solved, and the new velocities are mapped back to the particles. The final particle velocity is typically a weighted blend of FLIP (highly turbulent, preserves energy) and PIC (stable, but dissipative) to achieve visually appealing, splashy fluids.

### Double Dam Break Simulation
<p align="center">
  <img width="70%" src="https://github.com/user-attachments/assets/79a250dc-56d1-42f8-9404-9d9bff97e543" alt="PIC/FLIP Double Dam Break Simulation">
</p>

**Reference Literature:**
* Bridson, R. (2015). *Fluid Simulation for Computer Graphics*. CRC Press. 
* Zhu, Y., & Bridson, R. (2005). *Animating Sand as a Fluid*. ACM Transactions on Graphics (SIGGRAPH).
* Horváth, Z., & Herout, A. (2012). *Real-time particle simulation of fluids*. (Reference for screen-space rendering techniques).
