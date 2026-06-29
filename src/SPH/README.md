# Particle-Based Fluid Simulations (PBF & SPH)

This directory contains implementations of purely Lagrangian, particle-based fluid solvers. Instead of using a fixed grid, these methods represent the fluid as a set of discrete particles that carry physical properties (like mass and velocity) and move freely through space.

This folder includes both CPU and GPU (Vulkan Compute) implementations for two distinct algorithms: **Position Based Fluids (PBF)** and **Smoothed Particle Hydrodynamics (SPH)**.

## Position Based Fluids (PBF)

PBF builds upon the Position Based Dynamics (PBD) framework. Unlike standard SPH, which uses forces to correct density errors, PBF enforces fluid incompressibility by iteratively projecting particle positions directly. This allows the simulation to use much larger timesteps while remaining completely stable, eliminating the "bouncy" or compressible artifacts often seen in basic SPH.

### Bunny Bath
<p align="center">
  <img width="70%" src="path/to/pbf_bunny_bath.gif" alt="PBF Bunny Bath Simulation">
</p>

### Dam Break
<p align="center">
  <img width="70%" src="path/to/pbf_dam_break.gif" alt="PBF Dam Break Simulation">
</p>

### City Waves
<p align="center">
  <img width="70%" src="path/to/pbf_city_waves.gif" alt="PBF City Waves Simulation">
</p>

**Reference Literature:**
* Macklin, M., & Müller, M. (2013). *[Position Based Fluids](https://mmacklin.com/pbf_sig_preprint.pdf)*. ACM Transactions on Graphics (SIGGRAPH).
* Horváth, Z., & Herout, A. (2012). *Real-time particle simulation of fluids*. (Reference for screen-space rendering techniques).

---

## Smoothed Particle Hydrodynamics (SPH)

SPH calculates fluid properties at any point in space by evaluating the contributions of neighboring particles using a smoothing kernel function. It computes forces (like pressure and viscosity) to update particle velocities and positions. While highly popular for interactive applications, it often requires small timesteps to maintain incompressibility and stability.

### Dam Break Simulation
<p align="center">
  <img width="70%" src="path/to/sph_dam_break.gif" alt="SPH Dam Break Simulation">
</p>

**Reference Paper:**
* Müller, M., Charypar, D., & Gross, M. (2003). *[Particle-Based Fluid Simulation for Interactive Applications](https://matthias-research.github.io/pages/publications/sca03.pdf)*. Symposium on Computer Animation.
