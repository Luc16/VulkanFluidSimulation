# FLIP on the GPU

## CG

- [x] Create a vector on the GPU, add one and print it
- [x] Implement vector addition
- [x] Implement dot product
- [x] Use the vector to print a matrix
- [x] Implement matrix multiplication by a vector
- [x] Create a scenario with a grid with only gravity
- [x] Calculate determinant
- [x] Transfer buffers to PressureSolver
- [x] Implement CG
- [x] Test convergence for initial scenario

## FLIP

- [x] Draw particles with buffers in the GPU
- [x] Create kernels
  - [x] Advect particles
  
  - [x] Transfer particles to GRID
    - [x] Reset grid (reset velocity, weights, types, rhs and pressure) 
    - [x] Add vel component to grid
    
  - [x] Apply Weights and Gravity

  - [x] Apply boundary conditions

  - [x] Project
    - [x] Set previous velocity as current
    - [x] Create matrix
    - [x] Solve pressure
    - [x] Apply pressure on velocities

  - [x] Transfer grid velocities to particles
    - [x] Use PIC
    - [x] Add FLIP
  
  - [x] Extend velocities
