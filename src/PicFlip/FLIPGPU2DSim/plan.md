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

- [ ] Draw particles with buffers in the GPU
- [ ] Create kernels
  - [ ] Advect particles
  
  - [ ] Transfer particles to GRID
    - [ ] Reset grid (reset velocity, weights, types, rhs and pressure) 
    - [ ] Add vel component to grid
    
  - [ ] Apply Weights and Gravity

  - [ ] Apply boundary conditions
      - [ ] x axis
      - [ ] y axis

  - [ ] Project
    - [ ] Set previous velocity as current
    - [x] Create matrix
    - [x] Solve pressure
    - [ ] Apply pressure on velocities
    

  
  - [ ] Transfer grid velocities to particles
    - [ ] Use PIC
    - [ ] Add FLIP
  
  - [ ] Extend velocities
    - [ ] Reset air cells
    - [ ] Expand velocities