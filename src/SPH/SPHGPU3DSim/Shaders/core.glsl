#ifndef CORE_SHADER_H
#define CORE_SHADER_H

struct computeUBO {
    float deltaTime;
    vec3 BOUNDARY_SIZE;
    float planeY;

    vec3 G;   // external (gravitational) forces
    float REST_DENS;  // rest density
    float GAS_CONST; // const for equation of state
    float H;           // kernel radius
    float HSQ;        // radius^2 for optimization
    float MASS;
    float VISC;       // viscosity constant
    float DT;       // integration timestep
    float POLY6;
    float SPIKY_GRAD;
    float VISC_LAP;
    float EPS;
    float BOUND_DAMPING;
    uint numParticles;
    uint GRID_SIZE;
};

#endif //CORE_SHADER_H