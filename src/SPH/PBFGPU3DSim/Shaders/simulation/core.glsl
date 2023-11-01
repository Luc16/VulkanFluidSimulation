#ifndef CORE_SHADER_H
#define CORE_SHADER_H


struct computeUBO {
    vec3 BOUNDARY_SIZE;
    vec3 G;

    float planeY;
    float REST_DENS;  // rest density
    float H;           // kernel radius
    float HSQ;        // radius^2 for optimization
    float VISC;       // viscosity constant
    float DT;       // integration timestep
    float ART_PRESSURE_COEF;
    float VORTICITY_COEF;
    uint numParticles;
    float POLY6;
    float SPIKY_GRAD;

    float CFM;
    float EPS; // boundary epsilon
    bool activateVort;
};


#endif //CORE_SHADER_H