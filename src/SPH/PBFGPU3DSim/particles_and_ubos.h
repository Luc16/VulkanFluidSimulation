//
// Created by luc on 20/11/23.
//

#ifndef VULKANFLUIDSIMULATION_PARTICLES_AND_UBOS_H
#define VULKANFLUIDSIMULATION_PARTICLES_AND_UBOS_H

#include "../../lib/utils.h"


// these vec4s can be read as vec3 in shader because of the 16 byte spacing
struct ParticleData {
    std::vector<glm::vec4> position{}, velocity{};
    std::vector<float> density{};
    std::vector<uint32_t> gIdx{}, type{};

    void resize(size_t newSize) {
        position.resize(newSize);
        velocity.resize(newSize);
        density.resize(newSize);
        gIdx.resize(newSize);
        type.resize(newSize);
    }
};

struct UniformBufferObject {
    alignas(16) glm::mat4 view;
    alignas(16) glm::mat4 proj;
    alignas(16) glm::mat4 inverseView;
    alignas(16) glm::vec3 lightDir = glm::vec3(1.0f, -1.0f, 0.0f);
    float radius;
    float screenHeight;
    float screenWidth;
    float tanHalfFov = std::tan(glm::radians(50.0f)/2);
    uint32_t renderType = 8;
    float zNear = 0.1f;
    float zFar = 500.0f;
    uint32_t blurMode = 0;
    int filterRadius = 14;
    float blurScale = 0.2;
    float blurDepthFalloff = 1000;
    alignas(16) glm::vec3 planeSize;
    float restDens;
    float transparency = 0.8f;
};

struct ComputeUniformBufferObject {
    alignas(16) glm::vec3 BOUNDARY_SIZE = glm::vec3(8.0f);
    alignas(16) glm::vec3 G = glm::vec3(0.0f, -9.8f, 0.0f);

    float wallX = 0.0f;
    float REST_DENS = 5000.0f;  // rest density
    float H = 0.1f;           // kernel radius
    float HSQ = H * H;        // radius^2 for optimization
    float VISC = 0.0001f;       // viscosity constant
    float DT = 1/60.0f;       // integration timestep
//        float DT = 1/60.0f;       // integration timestep
    float ART_PRESSURE_COEF = 0.00025f;
    float VORTICITY_COEF = 0.0004f;
    uint32_t numParticles = 0;
    uint32_t numRigidParticles = 0;
    float POLY6 = 315.0f / (64.0f * glm::pi<float>() *H*H*H *H*H*H *H*H*H);
    float SPIKY_GRAD = -45.0f / (glm::pi<float>() * H*H*H *H*H*H);

    float CFM = 600.0f;
    float EPS = H; // boundary epsilon
    bool activateVort = true;
};

struct SolverUniformBufferObject {
    uint32_t solverSteps;
    uint32_t step;
};

#endif //VULKANFLUIDSIMULATION_PARTICLES_AND_UBOS_H
