//
// Created by luc on 09/05/24.
//

#include "PbfInitializer.h"


PbfInitializer::PbfInitializer(ParticleData& particles): m_particles(particles) {}

uint32_t PbfInitializer::damBreakInitializer(ComputeUniformBufferObject& cUbo, bool activateRandomOffsets) {
    uint32_t count = 0;
    auto numParticlesXZ = glm::ivec2(int(std::cbrt(cUbo.numParticles)));
    float particleSpacing = cUbo.H*0.56f;
    float particleVerticalSpacing = cUbo.H*0.50f;
    glm::vec4 initialPos = {cUbo.EPS, cUbo.EPS, cUbo.EPS, 0};
    auto accPos = initialPos;
    for (uint32_t i = 0; i < cUbo.numParticles; i++) {
        m_particles.position[i] = accPos;
        if (activateRandomOffsets) {
            m_particles.position[i] += glm::vec4(
                    randomFloat((accPos.x != cUbo.EPS) ? -cUbo.H/5 : 0.0f, cUbo.H/5),
                    randomFloat((accPos.y != cUbo.EPS) ? -cUbo.H/5 : 0.0f, cUbo.H/5),
                    randomFloat((accPos.z != cUbo.EPS) ? -cUbo.H/5 : 0.0f, cUbo.H/5),
                    1.0f);
        }
        m_particles.density[i] = cUbo.REST_DENS;
        m_particles.type[i] = 0;
        m_particles.velocity[i] = glm::vec4(0.0f);
        accPos.x += particleSpacing;

        if (i % numParticlesXZ.x == numParticlesXZ.x - 1) {
            count++;
            accPos.z += particleSpacing;
            accPos.x = initialPos.x;
            if (count == numParticlesXZ.y) {
                count = 0;
                accPos.y += particleVerticalSpacing;
                accPos.z = initialPos.z;
            }
        }
    }

    return 0;

}

uint32_t PbfInitializer::doubleDamBreakInitializer(ComputeUniformBufferObject &cUbo, bool activateRandomOffsets) {
    uint32_t count = 0;
    auto numParticlesXZ = glm::ivec2(int(std::cbrt(cUbo.numParticles/2)));
    float particleSpacing = cUbo.H*0.56f;
    float particleVerticalSpacing = cUbo.H*0.50f;
    glm::vec4 initialPos = {cUbo.EPS, cUbo.EPS, cUbo.EPS, 0};
    auto accPos = initialPos;
    for (uint32_t i = 0; i < cUbo.numParticles/2; i++) {
        m_particles.position[i] = accPos;
        if (activateRandomOffsets) {
            m_particles.position[i] += glm::vec4(
                    randomFloat((accPos.x != cUbo.EPS) ? -cUbo.H/5 : 0.0f, cUbo.H/5),
                    randomFloat((accPos.y != cUbo.EPS) ? -cUbo.H/5 : 0.0f, cUbo.H/5),
                    randomFloat((accPos.z != cUbo.EPS) ? -cUbo.H/5 : 0.0f, cUbo.H/5),
                    1.0f);
        }
        m_particles.density[i] = cUbo.REST_DENS;
        m_particles.type[i] = 0;
        m_particles.velocity[i] = glm::vec4(0.0f);
        accPos.x += particleSpacing;

        if (i % numParticlesXZ.x == numParticlesXZ.x - 1) {
            count++;
            accPos.z += particleSpacing;
            accPos.x = initialPos.x;
            if (count == numParticlesXZ.y) {
                count = 0;
                accPos.y += particleVerticalSpacing;
                accPos.z = initialPos.z;
            }
        }
    }

    accPos = initialPos = {cUbo.BOUNDARY_SIZE.x - cUbo.EPS - float(numParticlesXZ.x)*particleSpacing, cUbo.EPS,
                  cUbo.BOUNDARY_SIZE.z - cUbo.EPS - float(numParticlesXZ.y)*particleSpacing, 0};
    for (uint32_t i = cUbo.numParticles/2; i < cUbo.numParticles; i++) {
        m_particles.position[i] = accPos;
        if (activateRandomOffsets) {
            m_particles.position[i] += glm::vec4(
                    randomFloat((accPos.x != cUbo.EPS) ? -cUbo.H/5 : 0.0f, cUbo.H/5),
                    randomFloat((accPos.y != cUbo.EPS) ? -cUbo.H/5 : 0.0f, cUbo.H/5),
                    randomFloat((accPos.z != cUbo.EPS) ? -cUbo.H/5 : 0.0f, cUbo.H/5),
                    1.0f);
        }
        m_particles.density[i] = cUbo.REST_DENS;
        m_particles.type[i] = 0;
        m_particles.velocity[i] = glm::vec4(0.0f);
        accPos.x += particleSpacing;

        if (i % numParticlesXZ.x == numParticlesXZ.x - 1) {
            count++;
            accPos.z += particleSpacing;
            accPos.x = initialPos.x;
            if (count == numParticlesXZ.y) {
                count = 0;
                accPos.y += particleVerticalSpacing;
                accPos.z = initialPos.z;
            }
        }
    }

    return 0;
}

uint32_t PbfInitializer::splashInitializer(ComputeUniformBufferObject &cUbo, bool activateRandomOffsets) {
    float particleSpacing = cUbo.H*0.56f;
    float sphereSpacing = cUbo.H*0.68f;
    float particleVerticalSpacing = cUbo.H*0.50f;

    float radius = 1.2f;
    glm::vec3 center = {cUbo.BOUNDARY_SIZE.x/2, cUbo.BOUNDARY_SIZE.y/2, cUbo.BOUNDARY_SIZE.z/2};
    uint32_t i = 0;
    uint32_t l = 1;
    bool lhalf = false;
    uint32_t numDiscs = std::floor(radius/sphereSpacing);
    while (l > 0) {
        float r = std::sqrt(radius * radius - (radius - float(l) * sphereSpacing) * (radius - float(l) * sphereSpacing));
        uint32_t numLines = std::floor(r/sphereSpacing);
        uint32_t c = 1;
        bool half = false;
        while (c > 0) {
            float z = std::sqrt(r * r - (r - float(c) * sphereSpacing) * (r - float(c) * sphereSpacing));
            uint32_t numCols = std::floor(2 * z / sphereSpacing);
            glm::vec4 initPos = {center.x + (2*float(half) - 1.0f)*(r - float(c)*sphereSpacing), center.y + (2*float(lhalf) - 1.0f)*(radius - float(l)*sphereSpacing), center.z - z, 0.0f};
            glm::vec4 pos = initPos;
            for (uint32_t k = 0; k < numCols; k++) {
                m_particles.position[i] = pos;
                if (activateRandomOffsets) {
                    m_particles.position[i] += glm::vec4(
                            randomFloat((pos.x != cUbo.EPS) ? -cUbo.H/5 : 0.0f, cUbo.H/5),
                            randomFloat((pos.y != cUbo.EPS) ? -cUbo.H/5 : 0.0f, cUbo.H/5),
                            randomFloat((pos.z != cUbo.EPS) ? -cUbo.H/5 : 0.0f, cUbo.H/5),
                            1.0f);
                }

                pos.z += sphereSpacing;
                m_particles.density[i] = cUbo.REST_DENS;
                m_particles.type[i] = 0;
                m_particles.velocity[i] = glm::vec4(0.0f);

                i++;
                if (i == cUbo.numParticles) {
                    return 0;
                }
            }

            if (!half) {
                c++;
                if (c > numLines) {
                    half = true;
                }
            } else {
                c--;
            }
//            std::cout << "c: " << c << '\n';

        }
        if (!lhalf) {
            l++;
            if (l > numDiscs) {
                lhalf = true;
            }
        } else {
            l--;
        }
    }

    glm::vec4 initialPos = {cUbo.EPS, cUbo.EPS, cUbo.EPS, 0};
    glm::vec4 limitPos = {cUbo.BOUNDARY_SIZE.x - 1.2*cUbo.EPS, cUbo.BOUNDARY_SIZE.y - 1.2*cUbo.EPS, cUbo.BOUNDARY_SIZE.z - 1.2*cUbo.EPS, 0};
    auto accPos = initialPos;
    for (; i < cUbo.numParticles; i++) {
        m_particles.position[i] = accPos;
        if (activateRandomOffsets) {
            m_particles.position[i] += glm::vec4(
                    randomFloat((accPos.x != cUbo.EPS) ? -cUbo.H/5 : 0.0f, cUbo.H/5),
                    randomFloat((accPos.y != cUbo.EPS) ? -cUbo.H/5 : 0.0f, cUbo.H/5),
                    randomFloat((accPos.z != cUbo.EPS) ? -cUbo.H/5 : 0.0f, cUbo.H/5),
                    1.0f);
        }
        m_particles.density[i] = cUbo.REST_DENS;
        m_particles.type[i] = 0;
        m_particles.velocity[i] = glm::vec4(0.0f);
        accPos.x += particleSpacing;

        if (accPos.x + particleSpacing >= limitPos.x) {
            accPos.z += particleSpacing;
            accPos.x = initialPos.x;
            if (accPos.z + particleSpacing >= limitPos.z) {
                accPos.y += particleVerticalSpacing;
                accPos.z = initialPos.z;
            }
        }
    }

//    for (uint32_t i = 3*cUbo.numParticles/4; i < cUbo.numParticles; i++) {
////        // place particles in a sphere
////        float theta = randomFloat(0.0f, 2*glm::pi<float>());
////        float phi = randomFloat(0.0f, glm::pi<float>());
////        float r = randomFloat(0.0f, std::min(cUbo.BOUNDARY_SIZE.x, cUbo.BOUNDARY_SIZE.z)/8);
////        m_particles.position[i] = glm::vec4(
////                r*std::sin(phi)*std::cos(theta) + cUbo.BOUNDARY_SIZE.x/2,
////                r*std::cos(phi) + cUbo.BOUNDARY_SIZE.y/2,
////                r*std::sin(phi)*std::sin(theta) + cUbo.BOUNDARY_SIZE.z/2,
////                1.0f);
//
//
//    }

    return 0;
}


uint32_t PbfInitializer::waterFallInitializer(ComputeUniformBufferObject &cUbo, bool activateRandomOffsets) {
    float particleSpacing = cUbo.H*0.56f;
    float sphereSpacing = cUbo.H*0.68f;
    float particleVerticalSpacing = cUbo.H*0.50f;
    float r = 0.5f;
    glm::vec4 center = {cUbo.EPS*5.0f, cUbo.BOUNDARY_SIZE.y/2, cUbo.BOUNDARY_SIZE.z/2, 0.0f};
    uint32_t numLines = std::floor(r/sphereSpacing);
    uint32_t c = 1;
    bool half = false;
    std::vector<glm::vec4> positions;
    while (c > 0) {
        float z = std::sqrt(r * r - (r - float(c) * sphereSpacing) * (r - float(c) * sphereSpacing));
        uint32_t numCols = std::floor(2 * z / sphereSpacing);
        glm::vec4 initPos = {center.x, center.y + (2*float(half) - 1.0f)*(r - float(c)*sphereSpacing), center.z - z, 0.0f};
        glm::vec4 pos = initPos;
        for (uint32_t k = 0; k < numCols; k++) {
            positions.push_back(pos);
            pos.z += sphereSpacing;
        }
        if (!half) {
            c++;
            if (c > numLines) {
                half = true;
            }
        } else {
            c--;
        }
    }

    glm::vec4 initialPos = {cUbo.EPS, cUbo.EPS, cUbo.EPS, 0};
    glm::vec4 limitPos = {5.0f - 1.2*cUbo.EPS, cUbo.BOUNDARY_SIZE.y - 1.2*cUbo.EPS, cUbo.BOUNDARY_SIZE.z - 1.2*cUbo.EPS, 0};
    auto accPos = initialPos;
    for (uint32_t i = 0; i < cUbo.numParticles/2; i++) {
        m_particles.position[i] = accPos;
        if (activateRandomOffsets) {
            m_particles.position[i] += glm::vec4(
                    randomFloat((accPos.x != cUbo.EPS) ? -cUbo.H/5 : 0.0f, cUbo.H/5),
                    randomFloat((accPos.y != cUbo.EPS) ? -cUbo.H/5 : 0.0f, cUbo.H/5),
                    randomFloat((accPos.z != cUbo.EPS) ? -cUbo.H/5 : 0.0f, cUbo.H/5),
                    1.0f);
        }
        m_particles.density[i] = cUbo.REST_DENS;
        m_particles.type[i] = 0;
        m_particles.velocity[i] = glm::vec4(0.0f);
        accPos.x += particleSpacing;

        if (accPos.x + particleSpacing >= limitPos.x) {
            accPos.z += particleSpacing;
            accPos.x = initialPos.x;
            if (accPos.z + particleSpacing >= limitPos.z) {
                accPos.y += particleVerticalSpacing;
                accPos.z = initialPos.z;
            }
        }
    }

    for (uint32_t i = cUbo.numParticles/2; i < cUbo.numParticles; i++) {
        m_particles.position[i] = positions[i%positions.size()];
        m_particles.density[i] = cUbo.REST_DENS;
        m_particles.type[i] = 0;
        m_particles.velocity[i] = glm::vec4(0.4f * cUbo.H / cUbo.DT, 0.0f, 0.0f, 0.0f);
    }

    cUbo.numParticles /= 2;

    return positions.size();
}