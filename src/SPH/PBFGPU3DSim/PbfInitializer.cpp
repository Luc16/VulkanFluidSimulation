//
// Created by luc on 09/05/24.
//

#include "PbfInitializer.h"


PbfInitializer::PbfInitializer(ParticleData& particles): m_particles(particles) {}

void PbfInitializer::damBreakInitializer(const ComputeUniformBufferObject& cUbo, const glm::vec4& initialPos, const glm::ivec2& numParticlesXZ, float particleSpacing, float particleVerticalSpacing, bool activateRandomOffsets) {
    uint32_t count = 0;
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

    accPos = glm::vec4(0.75*cUbo.BOUNDARY_SIZE.x + cUbo.EPS, cUbo.EPS, cUbo.EPS, 0.0f);

}