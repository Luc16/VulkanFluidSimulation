//
// Created by luc on 16/05/24.
//

#include "FlipInitializer.h"

void FlipInitializer::damBreakInitializer(ComputeUniformBufferObject &cUbo, uint32_t& particlesToAdd, bool dislocatePos, std::vector<glm::vec4>& pPos, std::vector<glm::vec4>& pVel) const {

    glm::ivec3 particleStart{2, 2, 2};
    glm::ivec3 particleSpan{cUbo.dim.x/2, cUbo.dim.y - 4, cUbo.dim.z/2};

    uint32_t p = 0;
    for (uint32_t j = particleStart.y; j < particleStart.y + particleSpan.y; j++) {
        for (uint32_t k = particleStart.z; k < particleStart.z + particleSpan.z; k++) {
            for (uint32_t i = particleStart.x; i < particleStart.x + particleSpan.x; i++) {
                placeParticlesInCell(pPos, p, i, j, k, cUbo.cellSize, dislocatePos);
            }
        }
    }

//    if (p < numParticles) {
//        throw std::runtime_error("Too many particles to fit in grid");
//    }


}

void FlipInitializer::doubleDamBreakInitializer(ComputeUniformBufferObject &cUbo, uint32_t& particlesToAdd, bool dislocatePos, std::vector<glm::vec4>& pPos, std::vector<glm::vec4>& pVel) const {

    glm::ivec3 particleStart{2, 2, 2};
    glm::ivec3 particleSpan{3*cUbo.dim.x/8, cUbo.dim.y - 4, 3*cUbo.dim.z/8};

    uint32_t p = 0;
    for (uint32_t j = particleStart.y; j < particleStart.y + particleSpan.y; j++) {
        for (uint32_t k = particleStart.z; k < particleStart.z + particleSpan.z; k++) {
            for (uint32_t i = particleStart.x; i < particleStart.x + particleSpan.x; i++) {
                placeParticlesInCell(pPos, p, i, j, k, cUbo.cellSize, dislocatePos);
                if (p >= pPos.size()/2) {
                    goto endFirstHalf;
                }
            }
        }
    } endFirstHalf:

    particleStart = cUbo.dim - particleStart - particleSpan;

    for (uint32_t j = particleStart.y; j < particleStart.y + particleSpan.y; j++) {
        for (uint32_t k = particleStart.z; k < particleStart.z + particleSpan.z; k++) {
            for (uint32_t i = particleStart.x; i < particleStart.x + particleSpan.x; i++) {
                placeParticlesInCell(pPos, p, i, j, k, cUbo.cellSize, dislocatePos);
            }
        }
    }



//    if (p < numParticles) {
//        throw std::runtime_error("Too many particles to fit in grid");
//    }


}

void FlipInitializer::splashInitializer(ComputeUniformBufferObject &cUbo, uint32_t& particlesToAdd, bool dislocatePos, std::vector<glm::vec4>& pPos, std::vector<glm::vec4>& pVel) const {
    glm::ivec3 particleStart{2, 2, 2};
    glm::ivec3 particleSpan{cUbo.dim.x - 2, cUbo.dim.y - 4, cUbo.dim.z - 2};

    float radius = 6*cUbo.cellSize;

    uint32_t numCellsR = std::floor(radius/cUbo.cellSize);

    glm::ivec3 center{cUbo.dim.x/2, 3*cUbo.dim.y/4, cUbo.dim.z/2};
    uint32_t p = 0;

    for (uint32_t j = center.y - numCellsR; j < center.y + numCellsR; j++) {
        for (uint32_t k = center.z - numCellsR; k < center.z + numCellsR; k++) {
            for (uint32_t i = center.x - numCellsR; i < center.x + numCellsR; i++) {
                auto cellCenter = glm::vec3(
                        float(i)*cUbo.cellSize + cUbo.cellSize/2,
                        float(j)*cUbo.cellSize + cUbo.cellSize/2,
                        float(k)*cUbo.cellSize + cUbo.cellSize/2
                        );
                if (glm::distance(glm::vec3(center)*cUbo.cellSize, cellCenter) < radius) {
                    placeParticlesInCell(pPos, p, i, j, k, cUbo.cellSize, dislocatePos);
                }
            }
        }
    }

    for (uint32_t j = particleStart.y; j < particleStart.y + particleSpan.y; j++) {
        for (uint32_t k = particleStart.z; k < particleStart.z + particleSpan.z; k++) {
            for (uint32_t i = particleStart.x; i < particleStart.x + particleSpan.x; i++) {
                placeParticlesInCell(pPos, p, i, j, k, cUbo.cellSize, dislocatePos);
            }
        }
    }

}

void FlipInitializer::waterfallInitializer(ComputeUniformBufferObject &cUbo, uint32_t& particlesToAdd, bool dislocatePos, std::vector<glm::vec4>& pPos, std::vector<glm::vec4>& pVel) const {
    glm::ivec3 particleStart{2, 2, 2};
    glm::ivec3 particleSpan{std::floor(5.0/cUbo.cellSize), cUbo.dim.y - 4, cUbo.dim.z - 2};

    float radius = cUbo.cellSize*2.5f;

    uint32_t numCellsR = std::floor(radius/cUbo.cellSize);

    glm::ivec3 center{3, cUbo.dim.y/2 - 6, cUbo.dim.z/2};
    uint32_t p = 0;
    particlesToAdd = 0;


    for (uint32_t j = particleStart.y; j < particleStart.y + particleSpan.y; j++) {
        for (uint32_t k = particleStart.z; k < particleStart.z + particleSpan.z; k++) {
            for (uint32_t i = particleStart.x; i < particleStart.x + particleSpan.x; i++) {
                placeParticlesInCell(pPos, p, i, j, k, cUbo.cellSize, dislocatePos);
                if (p >= cUbo.numParticles/3) {
                    goto endFirstPart;
                }
            }
        }
    } endFirstPart:

    uint32_t startParticles = p;

    while (p < cUbo.numParticles) {
        for (uint32_t j = center.y - numCellsR; j < center.y + numCellsR; j++) {
            for (uint32_t k = center.z - numCellsR; k < center.z + numCellsR; k++) {
                for (uint32_t i = center.x; i < center.x + 1; i++) {
                    auto cellCenter = glm::vec3(
                            float(i)*cUbo.cellSize + cUbo.cellSize/2,
                            float(j)*cUbo.cellSize + cUbo.cellSize/2,
                            float(k)*cUbo.cellSize + cUbo.cellSize/2
                    );
                    if (glm::distance(glm::vec3(center)*cUbo.cellSize, cellCenter) < radius) {
                        uint32_t pp = p;
                        placeParticlesInCell(pPos, p, i, j, k, cUbo.cellSize, dislocatePos);
                        for (; pp < p; pp++) {
                            pVel[pp] = glm::vec4(0.05f*cUbo.cellSize/cUbo.dt, 0.0f, 0.0f, 0.0f);
                        }
                    }
                }
            }
        }
        if (particlesToAdd == 0) {
            particlesToAdd = p - startParticles;
        }
    }

    cUbo.numParticles = startParticles + particlesToAdd;
}

void FlipInitializer::placeParticlesInCell(std::vector<glm::vec4> &particles, uint32_t &p, uint32_t i, uint32_t j,
                                           uint32_t k, float cellSize, bool dislocatePos) const {
    for (uint32_t a = 0; a < particlesPerCell.x; a++) {
        for (uint32_t b = 0; b < particlesPerCell.y; b++) {
            for (uint32_t c = 0; c < particlesPerCell.z; c++) {
                if (p < particles.size()) {
                    particles[p++] = glm::vec4(
                            float(i) * cellSize + float(a) * cellSize / float(particlesPerCell.x) +
                            cellSize / float(particlesPerCell.x * particlesPerCell.x),
                            float(j) * cellSize + float(b) * cellSize / float(particlesPerCell.y) +
                            cellSize / float(particlesPerCell.y * particlesPerCell.y),
                            float(k) * cellSize + float(c) * cellSize / float(particlesPerCell.z) +
                            cellSize / float(particlesPerCell.z * particlesPerCell.z),
                            0.0f);
                    if (dislocatePos) particles[p-1] += glm::vec4(
                                randomFloat(0.0f, 0.3f * cellSize),
                                randomFloat(0.0f, 0.3f * cellSize),
                                randomFloat(0.0f, 0.3f * cellSize),
                                0.0f);
                }
            }
        }
    }
}


void FlipInitializer::cityInitializer(ComputeUniformBufferObject &cUbo, uint32_t& particlesToAdd, bool dislocatePos, std::vector<glm::vec4>& pPos, std::vector<glm::vec4>& pVel) const {

    glm::ivec3 particleStart{cUbo.dim.x-3, 2, 2};
    glm::ivec3 particleSpan{cUbo.dim.x/5, cUbo.dim.y - 4, cUbo.dim.z};

    uint32_t p = 0;
    for (uint32_t j = particleStart.y; j < particleStart.y + particleSpan.y; j++) {
        for (uint32_t k = particleStart.z; k < particleStart.z + particleSpan.z; k++) {
            for (int i = particleStart.x; i > particleStart.x - particleSpan.x; i--) {
//                if (k < 23 || k > 27)
                    placeParticlesInCell(pPos, p, i, j, k, cUbo.cellSize, dislocatePos);
            }
        }
    }
}