//
// Created by luc on 09/05/24.
//

#ifndef VULKANFLUIDSIMULATION_PBFINITIALIZER_H
#define VULKANFLUIDSIMULATION_PBFINITIALIZER_H

#include <vector>
#include "../../lib/utils.h"
#include "particles_and_ubos.h"

class PbfInitializer {

public:
    explicit PbfInitializer(ParticleData& particles);
    void damBreakInitializer(const ComputeUniformBufferObject& cUbo, bool activateRandomOffsets = false);
    void doubleDamBreakInitializer(const ComputeUniformBufferObject& cUbo, bool activateRandomOffsets = false);
    void splashInitializer(const ComputeUniformBufferObject& cUbo, bool activateRandomOffsets = false);

private:
    ParticleData& m_particles;
};


#endif //VULKANFLUIDSIMULATION_PBFINITIALIZER_H
