//
// Created by luc on 16/05/24.
//

#ifndef VULKANFLUIDSIMULATION_FLIPINITIALIZER_H
#define VULKANFLUIDSIMULATION_FLIPINITIALIZER_H

#include "../../lib/utils.h"
#include "structs.h"

class FlipInitializer {
public:
    explicit FlipInitializer(glm::ivec3 particlesPerCell): particlesPerCell(particlesPerCell){}

    std::vector<glm::vec4> damBreakInitializer(ComputeUniformBufferObject& cUbo, bool dislocatePos = false) const;
    std::vector<glm::vec4> doubleDamBreakInitializer(ComputeUniformBufferObject& cUbo, bool dislocatePos = false) const;
    std::vector<glm::vec4> splashInitializer(ComputeUniformBufferObject& cUbo, bool dislocatePos = false) const;



private:
    void placeParticlesInCell(std::vector<glm::vec4>& particles, uint32_t& p, uint32_t i, uint32_t j, uint32_t k, float cellSize, bool dislocatePos = false) const;

    glm::ivec3 particlesPerCell{};


};


#endif //VULKANFLUIDSIMULATION_FLIPINITIALIZER_H
