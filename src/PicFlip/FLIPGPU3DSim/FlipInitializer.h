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

    void damBreakInitializer(ComputeUniformBufferObject& cUbo, uint32_t& particlesToAdd, bool dislocatePos, std::vector<glm::vec4>& pPos, std::vector<glm::vec4>& pVel) const;
    void doubleDamBreakInitializer(ComputeUniformBufferObject& cUbo, uint32_t& particlesToAdd, bool dislocatePos, std::vector<glm::vec4>& pPos, std::vector<glm::vec4>& pVel) const;
    void splashInitializer(ComputeUniformBufferObject& cUbo, uint32_t& particlesToAdd, bool dislocatePos, std::vector<glm::vec4>& pPos, std::vector<glm::vec4>& pVel) const;
    void waterfallInitializer(ComputeUniformBufferObject& cUbo, uint32_t& particlesToAdd, bool dislocatePos, std::vector<glm::vec4>& pPos, std::vector<glm::vec4>& pVel) const;



private:
    void placeParticlesInCell(std::vector<glm::vec4>& particles, uint32_t& p, uint32_t i, uint32_t j, uint32_t k, float cellSize, bool dislocatePos = false) const;

    glm::ivec3 particlesPerCell{};


};


#endif //VULKANFLUIDSIMULATION_FLIPINITIALIZER_H
