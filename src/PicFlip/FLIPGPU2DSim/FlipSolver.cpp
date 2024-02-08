//
// Created by luc on 25/01/24.
//

#include "FlipSolver.h"

void FlipSolver::updateSimulation(float deltaTime, float flipRatio) {



}

void FlipSolver::initializeParticles()  {
    m_testKernel.createPipeline();


//    uint32_t k = 0, na = 3, nb = 3;
//    uint32_t s = numTilesX/4, e = 3*numTilesX/4;
////    uint32_t s = 0, e = numTilesX - 1;
//    for (uint32_t j = 4 ; j < numTilesY-1; j++) {
//        for (uint32_t i = s; i < e; i++) {
//            for (uint32_t a = 0; a < na; a++){
//                for (uint32_t b = 0; b < nb; b++) {
//                    if (k >= particles.size()) return;
//                    auto& particle = particles[k++];
//                    particle.position = glm::vec3(
//                            i*cellSize + float(a*cellSize)/float(na) +
//                            float(cellSize)/float(na*na) + randomFloat(0.0f, 0.3f*cellSize),
//                            j*cellSize + float(b*cellSize)/float(nb) +
//                            float(cellSize)/float(nb*nb) + randomFloat(0.0f, 0.3f*cellSize),
//                            0.0f);
//                    particle.velocity = glm::vec3(0.0f, 0.0f, 0.0f);
//                    particle.color = glm::vec4(0.2f, 0.6f, 1.0f, 1.0f);
//                }
//            }
//        }
//    }
}