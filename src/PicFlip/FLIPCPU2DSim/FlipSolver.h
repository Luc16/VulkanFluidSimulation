//
// Created by luc on 25/01/24.
//

#ifndef VULKANFLUIDSIMULATION_FLIPSOLVER_H
#define VULKANFLUIDSIMULATION_FLIPSOLVER_H
#include <sstream>

#include "../../lib/utils.h"
#include "../../lib/graphicsDataStructures/Matrices.h"
#include "structs.h"
#include "PressureSolver.h"

enum CellType {
    AIR, SOLID, FLUID
};

template<uint32_t numTilesX, uint32_t numTilesY, uint32_t cellSize, uint32_t numParticles>
class FlipSolver {
public:
    FlipSolver(float particleRadius, uint32_t numIterations, uint32_t extensions):
            particleRadius(particleRadius), numIterations(numIterations), extensions(extensions) {}

    [[nodiscard]] float getVelX(uint32_t i, uint32_t j) { return current.velX(i, j); }
    [[nodiscard]] float getVelY(uint32_t i, uint32_t j) { return current.velY(i, j); }
    [[nodiscard]] CellType getCellType(uint32_t i, uint32_t j) { return cellTypes(i, j); }
    [[nodiscard]] uint32_t hasVelAt(uint32_t i, uint32_t j) { return hasVel(i, j); }
    [[nodiscard]] float getSolidCells(uint32_t i, uint32_t j) { return solidCells(i, j); }

private:
    const static uint32_t totalCells = numTilesX*numTilesY;


    struct FluidData {
        Matrix<float, totalCells, numTilesX> velX{}, velY{};
    };

    [[nodiscard]] glm::ivec2 gridPos(const glm::vec3& pos, uint32_t xShift = 0, uint32_t yShift = 0) const { return {
                uint32_t(pos.x - float(xShift)) / cellSize,
                uint32_t(pos.y - float(yShift)) / cellSize }; };
    [[nodiscard]] glm::ivec2 gridPos(const Particle& particle, uint32_t xShift = 0, uint32_t yShift = 0) const { return gridPos(particle.position, xShift, yShift); }


        float particleRadius;
        uint32_t numIterations;
        uint32_t extensions;
        uint32_t fluidCells = 0;
        float dt = 1/60.0f;

        FluidData current, previous;
        Matrix<CellType, totalCells, numTilesX> cellTypes{};
        Matrix<uint32_t , totalCells, numTilesX> hasVel{};
        Matrix<float, totalCells, numTilesX> weightVelX{}, weightVelY{};
        Matrix<float, totalCells, numTilesX> solidCells{};
        HeapMatrix<double, numTilesX> pressure{totalCells};
        HeapMatrix<double, numTilesX> rhs{totalCells};
        PressureSolver<double, numTilesX> pressureSolver{};

        public:
        std::vector<Particle> particles{numParticles};


        void resetGrid(bool hardReset = false);
        void initializeParticles();
        void advectParticles(float deltaTime);
        static std::tuple<float, float, float, float> particleGridWeights(const glm::vec3& pos, glm::ivec2 gridPos, uint32_t xShift, uint32_t yShift);
        static void applyWeightedValuesToMatrix(Matrix<float, totalCells, numTilesX>& matrix, glm::ivec2 gridPos, std::tuple<float, float, float, float> weights, float value);
        void transferParticlesVelocitiesToGrid();
        void applyWeightsAndGravity();
        glm::vec3 getVelocityFromGrid(const glm::vec3& pos);
        void transferGridVelocitiesToParticles(float flipRatio);
        void enforceBoundaryConditions();
        void extendVelocities();
        void projectVelocities(float deltaTime);
        void updateSimulation(float deltaTime, float flipRatio);
    };

template<uint32_t numTilesX, uint32_t numTilesY, uint32_t cellSize, uint32_t numParticles>
void FlipSolver<numTilesX, numTilesY, cellSize, numParticles>::updateSimulation(float deltaTime, float flipRatio) {
    for (uint32_t _ = 0; _ < 5; _++) {
        advectParticles(0.2f*dt);
    }
    transferParticlesVelocitiesToGrid();
    applyWeightsAndGravity();
    extendVelocities();
    enforceBoundaryConditions();
    projectVelocities(deltaTime);
    transferGridVelocitiesToParticles(flipRatio);
}

template<uint32_t numTilesX, uint32_t numTilesY, uint32_t cellSize, uint32_t numParticles>
void FlipSolver<numTilesX, numTilesY, cellSize, numParticles>::resetGrid(bool hardReset) {
    if (hardReset) {
        for (uint32_t j = 0; j < numTilesY; j++) {
            for (uint32_t i = 0; i < numTilesX; i++) {
                solidCells(i, j) = (i == 0 || i == numTilesX-1 || j == 0) ? 0.0f : 1.0f;
            }
        }
    }
    for (uint32_t i = 0; i < totalCells; i++) {
        if (hardReset) {
            previous.velX[i] = 0.0f;
            previous.velY[i] = 0.0f;
        }
        current.velX[i] = 0.0f;
        current.velY[i] = 0.0f;
        weightVelX[i] = 0.0f;
        weightVelY[i] = 0.0f;
        cellTypes[i] = (i < numTilesX ||
                        i % numTilesX == 0 ||
                        i % numTilesX == numTilesX - 1 ||
                        i > totalCells - numTilesX) ? SOLID: AIR;
        hasVel[i] = 0;
        pressure[i] = 0.0f;
        rhs[i] = 0.0f;
    }
}

template<uint32_t numTilesX, uint32_t numTilesY, uint32_t cellSize, uint32_t numParticles>
void FlipSolver<numTilesX, numTilesY, cellSize, numParticles>::initializeParticles() {
    uint32_t k = 0, na = 3, nb = 3;
    uint32_t s = numTilesX/4, e = 3*numTilesX/4;
//    uint32_t s = 0, e = numTilesX - 1;
    for (uint32_t j = 4 ; j < numTilesY-1; j++) {
        for (uint32_t i = s; i < e; i++) {
            for (uint32_t a = 0; a < na; a++){
                for (uint32_t b = 0; b < nb; b++) {
                    if (k >= particles.size()) return;
                    auto& particle = particles[k++];
                    particle.position = glm::vec3(
                            i*cellSize + float(a*cellSize)/float(na) +
                            float(cellSize)/float(na*na) + randomFloat(0.0f, 0.3f*cellSize),
                            j*cellSize + float(b*cellSize)/float(nb) +
                            float(cellSize)/float(nb*nb) + randomFloat(0.0f, 0.3f*cellSize),
                            0.0f);
                    particle.velocity = glm::vec3(0.0f, 0.0f, 0.0f);
                    particle.color = glm::vec4(0.2f, 0.6f, 1.0f, 1.0f);
                }
            }
        }
    }
}

template<uint32_t numTilesX, uint32_t numTilesY, uint32_t cellSize, uint32_t numParticles>
glm::vec3 FlipSolver<numTilesX, numTilesY, cellSize, numParticles>::getVelocityFromGrid(const glm::vec3& pos) {

    auto velComponent = [this, &pos](uint32_t xShift, uint32_t yShift, Matrix<float, totalCells, numTilesX>& velComponent) {
        glm::ivec2 gPos = gridPos(pos, xShift, yShift);
        auto [wdl, wdr, wul, wur] = particleGridWeights(pos, gPos, xShift, yShift);

        auto vdl = velComponent(gPos);
        auto vdr = velComponent(gPos.x + 1, gPos.y);
        auto vul = velComponent(gPos.x, gPos.y + 1);
        auto vur = velComponent(gPos.x + 1, gPos.y + 1);

        return wdl*vdl + wdr*vdr + wul*vul + wur*vur;
    };

    return glm::vec3(
            velComponent(0, cellSize/2, current.velX),
            velComponent(cellSize/2, 0, current.velY),
            0.0f
    );

}

template<uint32_t numTilesX, uint32_t numTilesY, uint32_t cellSize, uint32_t numParticles>
void FlipSolver<numTilesX, numTilesY, cellSize, numParticles>::advectParticles(float deltaTime) {
    float minX = float(cellSize) + particleRadius, maxX = float(cellSize*(numTilesX-1)) - particleRadius;
    float minY = float(cellSize) + particleRadius, maxY = float(cellSize*(numTilesY-1)) - particleRadius;

    auto clampParticlePosition = [minX, maxX, minY, maxY](Particle& particle) {
        if (particle.position.x < minX) {
            particle.velocity.x = 0;
            particle.position.x = minX + 0.001f;
        }
        if (particle.position.x > maxX) {
            particle.velocity.x = 0;
            particle.position.x = maxX - 0.001f;
        }
        if (particle.position.y < minY) {
            particle.velocity.y = 0;
            particle.position.y = minY + 0.001f;
        }
        if (particle.position.y > maxY) {
            particle.velocity.y = 0;
            particle.position.y = maxY - 0.001f;
        }
    };

    for (auto &particle : particles) {
        glm::vec3 midWay = particle.position + 0.5f * deltaTime * getVelocityFromGrid(particle.position);
        midWay.x = std::clamp(midWay.x, minX, maxX);
        midWay.y = std::clamp(midWay.y, minY, maxY);

        particle.position += deltaTime * getVelocityFromGrid(midWay);
        clampParticlePosition(particle);
    }
}

template<uint32_t numTilesX, uint32_t numTilesY, uint32_t cellSize, uint32_t numParticles>
std::tuple<float, float, float, float> FlipSolver<numTilesX, numTilesY, cellSize, numParticles>::particleGridWeights(const glm::vec3& pos, glm::ivec2 gridPos, uint32_t xShift, uint32_t yShift){
    auto fSize = float(cellSize);

    float dx;
//    if (gridPos.x < 0) dx = 0;
//    else if (gridPos.x >= numTilesX - 1) dx = fSize;
//    else
        dx = pos.x - float(cellSize * gridPos.x + xShift);

    float dy;
//    if (gridPos.y < 0) dy = 0;
//    else if (gridPos.y >= numTilesY - 1) dy = fSize;
//    else
        dy = pos.y - float(cellSize * gridPos.y + yShift);

    return {
            (1.0f - dx / fSize) * (1.0f - dy / fSize), // down left
            (dx / fSize) * (1.0f - dy / fSize), // down right
            (1.0f - dx / fSize) * (dy / fSize), // up left
            (dx / fSize) * (dy / fSize), // up right
    };
}

template<uint32_t numTilesX, uint32_t numTilesY, uint32_t cellSize, uint32_t numParticles>
void FlipSolver<numTilesX, numTilesY, cellSize, numParticles>::applyWeightedValuesToMatrix(Matrix<float, totalCells, numTilesX> &matrix, glm::ivec2 gridPos,
                                               std::tuple<float, float, float, float> weights, float value) {
    matrix(gridPos) += std::get<0>(weights)*value;
    matrix(gridPos.x + 1, gridPos.y) += std::get<1>(weights)*value;
    matrix(gridPos.x, gridPos.y + 1) += std::get<2>(weights)*value;
    matrix(gridPos.x + 1, gridPos.y + 1) += std::get<3>(weights)*value;
}

template<uint32_t numTilesX, uint32_t numTilesY, uint32_t cellSize, uint32_t numParticles>
void FlipSolver<numTilesX, numTilesY, cellSize, numParticles>::transferParticlesVelocitiesToGrid() {
    previous.velX.swap(current.velX);
    previous.velY.swap(current.velY);
    // reset all cells (to air, and velocities and weights to 0)
    resetGrid();

    auto addVelComponent = [this](Matrix<float, totalCells, numTilesX>& velComponent, Matrix<float, totalCells, numTilesX>& weightComponent, bool isX) {
        for (auto& particle: particles) {
            float vel = particle.velocity.x;
            uint32_t xShift = 0, yShift = cellSize/2;
            if (!isX) {
                vel = particle.velocity.y;
                xShift = cellSize/2;
                yShift = 0;
            }
            glm::ivec2 gPos = gridPos(particle, xShift, yShift);

            // set all cells with particles to fluid
            if (cellTypes(gridPos(particle)) != SOLID) {
                cellTypes(gridPos(particle)) = FLUID;
                hasVel(gridPos(particle)) = 1;
            }


            // transfer velocities to grid
            auto weights = particleGridWeights(particle.position, gPos, xShift, yShift);
            applyWeightedValuesToMatrix(velComponent, gPos, weights, vel);
            applyWeightedValuesToMatrix(weightComponent, gPos, weights, 1.0f);

        }
    };

    addVelComponent(current.velX, weightVelX, true);
    addVelComponent(current.velY, weightVelY, false);


}

template<uint32_t numTilesX, uint32_t numTilesY, uint32_t cellSize, uint32_t numParticles>
void FlipSolver<numTilesX, numTilesY, cellSize, numParticles>::applyWeightsAndGravity() {
    for (uint32_t j = 1; j < numTilesY-1; j++) {
        for (uint32_t i = 1; i < numTilesX-1; i++) {
            if (weightVelY(i, j) > 0.0001) {
                current.velY(i, j) /= weightVelY(i, j);
            }
            if (weightVelX(i, j) > 0.0001) {
                current.velX(i, j) /= weightVelX(i, j);
            }
            // apply gravity
//            if (cellTypes(i, j) == FLUID)
            current.velY(i, j) -= 9.8f * dt * 500.0f;
        }
    }
}

template<uint32_t numTilesX, uint32_t numTilesY, uint32_t cellSize, uint32_t numParticles>
void FlipSolver<numTilesX, numTilesY, cellSize, numParticles>::enforceBoundaryConditions() {

    for (uint32_t j = 0; j < numTilesY; j++) {
        cellTypes(0, j) = cellTypes(numTilesX-1, j) = SOLID;
        current.velX(0, j) = current.velX(1, j) = current.velX(numTilesX - 1, j) = current.velX(numTilesX - 2, j) = 0.0f;
//        current.velY(0, j) = current.velY(1, j) = current.velY(numTilesX - 1, j) = current.velY(numTilesX - 2, j) = 0.0f;
    }
    for (uint32_t i = 0; i < numTilesX; i++) {
        cellTypes(i, 0) = cellTypes(i, numTilesY-1) = SOLID;
//        current.velX(i, 0) = current.velX(i, 1) = 0;//current.velX(i, numTilesY-1) = current.velX(i, numTilesY-2) = 0.0f;
        current.velY(i, 0) = current.velY(i, 1) = current.velY(i, numTilesY-1) = current.velY(i, numTilesY-2) = 0.0f;
    }

}

template<uint32_t numTilesX, uint32_t numTilesY, uint32_t cellSize, uint32_t numParticles>
void FlipSolver<numTilesX, numTilesY, cellSize, numParticles>::extendVelocities() {
//    for (uint32_t j = 1; j < numTilesY; j++) {
//        for (uint32_t i = 1; i < numTilesX; i++) {
//            if (cellTypes(i, j) != FLUID && cellTypes(i-1, j) != FLUID){
//                current.velX(i, j) = 0.0f;
//            }
//            if (cellTypes(i, j) != FLUID && cellTypes(i, j-1) != FLUID){
//                current.velY(i, j) = 0.0f;
//            }
//            previous.velX(i, j) = current.velX(i, j);
//            previous.velY(i, j) = current.velY(i, j);
//        }
//    }

    auto expandVelComponentCell = [this](uint32_t i, uint32_t j, uint32_t k,
                                         Matrix<float, totalCells, numTilesX>& cur){
        uint32_t validNeighbors = 0;
        cur(i, j) = 0;
        if (hasVel(i-1, j) == k) {
            cur(i, j) += cur(i-1, j);
            validNeighbors++;
        }
        if (hasVel(i+1, j) == k) {
            cur(i, j) += cur(i+1, j);
            validNeighbors++;
        }
        if (hasVel(i, j-1) == k) {
            cur(i, j) += cur(i, j-1);
            validNeighbors++;
        }
        if (hasVel(i, j+1) == k) {
            cur(i, j) += cur(i, j+1);
            validNeighbors++;
        }

        if (validNeighbors > 0) {
            cur(i, j) /= float(validNeighbors);
            hasVel(i, j) = k+1;
        }
    };

    for (uint32_t k = 1; k < extensions+1; k++) {
//        current.velX.swap(previous.velX);
//        current.velY.swap(previous.velY);
        for (uint32_t j = 1; j < numTilesY-1; j++) {
            for (uint32_t i = 1; i < numTilesX-1; i++) {
                uint32_t hadVel = hasVel(i, j);
                if (cellTypes(i, j) != SOLID && cellTypes(i-1, j) != SOLID && cellTypes(i, j-1) != SOLID && hadVel == 0){
                    expandVelComponentCell(i, j, k, current.velX);
                    expandVelComponentCell(i, j, k, current.velY);
                }
            }
        }
    }
}

template<uint32_t numTilesX, uint32_t numTilesY, uint32_t cellSize, uint32_t numParticles>
void FlipSolver<numTilesX, numTilesY, cellSize, numParticles>::projectVelocities(float deltaTime) {

    fluidCells = 0;
    for (uint32_t i = 0; i < totalCells; i++) {
        if (cellTypes[i] == FLUID) fluidCells++;
//        rhs[i] = 0;
//        pressure[i] = 0;
        previous.velX[i] = current.velX[i];
        previous.velY[i] = current.velY[i];
    }

    // generate matrix
    SparseMatrix<double, numTilesX> A{fluidCells, totalCells};
    for (uint32_t j = 1; j < numTilesY-1; j++) {
        for (uint32_t i = 1; i < numTilesX-1; i++) {
            if (cellTypes(i, j) != FLUID) continue;

            // sum non-solid cells around
            float sum = solidCells(i - 1, j) + solidCells(i + 1, j) + solidCells(i, j - 1) + solidCells(i, j + 1);
            if (sum == 0.0f) continue;

            // setup matrix A of the system A*p = rhs
            A.set(i + numTilesX*j, i + numTilesX*j, sum);
            if (cellTypes(i + 1, j) == FLUID) A.set(i, j, 3, -1);
            if (cellTypes(i - 1, j) == FLUID) A.set(i, j, 1, -1);
            if (cellTypes(i,j - 1) == FLUID) A.set(i, j, 2, -1);
            if (cellTypes(i,j + 1) == FLUID) A.set(i, j, 4, -1);

            // calculate rhs
            rhs(i, j) = (current.velX(i + 1, j) - current.velX(i, j) + current.velY(i, j + 1) - current.velY(i, j));
        }
    }


    // solve system with CG
    auto res = pressureSolver.solve(A, rhs, pressure, numIterations, 1e-6);
    if (res >= 1) {
        std::cout << "Num iteration exceeded!!\n";
    }
//    else {
//        std::cout << "Converged!!\n";
//    }


    for (uint32_t j = 1; j < numTilesY - 1; j++) {
        for (uint32_t i = 1; i < numTilesX - 1; i++) {
            if (pressure(i, j) != pressure(i, j)) {
                std::cout << "nan pressure" << std::endl;
                continue;
            }
            if (i > 1 && i < numTilesX - 1 && (cellTypes(i, j) == FLUID || cellTypes(i-1, j) == FLUID))
                current.velX(i, j) += float(pressure(i, j) - pressure(i - 1, j));
            if (j > 1 && j < numTilesY - 1 && (cellTypes(i, j) == FLUID || cellTypes(i, j-1) == FLUID))
                current.velY(i, j) += float(pressure(i, j) - pressure(i, j - 1));
        }
    }

}

template<uint32_t numTilesX, uint32_t numTilesY, uint32_t cellSize, uint32_t numParticles>
void FlipSolver<numTilesX, numTilesY, cellSize, numParticles>::transferGridVelocitiesToParticles(float flipRatio) {


    auto addVelComponent = [this, flipRatio](Matrix<float, totalCells, numTilesX>& velComponent,
            Matrix<float, totalCells, numTilesX>& prevVelComponent, bool isX) {
        for (auto& particle: particles) {
            float vel = particle.velocity.x;
            uint32_t xShift = 0, yShift = cellSize/2;
            if (!isX) {
                vel = particle.velocity.y;
                xShift = cellSize/2;
                yShift = 0;
            }
            glm::ivec2 gPos = gridPos(particle, xShift, yShift);
            auto [wdl, wdr, wul, wur] = particleGridWeights(particle.position, gPos, xShift, yShift);

            auto vdl = velComponent(gPos);
            auto vdr = velComponent(gPos.x + 1, gPos.y);
            auto vul = velComponent(gPos.x, gPos.y + 1);
            auto vur = velComponent(gPos.x + 1, gPos.y + 1);

            float picContribution = wdl*vdl + wdr*vdr + wul*vul + wur*vur;
            float correction = (wdl*(vdl - prevVelComponent(gPos)) +
                                wdr*(vdr - prevVelComponent(gPos.x + 1, gPos.y)) +
                                wul*(vul - prevVelComponent(gPos.x, gPos.y + 1)) +
                                wur*(vur - prevVelComponent(gPos.x + 1, gPos.y + 1)));
            float flipContribution = vel + correction;

            if (isX){
                particle.velocity.x = (1 - flipRatio) * picContribution + flipRatio * flipContribution;
//                particle.velocity.x = picContribution;
            }
            else {
                particle.velocity.y = (1 - flipRatio) * picContribution + flipRatio * flipContribution;
//                particle.velocity.y = picContribution;
            }


        }

    };
    addVelComponent(current.velX, previous.velX, true);
    addVelComponent(current.velY, previous.velY, false);

}


#endif //VULKANFLUIDSIMULATION_FLIPSOLVER_H
