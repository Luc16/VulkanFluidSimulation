//
// Created by luc on 25/01/24.
//

#ifndef VULKANFLUIDSIMULATION_FLIPSOLVER_H
#define VULKANFLUIDSIMULATION_FLIPSOLVER_H
#include <sstream>

#include "../lib/utils.h"
#include "../lib/graphicsDataStructures/Matrices.h"
#include "structs.h"
#include "PressureSolver.h"

enum CellType {
    AIR, SOLID, FLUID
};

template<uint32_t numTilesX, uint32_t numTilesY, uint32_t cellSize, uint32_t numParticles>
class FlipSolver {
public:
    FlipSolver(float particleRadius, float flipRatio, double rho, uint32_t numIterations, uint32_t extensions):
            particleRadius(particleRadius), flipRatio(flipRatio), rho(rho), numIterations(numIterations), extensions(extensions) {}

    [[nodiscard]] float getVelX(uint32_t i, uint32_t j) { return current.velX(i, j); }
    [[nodiscard]] float getVelY(uint32_t i, uint32_t j) { return current.velY(i, j); }
    [[nodiscard]] CellType getCellType(uint32_t i, uint32_t j) { return cellTypes(i, j); }
    [[nodiscard]] float getSolidCells(uint32_t i, uint32_t j) { return solidCells(i, j); }

private:
    const static uint32_t totalCells = numTilesX*numTilesY;


    struct FluidData {
        Matrix<float, totalCells, numTilesX> velX{}, velY{};
    };

    [[nodiscard]] glm::ivec2 gridPos(const Particle& particle, uint32_t xShift = 0, uint32_t yShift = 0) const { return {
                (uint32_t(std::clamp(particle.position.x, float(xShift), float(cellSize*(numTilesX - 1)))) - xShift) / cellSize,
                (uint32_t(std::clamp(particle.position.y, float(yShift), float(cellSize*(numTilesY - 1)))) - yShift) / cellSize}; }


    float particleRadius;
    float flipRatio;
    double rho;
    uint32_t numIterations;
    uint32_t extensions;
    float dt = 1/120.0f;

    FluidData current, previous;
    Matrix<CellType, totalCells, numTilesX> cellTypes{};
    Matrix<float, totalCells, numTilesX> weightVelX{}, weightVelY{};
    Matrix<float, totalCells, numTilesX> solidCells{};
    HeapMatrix<double, numTilesX> pressure{totalCells};
    HeapMatrix<double, numTilesX> rhs{totalCells};
    PressureSolver<double, numTilesX> pressureSolver{};

public:
    std::vector<Particle> particles{numParticles};


    void resetGrid(bool hardReset = false);
    void initializeParticles();
    void advectParticles();
    static std::tuple<float, float, float, float> particleGridWeights(const Particle& particle, glm::ivec2 gridPos, uint32_t xShift, uint32_t yShift);
    static void applyWeightedValuesToMatrix(Matrix<float, totalCells, numTilesX>& matrix, glm::ivec2 gridPos, std::tuple<float, float, float, float> weights, float value);
    void transferParticlesVelocitiesToGrid();
    void transferGridVelocitiesToParticles();
    void enforceDirichlet();
    void extendVelocities();
    void projectVelocities(float deltaTime);

    void updateSimulation(float deltaTime);
};

template<uint32_t numTilesX, uint32_t numTilesY, uint32_t cellSize, uint32_t numParticles>
void FlipSolver<numTilesX, numTilesY, cellSize, numParticles>::updateSimulation(float deltaTime) {
    // how it should be:

    // advect particles
    // transfer particles velocities to grid
    // update grid (gravity)
    // project velocities
    // transfer grid velocities to particles
    // add gravity to velY
    advectParticles();
    transferParticlesVelocitiesToGrid();
    enforceDirichlet();
    extendVelocities();
    projectVelocities(deltaTime);
    enforceDirichlet();
    transferGridVelocitiesToParticles();

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
        cellTypes[i] = (solidCells[i] == 0.0f) ? SOLID: AIR;
        weightVelX[i] = 0.0f;
        weightVelY[i] = 0.0f;
    }
}

template<uint32_t numTilesX, uint32_t numTilesY, uint32_t cellSize, uint32_t numParticles>
void FlipSolver<numTilesX, numTilesY, cellSize, numParticles>::initializeParticles() {
    float startingX = 3*float(cellSize*numTilesX)/8;
    auto accPos = glm::vec3(startingX, 100, 0);
    float step = 1.2f*particleRadius;

    for (uint32_t i = 0; i < numParticles; i++) {
        auto& particle = particles[i];
        particle.position = accPos;
        particle.position += glm::vec3(
                randomFloat(0.0f, 0.5f*particleRadius),
                randomFloat(0.0f, 0.5f*particleRadius),
                0.0f
        );
        particle.velocity = glm::vec3(0.0f, 0.0f, 0.0f);
        particle.color = glm::vec4(0.2f, 0.6f, 1.0f, 1.0f);

        accPos.x += step;

        if (accPos.x > (float) 5*float(cellSize*numTilesX)/8) {
            accPos.y += step;
            accPos.x = startingX;
        }
    }
}

template<uint32_t numTilesX, uint32_t numTilesY, uint32_t cellSize, uint32_t numParticles>
void FlipSolver<numTilesX, numTilesY, cellSize, numParticles>::advectParticles() {
    static constexpr float BOUND_DAMPING = 0.f;
    float minX = float(cellSize) + particleRadius, maxX = float(cellSize*(numTilesX-1)) - particleRadius;
    float minY = float(cellSize) + particleRadius, maxY = float(cellSize*(numTilesY-1)) - particleRadius;

    for (auto &particle : particles) {
        particle.position += dt * particle.velocity;

        if (particle.position.x < minX) {
            particle.velocity.x *= BOUND_DAMPING;
            particle.position.x = minX;
        }
        if (particle.position.x > maxX) {
            particle.velocity.x *= BOUND_DAMPING;
            particle.position.x = maxX;
        }
        if (particle.position.y < minY) {
            particle.velocity.y *= BOUND_DAMPING;
            particle.position.y = minY;
        }
        if (particle.position.y > maxY) {
            particle.velocity.y *= BOUND_DAMPING;
            particle.position.y = maxY;
        }

    }
}

template<uint32_t numTilesX, uint32_t numTilesY, uint32_t cellSize, uint32_t numParticles>
std::tuple<float, float, float, float> FlipSolver<numTilesX, numTilesY, cellSize, numParticles>::particleGridWeights(const Particle& particle, glm::ivec2 gridPos, uint32_t xShift, uint32_t yShift){
    auto fSize = float(cellSize);

    float dx = particle.position.x - float(cellSize * gridPos.x + xShift);
    float dy = particle.position.y - float(cellSize * gridPos.y + yShift);

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
            if (cellTypes(gridPos(particle)) != SOLID) cellTypes(gridPos(particle)) = FLUID;

            // transfer velocities to grid
            auto weights = particleGridWeights(particle, gPos, xShift, yShift);
            applyWeightedValuesToMatrix(velComponent, gPos, weights, vel);
            applyWeightedValuesToMatrix(weightComponent, gPos, weights, 1.0f);

        }
    };

    addVelComponent(current.velX, weightVelX, true);
    addVelComponent(current.velY, weightVelY, false);

    for (uint32_t j = 1; j < numTilesY-1; j++) {
        for (uint32_t i = 1; i < numTilesX - 1; i++) {
            if (weightVelY(i, j) > 0.0001) {
                current.velY(i, j) /= weightVelY(i, j);
            }
            if (weightVelX(i, j) > 0.0001) {
                current.velX(i, j) /= weightVelX(i, j);
            }
            // apply gravity
//            if (cellTypes(i, j) == FLUID) {
//                current.velY(i, j) -= 9.8f * dt * 500;
//            }
            current.velY(i, j) -= 9.8f * dt * 500;
        }
    }
}

template<uint32_t numTilesX, uint32_t numTilesY, uint32_t cellSize, uint32_t numParticles>
void FlipSolver<numTilesX, numTilesY, cellSize, numParticles>::enforceDirichlet() {
    for (uint32_t j = 0; j < numTilesY; j++) {
        for (uint32_t i = 0; i < numTilesX; i++) {
            if (cellTypes(i, j) == SOLID || i > 0 && cellTypes(i-1, j) == SOLID){
                current.velX(i, j) = 0.0f;
            }
            if (cellTypes(i, j) == SOLID || j > 0 && cellTypes(i, j-1) == SOLID){
                current.velY(i, j) = 0.0f;
            }
        }
    }
}

template<uint32_t numTilesX, uint32_t numTilesY, uint32_t cellSize, uint32_t numParticles>
void FlipSolver<numTilesX, numTilesY, cellSize, numParticles>::extendVelocities() {
    for (uint32_t j = 1; j < numTilesY; j++) {
        for (uint32_t i = 1; i < numTilesX; i++) {
            if (cellTypes(i, j) != FLUID && cellTypes(i-1, j) != FLUID){
                current.velX(i, j) = 0.0f;
            }
            if (cellTypes(i, j) != FLUID && cellTypes(i, j-1) != FLUID){
                current.velY(i, j) = 0.0f;
            }
            previous.velX(i, j) = current.velX(i, j);
            previous.velY(i, j) = current.velY(i, j);
        }
    }
    for (uint32_t _ = 0; _ < extensions; _++) {
        for (uint32_t j = 1; j < numTilesY - 1; j++) {
            for (uint32_t i = 1; i < numTilesX - 1; i++) {
                if (cellTypes(i, j) != SOLID && cellTypes(i-1, j) != SOLID && previous.velX(i, j) == 0.0f){
                    uint32_t validNeighbors = 0;
                    if (std::abs(current.velX(i-1, j)) > 0) {
                        previous.velX(i, j) += previous.velX(i-1, j);
                        validNeighbors++;
                    }
                    if (std::abs(current.velX(i+1, j)) > 0) {
                        previous.velX(i, j) += previous.velX(i+1, j);
                        validNeighbors++;
                    }
                    if (std::abs(current.velX(i, j-1)) > 0) {
                        previous.velX(i, j) += previous.velX(i, j-1);
                        validNeighbors++;
                    }
                    if (std::abs(current.velX(i, j+1)) > 0) {
                        previous.velX(i, j) += previous.velX(i, j+1);
                        validNeighbors++;
                    }

                    if (validNeighbors > 0) {
                        previous.velX(i, j) /= float(validNeighbors);
                    }
                }
                if (cellTypes(i, j) != SOLID || j > 0 && cellTypes(i, j-1) != SOLID && previous.velY(i, j) == 0.0f){
                    uint32_t validNeighbors = 0;
                    if (std::abs(current.velY(i-1, j)) > 0) {
                        previous.velY(i, j) += previous.velY(i-1, j);
                        validNeighbors++;
                    }
                    if (std::abs(current.velY(i+1, j)) > 0) {
                        previous.velY(i, j) += previous.velY(i+1, j);
                        validNeighbors++;
                    }
                    if (std::abs(current.velY(i, j-1)) > 0) {
                        previous.velY(i, j) += previous.velY(i, j-1);
                        validNeighbors++;
                    }
                    if (std::abs(current.velY(i, j+1)) > 0) {
                        previous.velY(i, j) += previous.velY(i, j+1);
                        validNeighbors++;
                    }

                    if (validNeighbors > 0) {
                        previous.velY(i, j) /= float(validNeighbors);
                    }
                }
            }
        }
        previous.velX.swap(current.velX);
        previous.velY.swap(current.velY);
    }
}

template<uint32_t numTilesX, uint32_t numTilesY, uint32_t cellSize, uint32_t numParticles>
void FlipSolver<numTilesX, numTilesY, cellSize, numParticles>::projectVelocities(float deltaTime) {

    size_t fluidCells = 0;
    for (uint32_t i = 0; i < totalCells; i++) {
        if (cellTypes[i] == FLUID) fluidCells++;
        rhs[i] = 0;
        pressure[i] = 0;
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
            if (solidCells(i + 1, j) > 0.0) A.set(i + numTilesX*j, i + 1 + numTilesX*j, -1);
            if (solidCells(i - 1, j) > 0.0) A.set(i + numTilesX*j, i - 1 + numTilesX*j, -1);
            if (solidCells(i,j - 1) > 0.0) A.set(i + numTilesX*j, i + numTilesX*(j-1), -1);
            if (solidCells(i,j + 1) > 0.0) A.set(i + numTilesX*j, i + numTilesX*(j+1), -1);

            if (A(i, j, 0) < 3) {
                std::cout << "A(" << i << ", " << j << ", 0) = " << A(i, j, 0) << std::endl;
            }

            // calculate rhs
            rhs(i, j) = current.velX(i + 1, j) - current.velX(i, j) + current.velY(i, j + 1) - current.velY(i, j);

        }
    }

    // solve system with CG
    auto res = pressureSolver.solve(A, rhs, pressure, numIterations, 1e-6);
//    if (res >= 1) {
//        std::cout << "Num iteration exceeded!!\n";
//    } else {
//        std::cout << "Converged!!\n";
//    }


    double scale = dt / (rho * cellSize);
    for (uint32_t j = 1; j < numTilesY - 1; j++) {
        for (uint32_t i = 1; i < numTilesX - 1; i++) {
            if (cellTypes(i, j) != FLUID) continue;
            if (pressure(i, j) != pressure(i, j)) {
                std::cout << "nan pressure" << std::endl;
                continue;
            }
            if (cellTypes(i-1, j) == FLUID) current.velX(i, j) += float(pressure(i, j) - pressure(i - 1, j));
            if (cellTypes(i, j-1) == FLUID) current.velY(i, j) += float(pressure(i, j) - pressure(i, j - 1));
        }
    }

}

template<uint32_t numTilesX, uint32_t numTilesY, uint32_t cellSize, uint32_t numParticles>
void FlipSolver<numTilesX, numTilesY, cellSize, numParticles>::transferGridVelocitiesToParticles() {

    auto addVelComponent = [this](Matrix<float, totalCells, numTilesX>& velComponent, Matrix<float, totalCells, numTilesX>& prevVelComponent, bool isX) {
        for (auto& particle: particles) {
            float vel = particle.velocity.x;
            uint32_t xShift = 0, yShift = cellSize/2;
            if (!isX) {
                vel = particle.velocity.y;
                xShift = cellSize/2;
                yShift = 0;
            }
            glm::ivec2 gPos = gridPos(particle, xShift, yShift);
            auto [wdl, wdr, wul, wur] = particleGridWeights(particle, gPos, xShift, yShift);

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
            }
            else {
                particle.velocity.y = (1 - flipRatio) * picContribution + flipRatio * flipContribution;
            }


        }

    };
    addVelComponent(current.velX, previous.velX, true);
    addVelComponent(current.velY, previous.velY, false);

}


#endif //VULKANFLUIDSIMULATION_FLIPSOLVER_H
