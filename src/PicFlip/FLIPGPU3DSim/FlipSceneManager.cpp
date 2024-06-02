//
// Created by luc on 01/06/24.
//

#include <fstream>
#include "FlipSceneManager.h"
#include "FlipSolver.h"


void FlipSceneManager::loadScene(FlipSolver& sim, std::vector<RigidObject>& rigidObjs, const std::string &fileName) {
    vkDeviceWaitIdle(sim.m_deviceRef.device());
    nlohmann::json jsonData;
    std::ifstream fileData(fileName);
    fileData >> jsonData;

    sim.m_numIterations = jsonData["numIterations"];
    sim.m_maxParticles = jsonData["maxParticles"];
    sim.m_particlesToAdd = jsonData["particlesToAdd"];
    sim.extensions = jsonData["extensions"];

    sim.m_cUbo.numParticles = jsonData["NUM_PARTICLES"];
    sim.m_boxSize = sim.m_cUbo.boxSize = glm::vec3(jsonData["boxSize"][0], jsonData["boxSize"][1], jsonData["boxSize"][2]);
    sim.m_cUbo.cellSize = jsonData["cellSize"];
    sim.m_cUbo.flipRatio = jsonData["flipRatio"];
    sim.m_cUbo.w = jsonData["w"];

    sim.m_cUbo.overCellSize = 1/sim.m_cUbo.cellSize;
    sim.m_cUbo.dim = glm::vec<3, uint32_t>(sim.m_boxSize/sim.m_cUbo.cellSize);
    sim.m_cUbo.size = sim.m_cUbo.dim.x*sim.m_cUbo.dim.y*sim.m_cUbo.dim.z;

    // walls
    sim.wallForwardSpeed = jsonData["walls"]["wallForwardSpeed"];
    sim.wallBackwardSpeed = jsonData["walls"]["wallBackwardSpeed"];
    sim.wallLimit = jsonData["walls"]["wallLimit"];

    sim.m_particleData.resize(sim.m_maxParticles);
    for (uint32_t i = 0; i < sim.m_maxParticles; i++){
        sim.m_particleData.positions[i] = glm::vec4(jsonData["particles"][i][0], jsonData["particles"][i][1], jsonData["particles"][i][2], 0.0f);
        sim.m_particleData.velocities[i] = glm::vec4(jsonData["particles"][i][3], jsonData["particles"][i][4], jsonData["particles"][i][5], 0.0f);
    }

    rigidObjs.clear();
    if (jsonData["rigid objects"] != nullptr) {
        for (auto & obj : jsonData["rigid objects"]) {
            std::string name = obj[0];
            rigidObjs.emplace_back(sim.m_deviceRef, name, nullptr, obj[2]);
            rigidObjs[rigidObjs.size()-1].translate(glm::vec3(obj[1][0],obj[1][1],obj[1][2]));
            rigidObjs[rigidObjs.size()-1].createSdf(sim.m_deviceRef, sim.m_cUbo.cellSize, sim.m_cUbo.boxSize);
        }
    }
}

void FlipSceneManager::saveScene(FlipSolver& sim, std::vector<RigidObject>& rigidObjs, const std::string &dir, const std::string &fileName) {
    // save the data from load scene function
    nlohmann::json saveData = {
            {"numIterations", sim.m_numIterations},
            {"maxParticles", sim.m_maxParticles},
            {"particlesToAdd", sim.m_particlesToAdd},
            {"extensions", sim.extensions},
            {"NUM_PARTICLES", sim.m_cUbo.numParticles},
            {"boxSize", {sim.m_boxSize.x, sim.m_boxSize.y, sim.m_boxSize.z}},
            {"cellSize", sim.m_cUbo.cellSize},
            {"flipRatio", sim.m_cUbo.flipRatio},
            {"w", sim.m_cUbo.w},
            {"walls", {
                    {"wallForwardSpeed", sim.wallForwardSpeed},
                    {"wallBackwardSpeed", sim.wallBackwardSpeed},
                    {"wallLimit", sim.wallLimit},
            }},
            {"particles", {}},
            {"rigidObjects", {}}

    };

    for (uint32_t i = 0; i < sim.m_maxParticles; i++){
        saveData["particles"].push_back({
            sim.m_particleData.positions[i].x,
            sim.m_particleData.positions[i].y,
            sim.m_particleData.positions[i].z,
            sim.m_particleData.velocities[i].x,
            sim.m_particleData.velocities[i].y,
            sim.m_particleData.velocities[i].z
        });
    }

    for (uint32_t i = 0; i < rigidObjs.size(); i++) {
        glm::vec3 pos = rigidObjs[i].getTranslation();
        saveData["rigid objects"][i].emplace_back(rigidObjs[i].getModelPath());
        saveData["rigid objects"][i].emplace_back(std::vector<float>{pos.x, pos.y, pos.z});
        saveData["rigid objects"][i].emplace_back(rigidObjs[i].getScale());
    }

    std::ofstream o(dir + fileName);
    if (!o.is_open()) {
        std::cerr << "Could not open file " << dir + fileName << " for writing" << std::endl;
        return;
    }
    o << saveData << std::endl;
    std::cout << "Scene saved to " << fileName << std::endl;
}