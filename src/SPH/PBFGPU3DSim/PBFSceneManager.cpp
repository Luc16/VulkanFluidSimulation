//
// Created by luc on 30/05/24.
//

#include "PBFSceneManager.h"
#include "PBFGPU3DSim.h"


void PBFSceneManager::loadScene(PBFGPU3DSim& sim, const std::string &fileName) {
    vkDeviceWaitIdle(sim.device.device());
    nlohmann::json jsonData;
    std::ifstream fileData(fileName);
    fileData >> jsonData;

    sim.NUM_PARTICLES = jsonData["NUM_PARTICLES"];
    sim.substeps = jsonData["substeps"];
    sim.gaussPartition = jsonData["gaussSteps"];
    sim.sourceParticleSum = jsonData["sourceParticleSum"];
    sim.initialParticles = jsonData["initialParticles"];

    // rendering param
    sim.gUbo.radius = jsonData["rendering"]["radius"];
    sim.gUbo.renderType = jsonData["rendering"]["renderType"];
    sim.gUbo.blurMode = jsonData["rendering"]["blurMode"];
    sim.blurIterations = jsonData["rendering"]["blurIterations"];
    sim.gUbo.filterRadius = jsonData["rendering"]["filterRadius"];
    sim.gUbo.blurScale = jsonData["rendering"]["blurScale"];
    sim.gUbo.blurDepthFalloff = jsonData["rendering"]["blurDepthFalloff"];

    // simulation params
    sim.jacobiIterations = jsonData["simulation"]["jacobiIterations"];
    sim.cUbo.BOUNDARY_SIZE[0] = jsonData["simulation"]["BOUNDARY_SIZE"][0];
    sim.cUbo.BOUNDARY_SIZE[1] = jsonData["simulation"]["BOUNDARY_SIZE"][1];
    sim.cUbo.BOUNDARY_SIZE[2] = jsonData["simulation"]["BOUNDARY_SIZE"][2];
    sim.cUbo.REST_DENS = jsonData["simulation"]["REST_DENS"];
    sim.cUbo.VISC = jsonData["simulation"]["VISC"];
    sim.cUbo.ART_PRESSURE_COEF = jsonData["simulation"]["ART_PRESSURE_COEF"];
    sim.cUbo.VORTICITY_COEF = jsonData["simulation"]["VORTICITY_COEF"];
    sim.cUbo.CFM = jsonData["simulation"]["CFM"];
    sim.cUbo.activateVort = jsonData["simulation"]["activateVort"];
    sim.cUbo.numParticles = jsonData["simulation"]["numParticles"];
    sim.activateVisc = jsonData["simulation"]["activateVisc"];
    sim.activateVorticity = jsonData["simulation"]["activateVorticity"];

    // walls
    sim.wallForwardSpeed = jsonData["walls"]["wallForwardSpeed"];
    sim.wallBackwardSpeed = jsonData["walls"]["wallBackwardSpeed"];
    sim.wallLimit = jsonData["walls"]["wallLimit"];

    sim.NUM_RIGID_PARTICLES = 0;
    sim.particles.resize(sim.NUM_PARTICLES);
    for (uint32_t i = 0; i < sim.NUM_PARTICLES; i++){
        sim.particles.position[i] = glm::vec4(jsonData["particles"][i][0], jsonData["particles"][i][1], jsonData["particles"][i][2], 0.0f);
        sim.particles.type[i] = static_cast<uint32_t>(jsonData["particles"][i][3]);
        if (sim.particles.type[i] == 1) {
            sim.initialParticles++;
            sim.NUM_RIGID_PARTICLES++;
        }
        sim.particles.velocity[i] = glm::vec4(jsonData["particles"][i][4], jsonData["particles"][i][5], jsonData["particles"][i][6], 0.0f);
    }


    sim.rigidObjects.clear();
    sim.rigidObjectsNames.clear();
    if (jsonData["rigid objects"] != nullptr) {
        for (auto & obj : jsonData["rigid objects"]) {
            std::string name = obj[0];
            sim.rigidObjectsNames.emplace_back(name);

            sim.rigidObjects.emplace_back(sim.device, "../Models/" + name + ".obj",
                                          sim.rockTex, obj[2], sim.cUbo.H / 2);
            sim.rigidObjects[sim.rigidObjects.size()-1].translate(glm::vec3(obj[1][0],obj[1][1],obj[1][2]));
        }
    }
}

void PBFSceneManager::saveScene(PBFGPU3DSim& sim, const std::string &fileName) {
    nlohmann::json saveData = {
            {"NUM_PARTICLES", sim.NUM_PARTICLES},
            {"substeps", sim.substeps},
            {"gaussSteps", sim.gaussPartition},
            {"sourceParticleSum", sim.sourceParticleSum},
            {"initialParticles", sim.initialParticles},

            {"rendering",             {
                                        {"radius", sim.gUbo.radius},
                                        {"renderType", sim.gUbo.renderType},
                                        {"blurMode", sim.gUbo.blurMode},
                                        {"blurIterations",   sim.blurIterations},
                                        {"filterRadius", sim.gUbo.filterRadius},
                                        {"blurScale", sim.gUbo.blurScale},
                                        {"blurDepthFalloff", sim.gUbo.blurDepthFalloff},
                                },
            },
            {"simulation", {
                                        {"jacobiIterations", sim.jacobiIterations},
                                        {"BOUNDARY_SIZE", {sim.cUbo.BOUNDARY_SIZE.x, sim.cUbo.BOUNDARY_SIZE.y, sim.cUbo.BOUNDARY_SIZE.z}},
                                        {"REST_DENS", sim.cUbo.REST_DENS},
                                        {"VISC", sim.cUbo.VISC},
                                        {"ART_PRESSURE_COEF", sim.cUbo.ART_PRESSURE_COEF},
                                        {"VORTICITY_COEF", sim.cUbo.VORTICITY_COEF},
                                        {"CFM", sim.cUbo.CFM},
                                        {"activateVort", sim.cUbo.activateVort},
                                        {"numParticles", sim.cUbo.numParticles},
                                        {"activateVisc", sim.activateVisc},
                                        {"activateVorticity", sim.activateVorticity},
                                },
            },
            {"walls", {
                                        {"wallForwardSpeed", sim.wallForwardSpeed},
                                        {"wallBackwardSpeed", sim.wallBackwardSpeed},
                                        {"wallLimit", sim.wallLimit},
                                },
            },
            {"particles", {}},
            {"rigid objects", {}}
    };

    for (uint32_t i = 0; i < sim.NUM_PARTICLES; i++) {
        glm::vec3 pos = sim.particles.position[i];
        uint32_t type = sim.particles.type[i];
        glm::vec3 vel = sim.particles.velocity[i];
        saveData["particles"].emplace_back(std::vector<float>{pos.x, pos.y, pos.z, static_cast<float>(type), vel.x, vel.y, vel.z});
    }

    for (uint32_t i = 0; i < sim.rigidObjects.size(); i++) {
        glm::vec3 pos = sim.rigidObjects[i].getTranslation();
        saveData["rigid objects"][i].emplace_back(sim.rigidObjectsNames[i]);
        saveData["rigid objects"][i].emplace_back(std::vector<float>{pos.x, pos.y, pos.z});
        saveData["rigid objects"][i].emplace_back(sim.rigidObjects[i].getScale());
    }

    std::ofstream o(sim.PRESET_DIR + fileName);
    if (!o.is_open()) {
        std::cerr << "Could not open file " << sim.PRESET_DIR + fileName << " for writing" << std::endl;
        return;
    }
    o << saveData << std::endl;
    sim.presets.emplace_back(fileName);
//    std::cout << "Scene saved to " << fileName << std::endl;
}