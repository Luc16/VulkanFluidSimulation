//
// Created by luc on 30/05/24.
//

#ifndef VULKANFLUIDSIMULATION_PBFSCENEMANAGER_H
#define VULKANFLUIDSIMULATION_PBFSCENEMANAGER_H

#include "../../../external/nlohmann_json/json.hpp"
#include "../../lib/utils.h"
class PBFGPU3DSim;
class PBFSceneManager {

public:
    static void loadScene(PBFGPU3DSim& sim, const std::string& scenePath);
    static void saveScene(PBFGPU3DSim& sim, const std::string& scenePath);

};


#endif //VULKANFLUIDSIMULATION_PBFSCENEMANAGER_H
