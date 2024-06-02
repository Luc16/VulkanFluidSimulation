//
// Created by luc on 01/06/24.
//

#ifndef VULKANFLUIDSIMULATION_FLIPSCENEMANAGER_H
#define VULKANFLUIDSIMULATION_FLIPSCENEMANAGER_H

#include "../../../external/nlohmann_json/json.hpp"
#include "../../lib/utils.h"
#include "RigidObject.h"

class FlipSolver;
class FlipSceneManager {

public:
    static void loadScene(FlipSolver& sim, std::vector<RigidObject>& rigidObjs, const std::string& scenePath);
    static void saveScene(FlipSolver& sim, std::vector<RigidObject>& rigidObjs, const std::string &dir, const std::string &fileName);

};



#endif //VULKANFLUIDSIMULATION_FLIPSCENEMANAGER_H
