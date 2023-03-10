//
// Created by luc on 30/12/22.
//

#include "Grid2DSim.h"
#include <execution>

void Grid2DSim::forEachNeighbor(uint32_t i, uint32_t j, const std::function<void(uint32_t ni, uint32_t nj)>& func) { // does not check if i, j pair is out of bounds
    func(i - 1, j);
    func(i + 1, j);
    func(i, j - 1);
    func(i, j + 1);
}

void Grid2DSim::onCreate() {
    initializeObjects();
    createUniformBuffers();

    // Default render system
    auto defaultDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
            .build();
    defaultDescriptorSets = createDescriptorSets(defaultDescriptorLayout,{uniformBuffers[0]->descriptorInfo()});
    {
        defaultSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), 0);
        defaultSystem.createPipeline(renderer.renderPass(), shaderPaths);
    }

    {
        instanceSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), 0);
        instanceSystem.createPipeline(renderer.renderPass(), instanceShaderPaths, [this](vkb::GraphicsPipeline::PipelineConfigInfo& info) {
            info.bindingDescription.push_back(grid.getBindingDescription());
            info.attributeDescription.push_back({4, 1, VK_FORMAT_R32G32B32_SFLOAT, offsetof(InstanceData, position)});
            info.attributeDescription.push_back({5, 1, VK_FORMAT_R32G32B32_SFLOAT, offsetof(InstanceData, color)});
            info.attributeDescription.push_back({6, 1, VK_FORMAT_R32_SFLOAT, offsetof(InstanceData, scale)});
        });
    }

}

void Grid2DSim::initializeObjects() {
    camera.setViewTarget({0.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 0.0f }, {0.0f, 1.0f, 0.0f});
    camera.updateView();
    auto extent = window.extent();
    camera.setOrthographicProjection(0.0f, (float) extent.width, (float) extent.height, 0.0f, 0.1f, 1000.f);
    ubo.view = camera.getView();
    ubo.proj = camera.getProjection();

    vkDeviceWaitIdle(device.device());

    grid.resizeBuffer(INSTANCE_COUNT);
    iter.resize(INSTANCE_COUNT);

    auto size = (float) SIZE;
    auto accPos = glm::vec3(0.0f, 0.0f, 0.0f);
    auto screenExtent = window.extent();

    for (uint32_t i = 0; i < grid.size(); i++) {
        auto& tile = grid[i];
        tile.color = glm::vec3(0.0f, 0.0f, 0.0f);
        curState.velX[i] = 0.0f;
        curState.velY[i] = 0.0f;
        curState.density[i] = 0.0f;
        prevState.velX[i] = 0.0f;
        prevState.velY[i] = 0.0f;
        prevState.density[i] = 0.0f;
        cellTypes[i] = EMPTY;
        tile.scale = size;
        tile.position = accPos;
        accPos.x += size;

        iter[i] = i;
        if (accPos.x > (float) screenExtent.width) {
            accPos.y += size;
            accPos.x = 0.0f;
        }
    }
    for (int i = 1; i <= numTilesMiddleX; ++i) {
        cellTypes(i, 0) = OUT_BOUNDARY;
        cellTypes(i, numTilesMiddleY+1) = OUT_BOUNDARY;
    }
    for (int i = 1; i <= numTilesMiddleY; ++i) {
        cellTypes(0, i) = OUT_BOUNDARY;
        cellTypes(numTilesMiddleX+1, i) = OUT_BOUNDARY;
    }

    cellTypes(0, 0) = OUT_BOUNDARY;
    cellTypes(0, numTilesMiddleY+1) = OUT_BOUNDARY;
    cellTypes(numTilesMiddleX+1, 0) = OUT_BOUNDARY;
    cellTypes(numTilesMiddleX+1, numTilesMiddleY+1) = OUT_BOUNDARY;


    grid.updateBuffer();
}

void Grid2DSim::createUniformBuffers() {
    VkDeviceSize bufferSize = sizeof(UniformBufferObject);
    uniformBuffers.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (size_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; ++i) {
        uniformBuffers[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        uniformBuffers[i]->map();
    }
}

void Grid2DSim::mainLoop(float deltaTime) {
    auto currentTime = std::chrono::high_resolution_clock::now();

    updateGrid(deltaTime);
    updateUniformBuffer(renderer.currentFrame());

    if (activateTimer) {
        auto time = std::chrono::high_resolution_clock::now();
        cpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(time - currentTime).count();
        currentTime = time;
    }

    renderer.runFrame([this](VkCommandBuffer commandBuffer){
        showImGui();

        renderer.runRenderPass([this](VkCommandBuffer& commandBuffer){
            if (instancingMode){
                instanceSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);
                grid.render(instanceSystem, commandBuffer);
            } else {
                defaultSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);
                grid2D.render(commandBuffer);

            }
        });
    });
    if (activateTimer) gpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(std::chrono::high_resolution_clock::now() - currentTime).count();
}

void Grid2DSim::resetGrid(bool hardReset) {
    for (uint32_t i = 0; i < grid.size(); i++){
        if (hardReset) {
            cellTypes[i] = EMPTY;
            insideBoundaries.clear();
        }
        curState.velX[i] = 0.0f;
        curState.velY[i] = 0.0f;
        curState.density[i] = 0.0f;
        prevState.velX[i] = 0.0f;
        prevState.velY[i] = 0.0f;
        prevState.density[i] = 0.0f;
        grid[i].color = glm::vec3(0.0f);
    }
    grid.updateBuffer();
}

void Grid2DSim::updateUniformBuffer(uint32_t frameIndex) {
    uniformBuffers[frameIndex]->write(&ubo);
}

void Grid2DSim::showImGui(){
    ImGui::Begin("Control Panel");

    ImGui::Text("Rendering %d instances", INSTANCE_COUNT);
    ImGui::Checkbox("Display time", &activateTimer);
    if (activateTimer){
        ImGui::Text("Gpu time: %f ms", gpuTime);
        ImGui::Text("Cpu time: %f ms", cpuTime);
    }

    ImGui::DragFloat("Diffusion Factor", &diffusionFactor, 0.0001f, 0.0f, 5.0f, "%.4f");
    ImGui::DragFloat("Viscosity", &viscosity, 0.0001f, 0.0f, 5.0f, "%.4f");
    ImGui::DragFloat("Dissolve Factor", &dissolveFactor, 0.001f, 0.0f, 2.0f, "%.3f");
    ImGui::DragFloat("Speed", &initialSpeed, 1.0f, 000.0f, 1000.0f, "%.1f");
    if (ImGui::Button("RESET ALL")) resetGrid(true);
    auto prevMode = wallMode;
    ImGui::Checkbox("Activate Wall Drawing Mode", &wallMode);
    ImGui::Checkbox("Activate Instancing Mode", &instancingMode);
    ImGui::Checkbox("Activate Middle Flow", &middleFlow);
    static bool prevSpace = false;
    bool space = glfwGetKey(window.window(), GLFW_KEY_SPACE) == GLFW_PRESS;
    if (space && !prevSpace) {
        wallMode = !wallMode;
    }
    prevSpace = space;
    if (prevMode != wallMode){
        resetGrid();
    }

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                ImGui::GetIO().Framerate);

    if (ImGui::CollapsingHeader("Mouse")) {
        double x, y;
        glfwGetCursorPos(window.window(), &x, &y);
        y = window.height() - y;
        ImGui::Text("Mouse: (%.3lf, %.3f)", x, y);
    }

    ImGui::End();
}

void Grid2DSim::updateGrid(float deltaTime) {
    if (wallMode) {
        createWalls();
    } else {
        addExternalForces(deltaTime);
        updateVelocities(deltaTime);
        updateDensities(deltaTime);
    }

    for (uint32_t i = 0; i < grid.size(); i++){
        if (cellTypes[i] == WALL) {
            grid[i].color = {0.1f, 0.2f, 0.0f};
            grid2D.updateColor(i, {0.1f, 0.2f, 0.0f});
            continue;
        }
        curState.density[i] = std::clamp(curState.density[i] - deltaTime*dissolveFactor, 0.0f, 1.0f);
        grid[i].color = glm::vec3(curState.density[i]);
        grid2D.updateColor(i, glm::vec3(curState.density[i]));

    }

    grid.updateBuffer();
    grid2D.updateBuffer();
}

void Grid2DSim::createWalls() {
    double x, y;
    glfwGetCursorPos(window.window(), &x, &y);
    y = window.height() - y;

    uint32_t idx = (uint32_t) (x/SIZE) + window.width()*((uint32_t) (y/SIZE))/SIZE;
    auto i = (uint32_t) (x/SIZE), j = (uint32_t) (y/SIZE);
    if (idx > 0 && idx < grid.size() && glfwGetMouseButton(window.window(), GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        if (i > 0 && i < numTilesX - 1 && j > 0 && j < numTilesY - 1) {
            if (cellTypes(i, j) == IN_BOUNDARY) insideBoundaries.remove(glm::vec2(i, j));
            cellTypes(i, j) = WALL;
            forEachNeighbor(i, j, [this](uint32_t ni, uint32_t nj){
                if (cellTypes(ni, nj) == EMPTY){
                    cellTypes(ni, nj) = IN_BOUNDARY;
                    insideBoundaries.emplace_front(ni, nj);
                }
            });
        }

    }
}

void Grid2DSim::addExternalForces(float deltaTime) {
    static glm::vec<2, double> prevPos{};
    double x, y;
    glfwGetCursorPos(window.window(), &x, &y);
    y = window.height() - y;

    if (middleFlow) {
        float centerX = (float) window.width() / 2;
        float centerY = (float) window.height() / 2;

        auto vel = initialSpeed*deltaTime*glm::normalize(glm::vec2((float) x - centerX, (float) y - centerY));

        curState.velX(numTilesX/2, numTilesY/2) = vel.x;
        curState.velY(numTilesX/2, numTilesY/2) = vel.y;
        curState.velX(numTilesX/2 + 1, numTilesY/2) = vel.x;
        curState.velY(numTilesX/2 + 1, numTilesY/2) = vel.y;
        curState.velX(numTilesX/2, numTilesY/2 + 1) = vel.x;
        curState.velY(numTilesX/2, numTilesY/2 + 1) = vel.y;
        curState.velX(numTilesX/2 + 1, numTilesY/2 + 1) = vel.x;
        curState.velY(numTilesX/2 + 1, numTilesY/2 + 1) = vel.y;


        curState.density(numTilesX/2, numTilesY/2) += randomFloat(0.3f, 0.6f);
        curState.density(numTilesX/2 + 1, numTilesY/2) += randomFloat(0.3f, 0.6f);
        curState.density(numTilesX/2, numTilesY/2 + 1) += randomFloat(0.3f, 0.6f);
        curState.density(numTilesX/2 + 1, numTilesY/2 + 1) += randomFloat(0.3f, 0.6f);
    } else {

        auto i = (uint32_t) (x/SIZE), j = (uint32_t) (y/SIZE);
        if (x > 0 && x < window.width() && y > 0 && y < window.height() && cellTypes(i, j) == EMPTY) {
            if (glfwGetMouseButton(window.window(), GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
                curState.density(i, j) += 0.6;
            }

            curState.velX(i, j) = initialSpeed*deltaTime*float(x - prevPos.x);
            curState.velY(i, j) = initialSpeed*deltaTime*float(y - prevPos.y);
        }

        prevPos.x = x;
        prevPos.y = y;
    }

}

void Grid2DSim::updateDensities(float deltaTime) {
    curState.density.swap(prevState.density);
    diffuse(curState.density, prevState.density, diffusionFactor, deltaTime);
    curState.density.swap(prevState.density);
    advect(curState.density, prevState.density, curState.velX, curState.velY, deltaTime);
}

void Grid2DSim::updateVelocities(float deltaTime) {
    curState.velX.swap(prevState.velX);
    curState.velY.swap(prevState.velY);
    diffuse(curState.velX, prevState.velX, viscosity, deltaTime);
    diffuse(curState.velY, prevState.velY, viscosity, deltaTime);

    project(curState.velX, curState.velY, prevState.velX, prevState.velY);

    curState.velX.swap(prevState.velX);
    curState.velY.swap(prevState.velY);
    advect(curState.velX, prevState.velX, prevState.velX, prevState.velY, deltaTime, MIRROR_X);
    advect(curState.velY, prevState.velY, prevState.velX, prevState.velY, deltaTime, MIRROR_Y);

    project(curState.velX, curState.velY, prevState.velX, prevState.velY);
}

void Grid2DSim::diffuse(Matrix<float, INSTANCE_COUNT, numTilesX>& x, const Matrix<float, INSTANCE_COUNT, numTilesX>& x0, float diff, float dt){
    float a = dt*diff* (float) (numTilesMiddleY*numTilesMiddleX);
    for (uint32_t k = 0; k < 20; ++k) {
        for (uint32_t j = 1; j <= numTilesMiddleY; ++j) {
            for (uint32_t i = 1; i <= numTilesMiddleX; ++i) {
                if (cellTypes(i, j) == EMPTY) x(i,j) = (x0(i,j) + a*(x(i-1,j) + x(i+1,j) + x(i,j-1) + x(i,j+1)))/(1+4*a);
            }
        }
        diffuseInnerBounds(x, x0, a);
        setBounds(x, REGULAR);
    }
}

void Grid2DSim::advect(Matrix<float, INSTANCE_COUNT, numTilesX>& d, const Matrix<float, INSTANCE_COUNT, numTilesX>& d0, const Matrix<float, INSTANCE_COUNT, numTilesX>& velX, const Matrix<float, INSTANCE_COUNT, numTilesX>& velY, float dt, BoundConfig b){
    int i0, j0, i1, j1;
    float x, y, s0, t0, s1, t1;
    auto fNX = (float) numTilesMiddleX;
    auto fNY = (float) numTilesMiddleY;
    float dt0 = dt*std::min(fNX, fNY);

    for (uint32_t j = 1; j <= numTilesMiddleY; ++j) {
        for (uint32_t i = 1; i <= numTilesMiddleX; ++i) {
            if (cellTypes(i, j) != EMPTY) continue;
            x = (float) i - dt0*velX(i, j);
            if (x < 0.5f) x = 0.5f;
            if (x > fNX + 0.5f) x = fNX + 0.5f;
            y = (float) j - dt0*velY(i, j);
            if (y < 0.5f) y = 0.5f;
            if (y > fNX + 0.5f) y = fNY + 0.5f;
            i0 = (int) x;
            i1 = i0 + 1;
            j0 = (int) y;
            j1 = j0 + 1;
            s1 = x - (float) i0;
            t1 = y - (float) j0;
            s0 = 1 - s1;
            t0 = 1 - t1;

            d(i, j) = s0*(t0*d0(i0, j0) + t1*d0(i0, j1)) + s1*(t0*d0(i1, j0) + t1*d0(i1, j1));
        }
    }
    setInnerBounds(d, b);
    setBounds(d, b);
}

void Grid2DSim::project(Matrix<float, INSTANCE_COUNT, numTilesX> &velX, Matrix<float, INSTANCE_COUNT, numTilesX> &velY, Matrix<float, INSTANCE_COUNT, numTilesX> &div, Matrix<float, INSTANCE_COUNT, numTilesX> &p) {
    float h = 1/(float) std::min(numTilesMiddleX, numTilesMiddleY);

    for (uint32_t j = 1; j <= numTilesMiddleY; ++j) {
        for (uint32_t i = 1; i <= numTilesMiddleX; ++i) {
            if (cellTypes(i, j) != WALL)
                div(i, j) = -0.5f*h*(velX(i + 1, j) - velX(i - 1, j) + velY(i, j + 1) - velY(i, j - 1));
            p(i, j) = 0;
        }
    }
    setBounds(div);
    setBounds(p);


    for (uint32_t k = 0; k < 20; ++k) {
        for (uint32_t j = 1; j <= numTilesMiddleY; ++j) {
            for (uint32_t i = 1; i <= numTilesMiddleX; ++i) {
                if (cellTypes(i, j) != WALL) p(i,j) = (div(i,j) + p(i + 1,j) + p(i - 1,j) + p(i,j + 1) + p(i,j - 1))/4;
            }
        }
        setBounds(p);
        setInnerBounds(p);
    }

    for (uint32_t j = 1; j <= numTilesMiddleY; ++j) {
        for (uint32_t i = 1; i <= numTilesMiddleX; ++i) {
            if (cellTypes(i, j) != EMPTY) continue;
            velX(i,j) -= 0.5f*(p(i + 1,j) - p(i - 1,j))/h;
            velY(i,j) -= 0.5f*(p(i,j + 1) - p(i,j - 1))/h;
        }
    }
    setBounds(velX, MIRROR_X);
    setBounds(velY, MIRROR_Y);
    setInnerBounds(velX, MIRROR_X);
    setInnerBounds(velY, MIRROR_Y);

}

void Grid2DSim::setBounds(Matrix<float, INSTANCE_COUNT, numTilesX>& x, BoundConfig b) {
    for (uint32_t i = 1; i <= numTilesMiddleX; ++i) {
        x(i, 0) = (b == MIRROR_Y) ? -x(i, 1) : x(i, 1);
        x(i, numTilesMiddleY+1) = (b == MIRROR_Y) ? -x(i, numTilesMiddleY) : x(i, numTilesMiddleY);
    }
    for (uint32_t i = 1; i <= numTilesMiddleY; ++i) {
        x(0, i) = (b == MIRROR_X) ? -x(1, i) : x(1, i);
        x(numTilesMiddleX+1, i) = (b == MIRROR_X) ? -x(numTilesMiddleX, i) : x(numTilesMiddleX, i);
    }


    x(0, 0) = 0.5f*(x(1, 0)+x(0, 1));
    x(0, numTilesMiddleY+1) = 0.5f*(x(1, numTilesMiddleY+1)+x(0, numTilesMiddleY));
    x(numTilesMiddleX+1, 0) = 0.5f*(x(numTilesMiddleX, 0)+x(numTilesMiddleX+1, 1));
    x(numTilesMiddleX+1, numTilesMiddleY+1) = 0.5f*(x(numTilesMiddleX, numTilesMiddleY+1)+x(numTilesMiddleX+1, numTilesMiddleY));


}

void Grid2DSim::diffuseInnerBounds(Matrix<float, INSTANCE_COUNT, numTilesX> &x, const Matrix<float, INSTANCE_COUNT, numTilesX> &x0, float a) {
    for (auto& vec: insideBoundaries) {
        float sum = 0.0f, amount = 0.0f;
        forEachNeighbor(vec.x, vec.y, [this, &sum, &amount, &x](uint32_t i, uint32_t j){
            if (cellTypes(i, j) == WALL) return;
            sum += x(i, j);
            amount += 1.0f;
        });
        x(vec) = (x0(vec) + a*(sum))/(1+amount*a);
    }
}

void Grid2DSim::setInnerBounds(Matrix<float, INSTANCE_COUNT, numTilesX>& d, BoundConfig b) {
    for (auto& vec: insideBoundaries) {
        float sum = 0.0f, amount = 0.0f;
        forEachNeighbor(vec.x, vec.y, [this, &sum, &amount, &d](uint32_t i, uint32_t j){
            switch (cellTypes(i, j)) {
                case WALL:
                    break;
                case OUT_BOUNDARY:
                case IN_BOUNDARY:
                    sum += d(i, j);
                    amount += 1.0f;
                    break;
                case EMPTY:
                    sum += 2*d(i, j);
                    amount += 2.0f;
                    break;
            }
        });
        if (amount <= 0.0f) {
            d(vec) = 0.0f;
            continue;
        }
        d(vec) = sum/amount;
        if (b == MIRROR_X) {
            if (cellTypes(vec.x + 1, vec.y) == WALL && d(vec) > 0 || cellTypes(vec.x - 1, vec.y) == WALL && d(vec) < 0) {
                d(vec) = -d(vec);
            }
        }
        else if (b == MIRROR_Y) {
            if (cellTypes(vec.x, vec.y + 1) == WALL && d(vec) > 0 || cellTypes(vec.x, vec.y - 1) == WALL && d(vec) < 0) {
                d(vec) = -d(vec);
            }
        }
    }
}


