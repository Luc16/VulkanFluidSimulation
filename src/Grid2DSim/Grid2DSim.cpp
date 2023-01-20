//
// Created by luc on 30/12/22.
//

#include "Grid2DSim.h"
#include <execution>

void Grid2DSim::FluidData::resize(size_t newSize, uint32_t row) {
    density.resize(newSize, row);
    velX.resize(newSize, row);
    velY.resize(newSize, row);
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
        instanceSystem.createPipeline(renderer.renderPass(), instanceShaderPaths, [this](vkb::Pipeline::PipelineConfigInfo& info) {
            info.bindingDescription.push_back(grid.getBindingDescription());
            info.attributeDescription.push_back({4, 1, VK_FORMAT_R32G32B32_SFLOAT, offsetof(InstanceData, position)});
            info.attributeDescription.push_back({5, 1, VK_FORMAT_R32G32B32_SFLOAT, offsetof(InstanceData, color)});
            info.attributeDescription.push_back({6, 1, VK_FORMAT_R32_SFLOAT, offsetof(InstanceData, scale)});
        });
    }

}

void Grid2DSim::initializeObjects() {
    camera.setViewTarget({0.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 0.0f }, {0.0f, 1.0f, 0.0f});

    createInstances();
}

void Grid2DSim::createInstances() {
    vkDeviceWaitIdle(device.device());

    numTilesX = window.width()/SIZE;
    numTilesMiddle = numTilesX - 2;
    numTilesY = window.height()/SIZE;
    INSTANCE_COUNT = numTilesX*numTilesY;

    grid.resizeBuffer(INSTANCE_COUNT);
    iter.resize(INSTANCE_COUNT);
    curState.resize(INSTANCE_COUNT, numTilesX);
    prevState.resize(INSTANCE_COUNT, numTilesX);

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
        tile.scale = size;
        tile.position = accPos;
        accPos.x += size;

        iter[i] = i;
        if (accPos.x + size > (float) screenExtent.width) {
            accPos.y += size;
            accPos.x = 0.0f;
        }
    }
    grid.updateBuffer();
}

void Grid2DSim::createUniformBuffers() {
    VkDeviceSize bufferSize = sizeof(UniformBufferObject);
    uniformBuffers.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (size_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; ++i) {
        uniformBuffers[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    }
}

void Grid2DSim::mainLoop(float deltaTime) {
    auto currentTime = std::chrono::high_resolution_clock::now();

    camera.updateView();
    updateGrid(deltaTime);
    updateUniformBuffer(renderer.currentFrame(), deltaTime);

    if (activateTimer) {
        auto time = std::chrono::high_resolution_clock::now();
        cpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(time - currentTime).count();
        currentTime = time;
    }

    renderer.runFrame([this](VkCommandBuffer commandBuffer){
        showImGui();

        renderer.runRenderPass([this](VkCommandBuffer& commandBuffer){

            instanceSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);
            grid.render(instanceSystem, commandBuffer);
        });
    });
    if (activateTimer) gpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(std::chrono::high_resolution_clock::now() - currentTime).count();
}


void Grid2DSim::updateGrid(float deltaTime) {
    updateVelocities(deltaTime);
    updateDensities(deltaTime);

    for (uint32_t i = 0; i < grid.size(); i++){
        curState.density[i] = std::clamp(curState.density[i] - dissolveFactor, 0.0f, 1.0f);
        grid[i].color = glm::vec3(curState.density[i]);
    }
    grid.updateBuffer();

}

void Grid2DSim::onResize(int width, int height) {
    INSTANCE_COUNT = (width/SIZE)*(height/SIZE);
    createInstances();
}

void Grid2DSim::updateUniformBuffer(uint32_t frameIndex, float deltaTime){

    UniformBufferObject ubo{};
    auto extent = window.extent();
    camera.setOrthographicProjection(0.0f, (float) extent.width, (float) extent.height, 0.0f, 0.1f, 1000.f);
    ubo.view = camera.getView();
    ubo.proj = camera.getProjection();
    uniformBuffers[frameIndex]->singleWrite(&ubo);

}

void Grid2DSim::showImGui(){

    ImGui::Begin("Control Panel");

    ImGui::Text("Rendering %d instances", INSTANCE_COUNT);
    ImGui::Checkbox("Display time", &activateTimer);
    if (activateTimer){
        ImGui::Text("Gpu time: %f ms", gpuTime);
        ImGui::Text("Cpu time: %f ms", cpuTime);
    }

    ImGui::DragFloat("Diffusion Factor: ", &diffusionFactor, 0.0001f, 0.0f, 5.0f, "%.5f");
    ImGui::DragFloat("Viscosity: ", &viscosity, 0.0001f, 0.0f, 5.0f, "%.5f");
    ImGui::DragFloat("Dissolve Factor: ", &dissolveFactor, 0.00001f, 0.0f, 0.001, "%.5f");

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

void Grid2DSim::updateDensities(float deltaTime) {
    curState.density(numTilesX/2, numTilesY/2) += randomFloat(0.3f, 0.6f);
    curState.density(numTilesX/2 + 1, numTilesY/2) += randomFloat(0.3f, 0.6f);
    curState.density(numTilesX/2, numTilesY/2 + 1) += randomFloat(0.3f, 0.6f);
    curState.density(numTilesX/2 + 1, numTilesY/2 + 1) += randomFloat(0.3f, 0.6f);

    curState.density.swap(prevState.density);
    diffuse(curState.density, prevState.density, diffusionFactor, deltaTime);
    curState.density.swap(prevState.density);
    advect(curState.density, prevState.density, curState.velX, curState.velY, deltaTime);
}

void Grid2DSim::updateVelocities(float deltaTime) {
    double x, y;
    glfwGetCursorPos(window.window(), &x, &y);
    y = window.height() - y;

    float centerX = (float) window.width() / 2;
    float centerY = (float) window.height() / 2;

    float speed = 500.0f;
    auto vel = speed*deltaTime*glm::normalize(glm::vec2((float) x - centerX, (float) y - centerY));

    curState.velX(numTilesX/2, numTilesY/2) = vel.x;
    curState.velY(numTilesX/2, numTilesY/2) = vel.y;
    curState.velX(numTilesX/2 + 1, numTilesY/2) = vel.x;
    curState.velY(numTilesX/2 + 1, numTilesY/2) = vel.y;
    curState.velX(numTilesX/2, numTilesY/2 + 1) = vel.x;
    curState.velY(numTilesX/2, numTilesY/2 + 1) = vel.y;
    curState.velX(numTilesX/2 + 1, numTilesY/2 + 1) = vel.x;
    curState.velY(numTilesX/2 + 1, numTilesY/2 + 1) = vel.y;


    curState.velX.swap(prevState.velX);
    curState.velY.swap(prevState.velY);
    diffuse(curState.velX, prevState.velX, viscosity, deltaTime, 1);
    diffuse(curState.velY, prevState.velY, viscosity, deltaTime, 2);

    project(curState.velX, curState.velY, prevState.velX, prevState.velY);

    curState.velX.swap(prevState.velX);
    curState.velY.swap(prevState.velY);
    advect(curState.velX, prevState.velX, prevState.velX, prevState.velY, deltaTime, 1);
    advect(curState.velY, prevState.velY, prevState.velX, prevState.velY, deltaTime, 2);

    project(curState.velX, curState.velY, prevState.velX, prevState.velY);
}

void Grid2DSim::diffuse(Matrix<float>& x, const Matrix<float>& x0, float diff, float dt, int b){
    int i, j, k;
    float a = dt*diff* (float) (numTilesMiddle*numTilesMiddle);
    for (k = 0; k < 20; ++k) {
        for (i = 1; i <= numTilesMiddle; ++i) {
            for (j = 1; j <= numTilesMiddle; ++j) {
                x(i,j) = (x0(i,j) + a*(x(i-1,j) + x(i+1,j) + x(i,j-1) + x(i,j+1)))/(1+4*a);
            }
        }
        setBounds(x, b);
    }
}

void Grid2DSim::advect(Matrix<float>& d, const Matrix<float>& d0, const Matrix<float>& velX, const Matrix<float>& velY, float dt, int b){
    int i, j, i0, j0, i1, j1;
    float x, y, s0, t0, s1, t1, dt0, fN = (float) numTilesMiddle;
    dt0 = dt*fN;

    for (i = 1; i <= numTilesMiddle; ++i) {
        for (j = 1; j <= numTilesMiddle; ++j) {
            x = (float) i - dt0*velX(i, j);
            if (x < 0.5f) x = 0.5f;
            if (x > fN + 0.5f) x = fN + 0.5f;
            y = (float) j - dt0*velY(i, j);
            if (y < 0.5f) y = 0.5f;
            if (y > fN + 0.5f) y = fN + 0.5f;
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
    setBounds(d, b);
}

void Grid2DSim::project(Matrix<float>& velX, Matrix<float>& velY, Matrix<float>& div, Matrix<float>& p){
    float h = 1/(float) numTilesMiddle;

    for (int j = 1; j <= numTilesMiddle; ++j) {
        for (int i = 1; i <= numTilesMiddle; ++i) {
            div(i, j) = -0.5f*h*(velX(i + 1, j) - velX(i - 1, j) + velY(i, j + 1) - velY(i, j - 1));
            p(i, j) = 0;
        }
    }
    setBounds(div);
    setBounds(p);


    for (int k = 0; k < 20; ++k) {
        for (int j = 1; j <= numTilesMiddle; ++j) {
            for (int i = 1; i <= numTilesMiddle; ++i) {
                p(i,j) = (div(i,j) + p(i + 1,j) + p(i - 1,j) + p(i,j + 1) + p(i,j - 1))/4;
            }
        }
        setBounds(p);
    }

    for (int j = 1; j <= numTilesMiddle; ++j) {
        for (int i = 1; i <= numTilesMiddle; ++i) {
            velX(i,j) -= 0.5f*(p(i + 1,j) - p(i - 1,j))/h;
            velY(i,j) -= 0.5f*(p(i,j + 1) - p(i,j - 1))/h;
        }
    }
    setBounds(velX, 1);
    setBounds(velY, 2);

}

void Grid2DSim::setBounds(Matrix<float>& x, int b) const {
    for (int i = 1; i<= numTilesMiddle; ++i) {
        x(0, i) = b==1 ? -x(1, i) : x(1, i);
        x(numTilesMiddle+1, i) = b==1 ? -x(numTilesMiddle, i) : x(numTilesMiddle, i);
        x(i, 0 ) = b==2 ? -x(i, 1) : x(i, 1);
        x(i, numTilesMiddle+1) = b==2 ? -x(i, numTilesMiddle) : x(i, numTilesMiddle);
    }
    x(0, 0) = 0.5f*(x(1, 0)+x(0, 1));
    x(0, numTilesMiddle+1) = 0.5f*(x(1, numTilesMiddle+1)+x(0, numTilesMiddle));
    x(numTilesMiddle+1, 0) = 0.5f*(x(numTilesMiddle, 0)+x(numTilesMiddle+1, 1));
    x(numTilesMiddle+1, numTilesMiddle+1) = 0.5f*(x(numTilesMiddle, numTilesMiddle+1)+x(numTilesMiddle+1, numTilesMiddle));
}

