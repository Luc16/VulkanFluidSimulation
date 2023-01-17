//
// Created by luc on 30/12/22.
//

#include "Grid2DSim.h"
#include <execution>

void Grid2DSim::onCreate() {
    initializeObjects();
    createUniformBuffers();

    // Default render system
    auto defaultDescriptorLayout = vkb::DescriptorSetLayout::Builder(device)
            .addBinding({0, VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER, 1, VK_SHADER_STAGE_ALL_GRAPHICS, nullptr})
            .build();
    defaultDescriptorSets = createDescriptorSets(defaultDescriptorLayout,{uniformBuffers[0]->descriptorInfo()});
    {
        defaultSystem.createPipelineLayout(defaultDescriptorLayout.descriptorSetLayout(), sizeof(vkb::DrawableObject::PushConstantData));
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
    numTilesX = window.width()/SIZE;
    numTilesY = window.height()/SIZE;
    INSTANCE_COUNT = numTilesX*numTilesY;
    camera.setViewTarget({0.0f, 0.0f, -1.0f}, {0.0f, 0.0f, 0.0f }, {0.0f, 1.0f, 0.0f});

    createInstances();
}

void Grid2DSim::createInstances() {
    vkDeviceWaitIdle(device.device());

    grid.resizeBuffer(INSTANCE_COUNT);
    iter.resize(INSTANCE_COUNT);
    dens.resize(INSTANCE_COUNT);
    prevDens.resize(INSTANCE_COUNT);
    velX.resize(INSTANCE_COUNT);
    velY.resize(INSTANCE_COUNT);
    prevVelX.resize(INSTANCE_COUNT);
    prevVelY.resize(INSTANCE_COUNT);

    auto size = (float) SIZE;
    auto accPos = glm::vec3(0.0f, 0.0f, 0.0f);
    auto screenExtent = window.extent();

    for (uint32_t i = 0; i < grid.size(); i++) {
        auto& tile = grid[i];
        tile.color = glm::vec3(0.0f, 0.0f, 0.0f);
        dens[i] = 0;
        prevDens[i] = 0;
        velX[i] = 0;
        velY[i] = 0;
        prevVelX[i] = 0;
        prevVelY[i] = 0;
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
    grid.updateBuffer();

    if (activateTimer) {
        auto time = std::chrono::high_resolution_clock::now();
        cpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(time - currentTime).count();
        currentTime = time;
    }

    renderer.runFrame([this](VkCommandBuffer commandBuffer){
        showImGui();

        renderer.runRenderPass([this](VkCommandBuffer& commandBuffer){

//            defaultSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);

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
        dens[i] = std::clamp(dens[i] - 0.0002f, 0.0f, 1.0f);
        grid[i].color = glm::vec3(dens[i]);
    }

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
    uint32_t N = numTilesX - 2;
    dens[IX(numTilesX/2, numTilesY/2)] += randomFloat(0.2f, 0.4f);
    dens[IX(numTilesX/2 + 1, numTilesY/2)] += randomFloat(0.2f, 0.4f);
    dens[IX(numTilesX/2, numTilesY/2) + 1] += randomFloat(0.2f, 0.4f);
    dens[IX(numTilesX/2 + 1, numTilesY/2) + 1] += randomFloat(0.2f, 0.4f);

    dens.swap(prevDens);
    diffuse(N, 0, dens, prevDens, diffusionFactor, deltaTime);
    dens.swap(prevDens);
    advect(N, 0, dens, prevDens, velX, velY, deltaTime);
}

void Grid2DSim::updateVelocities(float deltaTime) {
    double x, y;
    glfwGetCursorPos(window.window(), &x, &y);
    y = window.height() - y;

    float centerX = (float) window.width() / 2;
    float centerY = (float) window.height() / 2;

    uint32_t N = numTilesX - 2;
    float speed = 500.0f;
    auto vel = speed*deltaTime*glm::normalize(glm::vec2((float) x - centerX, (float) y - centerY));
    velX[IX(numTilesX/2, numTilesY/2)] = vel.x;
    velY[IX(numTilesX/2, numTilesY/2)] = vel.y;
    velX[IX(numTilesX/2 + 1, numTilesY/2)] = vel.x;
    velY[IX(numTilesX/2 + 1, numTilesY/2)] = vel.y;
    velX[IX(numTilesX/2, numTilesY/2) + 1] = vel.x;
    velY[IX(numTilesX/2, numTilesY/2) + 1] = vel.y;
    velX[IX(numTilesX/2 + 1, numTilesY/2) + 1] = vel.x;
    velY[IX(numTilesX/2 + 1, numTilesY/2) + 1] = vel.y;

    velX.swap(prevVelX);
    velY.swap(prevVelY);
    diffuse(N, 1, velX, prevVelX, viscosity, deltaTime);
    diffuse(N, 2, velY, prevVelY, viscosity, deltaTime);

    project(N, velX, velY, prevVelX, prevVelY);

    velX.swap(prevVelX);
    velY.swap(prevVelY);
    advect(N, 1, velX, prevVelX, prevVelX, prevVelY, deltaTime);
    advect(N, 2, velY, prevVelY, prevVelX, prevVelY, deltaTime);

    project(N, velX, velY, prevVelX, prevVelY);
}

void Grid2DSim::diffuse( uint32_t N, int b, std::vector<float>& x, const std::vector<float>& x0, float diff, float dt ){
    int i, j, k;
    float a = dt*diff* (float) (N*N);
    for (k = 0; k < 20; ++k) {
        for (i = 1; i <= N; ++i) {
            for (j = 1; j <= N; ++j) {
                x[IX(i,j)] = (x0[IX(i,j)] + a*(x[IX(i-1,j)] + x[IX(i+1,j)] + x[IX(i,j-1)] + x[IX(i,j+1)]))/(1+4*a);
            }
        }
        setBounds( N, b, x );
    }
}

void Grid2DSim::advect(uint32_t N, int b, std::vector<float>& d, const std::vector<float>& d0, const std::vector<float>& u, const std::vector<float>& v, float dt){
    int i, j, i0, j0, i1, j1;
    float x, y, s0, t0, s1, t1, dt0, fN = (float) N;
    dt0 = dt*fN;

    for (i = 1; i <= N; ++i) {
        for (j = 1; j <= N; ++j) {
           x = (float) i - dt0*u[IX(i, j)];
           if (x < 0.5f) x = 0.5f;
           if (x > fN + 0.5f) x = fN + 0.5f;
           y = (float) j - dt0*v[IX(i, j)];
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

           d[IX(i, j)] = s0*(t0*d0[IX(i0, j0)] + t1*d0[IX(i0, j1)]) + s1*(t0*d0[IX(i1, j0)] + t1*d0[IX(i1, j1)]);
        }
    }
    setBounds(N, b, d);
}

void Grid2DSim::project(uint32_t N, std::vector<float>& u, std::vector<float>& v, std::vector<float>& p, std::vector<float>& div) {
    float h = 1/(float) N;

    for (int j = 1; j <= N; ++j) {
        for (int i = 1; i <= N; ++i) {
            div[IX(i, j)] = -0.5f*h*(u[IX(i + 1, j)] - u[IX(i - 1, j)] + v[IX(i, j + 1)] - v[IX(i, j - 1)]);
            p[IX(i, j)] = 0;
        }
    }
    setBounds(N, 0, div);
    setBounds(N, 0, p);


    for (int k = 0; k < 20; ++k) {
        for (int j = 1; j <= N; ++j) {
            for (int i = 1; i <= N; ++i) {
                p[IX(i,j)] = (div[IX(i,j)] + p[IX(i + 1,j)] + p[IX(i - 1,j)] + p[IX(i,j + 1)] + p[IX(i,j - 1)])/4;
            }
        }
        setBounds(N, 0, p);
    }

    for (int j = 1; j <= N; ++j) {
        for (int i = 1; i <= N; ++i) {
            u[IX(i,j)] -= 0.5f*(p[IX(i + 1,j)] - p[IX(i - 1,j)])/h;
            v[IX(i,j)] -= 0.5f*(p[IX(i,j + 1)] - p[IX(i,j - 1)])/h;
        }
    }
    setBounds(N, 1, u);
    setBounds(N, 2, v);

}

void Grid2DSim::setBounds( uint32_t N, int b, std::vector<float>& x){
    int i;
    for ( i=1 ; i<=N ; i++ ) {
        x[IX(0 ,i)] = b==1 ? -x[IX(1,i)] : x[IX(1,i)];
        x[IX(N+1,i)] = b==1 ? -x[IX(N,i)] : x[IX(N,i)];
        x[IX(i,0 )] = b==2 ? -x[IX(i,1)] : x[IX(i,1)];
        x[IX(i,N+1)] = b==2 ? -x[IX(i,N)] : x[IX(i,N)];
    }
    x[IX(0 ,0 )] = 0.5f*(x[IX(1,0)]+x[IX(0,1)]);
    x[IX(0 ,N+1)] = 0.5f*(x[IX(1,N+1)]+x[IX(0,N)]);
    x[IX(N+1,0 )] = 0.5f*(x[IX(N,0)]+x[IX(N+1,1)]);
    x[IX(N+1,N+1)] = 0.5f*(x[IX(N,N+1)]+x[IX(N+1,N)]);
}