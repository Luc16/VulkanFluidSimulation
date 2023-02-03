//
// Created by luc on 30/01/23.
//

#include "Grid3DSim.h"
#include <cmath>
#include <execution>

void Grid3DSim::onCreate() {
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
            info.bindingDescription.push_back(instancedCubes.getBindingDescription());
            info.attributeDescription.push_back({4, 1, VK_FORMAT_R32G32B32_SFLOAT, offsetof(InstanceData, position)});
            info.attributeDescription.push_back({5, 1, VK_FORMAT_R32G32B32A32_SFLOAT, offsetof(InstanceData, color)});
            info.attributeDescription.push_back({6, 1, VK_FORMAT_R32_SFLOAT, offsetof(InstanceData, scale)});

            info.rasterizer.cullMode = VK_CULL_MODE_BACK_BIT;

            info.enableAlphaBlending();
        });
    }
}

void Grid3DSim::initializeObjects() {
    camera.setViewTarget({0.0f, 10.0f, 10.0f}, {12.0f, -1.0f, -12.0f }, {0.0f, 1.0f, 0.0f});
    camera.m_rotation = {0, glm::radians(180.0f), glm::radians(180.0f)};
    plane.setScale(100);
    plane.translate({0.0f, -1.0f, 0.0f});

    createInstances();
}

void Grid3DSim::createInstances() {
    vkDeviceWaitIdle(device.device());

    INSTANCE_COUNT = CUBE_SIDE*CUBE_SIDE*CUBE_SIDE;
    CUBE_N = CUBE_SIDE - 2;

    instancedCubes.resizeBuffer(INSTANCE_COUNT);
    indices.resize(INSTANCE_COUNT);
    sortedData.resize(INSTANCE_COUNT);
    curState.velX.resize(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);
    curState.velY.resize(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);
    curState.velZ.resize(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);
    curState.density.resize(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);
    prevState.velX.resize(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);
    prevState.velY.resize(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);
    prevState.velZ.resize(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);
    prevState.density.resize(CUBE_SIDE, CUBE_SIDE, CUBE_SIDE);

    auto accPos = glm::vec3(0.0f, 0.0f, -10.0f);

    float scale = 1.0f;
    float cubeSide = 2.0f*scale;

    for (uint32_t i = 0; i < INSTANCE_COUNT; i++) {
        // initializing cell states
        curState.velX[i] = 0.0f;
        curState.velY[i] = 0.0f;
        curState.velZ[i] = 0.0f;
        curState.density[i] = 0.0f;
        prevState.velX[i] = 0.0f;
        prevState.velY[i] = 0.0f;
        prevState.velZ[i] = 0.0f;
        prevState.density[i] = 0.0f;
        indices[i] = std::make_pair(i, i);

        // initializing cubes
        auto& cube = instancedCubes[i];
        cube.color = {0.0f, 0.0f, 0.0f, 0.0f};
        cube.scale = scale;
        cube.position = accPos;
        accPos.x += cubeSide;

        if (accPos.x >= (float)CUBE_SIDE*cubeSide) {
            accPos.y += cubeSide;
            if (accPos.y >= (float)CUBE_SIDE*cubeSide) {
                accPos.z -= cubeSide;
                accPos.y = 0.0f;
            }
            accPos.x = 0.0f;
        }
    }

    instancedCubes.updateBuffer();
}

void Grid3DSim::createUniformBuffers() {
    VkDeviceSize bufferSize = sizeof(UniformBufferObject);
    uniformBuffers.resize(vkb::SwapChain::MAX_FRAMES_IN_FLIGHT);
    for (size_t i = 0; i < vkb::SwapChain::MAX_FRAMES_IN_FLIGHT; ++i) {
        uniformBuffers[i] = std::make_unique<vkb::Buffer>(device, bufferSize, VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    }
}

void Grid3DSim::mainLoop(float deltaTime) {
    auto currentTime = std::chrono::high_resolution_clock::now();

    cameraController.moveCamera(window.window(), deltaTime, camera);
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

            defaultSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);
            plane.render(defaultSystem, commandBuffer);

            instanceSystem.bind(commandBuffer, &defaultDescriptorSets[renderer.currentFrame()]);
            instancedCubes.render(instanceSystem, commandBuffer);
        });
    });
    if (activateTimer) gpuTime = std::chrono::duration<float, std::chrono::milliseconds::period>(std::chrono::high_resolution_clock::now() - currentTime).count();
}

void Grid3DSim::updateUniformBuffer(uint32_t frameIndex) {

    UniformBufferObject ubo{};
    camera.setPerspectiveProjection(glm::radians(50.f), renderer.getSwapChainAspectRatio(), 0.1f, 1000.f);
    ubo.view = camera.getView();
    ubo.proj = camera.getProjection();
    uniformBuffers[frameIndex]->singleWrite(&ubo);

}

void Grid3DSim::showImGui(){

    ImGui::Begin("Control Panel");

    ImGui::Text("Rendering %d instances", INSTANCE_COUNT);
    ImGui::Checkbox("Display time", &activateTimer);
    if(activateTimer){
        ImGui::Text("Gpu time: %f ms", gpuTime);
        ImGui::Text("Cpu time: %f ms", cpuTime);
    }

    auto prevCbSide = CUBE_SIDE;
    ImGui::DragInt("Cube Side", (int *) &CUBE_SIDE, 1, 2, 50);
    if (CUBE_SIDE != prevCbSide) createInstances();

    ImGui::NewLine();
    ImGui::DragFloat("Diffusion Factor", &diffusionFactor, 0.0001f, 0.0f, 5.0f, "%.4f");
    ImGui::DragFloat("Viscosity", &viscosity, 0.0001f, 0.0f, 5.0f, "%.4f");
    ImGui::DragFloat("Dissolve Factor", &dissolveFactor, 0.001f, 0.0f, 2.0f, "%.3f");
    ImGui::DragFloat("Speed", &initialSpeed, 1.0f, 000.0f, 1000.0f, "%.1f");
    if (ImGui::Button("Reset")) createInstances();

    if (ImGui::CollapsingHeader("Plane", ImGuiTreeNodeFlags_DefaultOpen)) {

        ImGui::SliderFloat("y", &plane.m_translation.y, -100.0f, 10.0f);

    }

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate,
                ImGui::GetIO().Framerate);

    ImGui::End();
}

void Grid3DSim::updateGrid(float deltaTime){

    updateVelocities(deltaTime);
    updateDensities(deltaTime);

    for (const auto& [i, j] : indices){
        curState.density[i] = std::clamp(curState.density[i] - deltaTime*dissolveFactor, 0.0f, 1.0f);
        instancedCubes[j].color = glm::vec4(curState.density[i]);
    }

    // sort the cubes to draw first the ones further away
    std::sort(indices.begin(), indices.end(), [this](const std::pair<uint32_t, uint32_t >& i0, const std::pair<uint32_t, uint32_t >& i1) {
        auto diff1 = instancedCubes[i0.second].position - camera.m_translation;
        auto diff2 = instancedCubes[i1.second].position - camera.m_translation;

        return glm::dot(diff1, diff1) > glm::dot(diff2, diff2);
    });
    for (uint32_t i = 0; i < indices.size(); i++){
        sortedData[i] = instancedCubes[indices[i].second];
        indices[i].second = i;
    }

    instancedCubes.swap(sortedData);

    instancedCubes.updateBuffer();
}

void Grid3DSim::updateDensities(float deltaTime) {

    curState.density(CUBE_SIDE/2, 1, CUBE_SIDE/2) += randomFloat(0.3f, 0.6f);
    curState.density(CUBE_SIDE/2 + 1, 1, CUBE_SIDE/2) += randomFloat(0.3f, 0.6f);
    curState.density(CUBE_SIDE/2 - 1, 1, CUBE_SIDE/2) += randomFloat(0.3f, 0.6f);
    curState.density(CUBE_SIDE/2, 1, CUBE_SIDE/2 - 1) += randomFloat(0.3f, 0.6f);
    curState.density(CUBE_SIDE/2, 1, CUBE_SIDE/2 + 1) += randomFloat(0.3f, 0.6f);

    curState.density.swap(prevState.density);
    diffuse(curState.density, prevState.density, diffusionFactor, deltaTime);
    curState.density.swap(prevState.density);
    advect(curState.density, prevState.density, curState.velX, curState.velY, curState.velZ, deltaTime);
}

void Grid3DSim::updateVelocities(float deltaTime) {

    curState.velY(CUBE_SIDE/2, 1, CUBE_SIDE/2) = deltaTime*initialSpeed;
    curState.velY(CUBE_SIDE/2 + 1, 1, CUBE_SIDE/2) = deltaTime*initialSpeed;
    curState.velY(CUBE_SIDE/2 - 1, 1, CUBE_SIDE/2) = deltaTime*initialSpeed;
    curState.velY(CUBE_SIDE/2, 1, CUBE_SIDE/2 - 1) = deltaTime*initialSpeed;
    curState.velY(CUBE_SIDE/2, 1, CUBE_SIDE/2 + 1) = deltaTime*initialSpeed;

    curState.velX.swap(prevState.velX);
    curState.velY.swap(prevState.velY);
    curState.velZ.swap(prevState.velZ);
    diffuse(curState.velX, prevState.velX, viscosity, deltaTime);
    diffuse(curState.velY, prevState.velY, viscosity, deltaTime);
    diffuse(curState.velZ, prevState.velZ, viscosity, deltaTime);

    project(curState.velX, curState.velY, curState.velZ, prevState.velX, prevState.velY);

    curState.velX.swap(prevState.velX);
    curState.velY.swap(prevState.velY);
    curState.velZ.swap(prevState.velZ);
    advect(curState.velX, prevState.velX, prevState.velX, prevState.velY, prevState.velZ, deltaTime, MIRROR_X);
    advect(curState.velY, prevState.velY, prevState.velX, prevState.velY, prevState.velZ, deltaTime, MIRROR_Y);
    advect(curState.velZ, prevState.velZ, prevState.velX, prevState.velY, prevState.velZ, deltaTime, MIRROR_Z);

    project(curState.velX, curState.velY, curState.velZ, prevState.velX, prevState.velY);
}

void Grid3DSim::diffuse(Matrix3D<float>& x, const Matrix3D<float>& x0, float diff, float dt) {
    float a = dt*diff* (float) (CUBE_N*CUBE_N);
    for (int _ = 0; _ < 20; ++_) {
        for (uint32_t i = 1; i <= CUBE_N; ++i) {
            for (uint32_t j = 1; j <= CUBE_N; ++j) {
                for (uint32_t k = 1; k <= CUBE_N; ++k) {
                    x(i, j, k) = (x0(i, j, k) + a*(
                            x(i-1, j, k) + x(i+1, j, k) +
                            x(i, j-1, k) + x(i, j+1, k) +
                            x(i, j, k-1) + x(i, j, k+1)))/(1+6*a);
                }
            }
        }
        setBounds(x, REGULAR);
    }
}

std::tuple<uint32_t, uint32_t, float, float> Grid3DSim::linearBackTrace(float pos, float fN){
    if (pos < 0.5f) pos = 0.5f;
    if (pos > fN + 0.5f) pos = fN + 0.5f;
    auto i0 = (uint32_t) pos;
    auto i1 = i0 + 1;
    float s1 = pos - (float) i0;
    float s0 = 1 - s1;

    return std::make_tuple(i0, i1, s0, s1);

}

void Grid3DSim::advect(Matrix3D<float>& d, const Matrix3D<float>& d0, const Matrix3D<float>& velX, const Matrix3D<float>& velY, const Matrix3D<float>& velZ, float dt, BoundConfig b) {
    float dt0, fN = (float) CUBE_N;
    dt0 = dt*fN;

    for (uint32_t i = 1; i <= CUBE_N; ++i) {
        for (uint32_t j = 1; j <= CUBE_N; ++j) {
            for (uint32_t k = 1; k <= CUBE_N; ++k) {
                auto [i0, i1, s0, s1] = linearBackTrace((float) i - dt0*velX(i, j, k), fN);
                auto [j0, j1, t0, t1] = linearBackTrace((float) j - dt0*velY(i, j, k), fN);
                auto [k0, k1, u0, u1] = linearBackTrace((float) k - dt0*velZ(i, j, k), fN);

                d(i, j, k) =
                        s0 * (
                            t0 * (u0*d0(i0, j0, k0) +
                                  u1*d0(i0, j0, k1)) +
                            t1 * (u0*d0(i0, j1, k0) +
                                  u1*d0(i0, j1, k1))) +
                        s1 * (
                            t0 * (u0*d0(i1, j0, k0) +
                                  u1*d0(i1, j0, k1)) +
                            t1 * (u0*d0(i1, j1, k0) +
                                  u1*d0(i1, j1, k1)));
            }
        }
    }
    setBounds(d, b);
}

void Grid3DSim::project(Matrix3D<float> &velX, Matrix3D<float> &velY, Matrix3D<float> &velZ, Matrix3D<float> &div, Matrix3D<float> &p) {
    float h = 1/(float) CUBE_N;

    for (uint32_t i = 1; i <= CUBE_N; ++i) {
        for (uint32_t j = 1; j <= CUBE_N; ++j) {
            for (uint32_t k = 1; k <= CUBE_N; ++k) {
                div(i, j, k) = -0.5f * h * ( velX(i + 1, j, k) - velX(i - 1, j, k) +
                                             velY(i, j + 1, k) - velY(i, j - 1, k) +
                                             velZ(i, j, k + 1) - velZ(i, j, k - 1));
                p(i, j, k) = 0;
            }
        }
    }
    setBounds(div);
    setBounds(p);


    for (uint32_t _ = 0; _ < 20; ++_) {
        for (uint32_t i = 1; i <= CUBE_N; ++i) {
            for (uint32_t j = 1; j <= CUBE_N; ++j) {
                for (uint32_t k = 1; k <= CUBE_N; ++k) {
                    p(i,j, k) = (div(i,j, k) +
                                 p(i + 1, j, k) + p(i - 1, j, k) +
                                 p(i, j + 1, k) + p(i, j - 1, k) +
                                 p(i, j, k + 1) + p(i, j, k - 1))/6;
                }
            }
        }
        setBounds(p);
    }

    for (uint32_t i = 1; i <= CUBE_N; ++i) {
        for (uint32_t j = 1; j <= CUBE_N; ++j) {
            for (uint32_t k = 1; k <= CUBE_N; ++k) {
                velX(i, j, k) -= 0.5f * (p(i + 1, j, k) - p(i - 1, j, k)) / h;
                velY(i, j, k) -= 0.5f * (p(i, j + 1, k) - p(i, j - 1, k)) / h;
                velZ(i, j, k) -= 0.5f * (p(i, j, k + 1) - p(i, j, k - 1)) / h;
            }
        }
    }
    setBounds(velX, MIRROR_X);
    setBounds(velY, MIRROR_Y);
    setBounds(velZ, MIRROR_Z);

}

void Grid3DSim::setBounds(Matrix3D<float>& x, BoundConfig b) const{
    for (int i = 1; i <= CUBE_N; ++i) {
        for (int j = 1; j <= CUBE_N; ++j) {
            x(0, i, j) = (b == MIRROR_X) ? -x(1, i, j) : x(1, i, j);
            x(CUBE_N+1, i, j) = (b == MIRROR_X) ? -x(CUBE_N, i, j) : x(CUBE_N, i, j);

            x(i, 0, j) = (b == MIRROR_Y) ? -x(i, 1, j) : x(i, 1, j);
            x(i, CUBE_N+1, j) = (b == MIRROR_Y) ? -x(i, CUBE_N, j) : x(i, CUBE_N, j);

            x(i, j, 0) = (b == MIRROR_Z) ? -x(i, j, 1) : x(i, j, 1);
            x(i, j, CUBE_N+1) = (b == MIRROR_Z) ? -x(i, j, CUBE_N) : x(i, j, CUBE_N);
        }
    }

    for (int i = 1; i <= CUBE_N; ++i) {
        x(i, 0, 0) = 0.5f * (x(i, 1, 0) + x(i, 0, 1));
        x(0, i, 0) = 0.5f * (x(1, i, 0) + x(0, i, 1));
        x(0, 0, i) = 0.5f * (x(1, 0, i) + x(0, 1, i));


        x(i, CUBE_N+1, 0) = 0.5f * (x(i, CUBE_N, 0) + x(i, CUBE_N+1, 1));
        x(CUBE_N+1, i, 0) = 0.5f * (x(CUBE_N, i, 0) + x(CUBE_N+1, i, 1));
        x(CUBE_N+1, 0, i) = 0.5f * (x(CUBE_N, 0, i) + x(CUBE_N+1, 1, i));

        x(i, 0, CUBE_N+1) = 0.5f * (x(i, 1, CUBE_N+1) + x(i, 0, CUBE_N));
        x(0, i, CUBE_N+1) = 0.5f * (x(1, i, CUBE_N+1) + x(0, i, CUBE_N));
        x(0, CUBE_N+1, i) = 0.5f * (x(1, CUBE_N+1, i) + x(0, CUBE_N, i));

        x(i, CUBE_N+1, CUBE_N+1) = 0.5f * (x(i, CUBE_N, CUBE_N+1) + x(i, CUBE_N+1, CUBE_N));
        x(CUBE_N+1, i, CUBE_N+1) = 0.5f * (x(CUBE_N, i, CUBE_N+1) + x(CUBE_N+1, i, CUBE_N));
        x(CUBE_N+1, CUBE_N+1, i) = 0.5f * (x(CUBE_N, CUBE_N+1, i) + x(CUBE_N+1, CUBE_N, i));
    }

    x(0, 0, 0) = 0.333f*(x(1, 0, 0) +
                                 x(0, 1, 0) +
                                 x(0, 0, 1));
    x(CUBE_N+1, 0, 0) = 0.333f*(x(CUBE_N, 0, 0) +
                                        x(CUBE_N+1, 1, 0) +
                                        x(CUBE_N+1, 0, 1));
    x(0, CUBE_N+1, 0) = 0.333f*(x(1, CUBE_N+1, 0) +
                                        x(0, CUBE_N, 0) +
                                        x(0, CUBE_N+1, 1));
    x(0, 0, CUBE_N+1) = 0.333f*(x(1, 0, CUBE_N+1) +
                                        x(0, 1, CUBE_N+1) +
                                        x(0, 0, CUBE_N));
    x(0, CUBE_N+1, CUBE_N+1) = 0.333f*(x(1, CUBE_N+1, CUBE_N+1) +
                                               x(0, CUBE_N, CUBE_N+1) +
                                               x(0, CUBE_N+1, CUBE_N));
    x(CUBE_N+1, 0, CUBE_N+1) = 0.333f*(x(CUBE_N, 0, CUBE_N+1) +
                                               x(CUBE_N+1, 1, CUBE_N+1) +
                                               x(CUBE_N+1, 0, CUBE_N));
    x(CUBE_N+1, CUBE_N+1, 0) = 0.333f*(x(CUBE_N, CUBE_N+1, 0) +
                                               x(CUBE_N+1, CUBE_N, 0) +
                                               x(CUBE_N+1, CUBE_N+1, 1));
    x(CUBE_N+1, CUBE_N+1, CUBE_N+1) = 0.333f*(x(CUBE_N, CUBE_N+1, CUBE_N+1) +
                                                      x(CUBE_N+1, CUBE_N, CUBE_N+1) +
                                                      x(CUBE_N+1, CUBE_N+1, CUBE_N));


}