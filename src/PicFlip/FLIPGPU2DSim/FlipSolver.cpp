//
// Created by luc on 25/01/24.
//

#include "FlipSolver.h"

void FlipSolver::updateSimulation(float deltaTime, float flipRatio) {

    auto printVec = [](const std::vector<double>& vec){
        for (auto& v : vec) {
            std::cout << v << ", ";
        }
        std::cout << "\n";
    };

    m_testBufferData = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    printVec(m_testBufferData);
    m_testBufferData = {9, 8, 7, 6, 5, 4, 3, 2, 1, 0};
    printVec(m_testBufferData);
    m_computeHandler.runComputeIsolated(0, [this](VkCommandBuffer commandBuffer){
        m_addScaledKernel.bindAndDispatch(commandBuffer, 0, 1, 1, 1);
    });
    vkb::Buffer::writeBufferToVector(m_deviceRef, m_directionBuffer, m_testBufferData);
    printVec(m_testBufferData);



}

void FlipSolver::initializeParticles(const std::unique_ptr<vkb::DescriptorPool> &globalPool)  {
    m_addScaledKernel.createPipeline();
    m_cUbo.size = 10;
    m_computeUniformBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                          sizeof(m_cUbo),
                                                          VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT,
                                                          VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    m_computeUniformBuffer->singleWrite(&m_cUbo);
    m_testBufferData = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};

    m_directionBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                 m_testBufferData.size()*sizeof(m_testBufferData[0]),
                                                 VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                 VK_BUFFER_USAGE_TRANSFER_DST_BIT |
                                                 VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                                 VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    vkb::Buffer::writeVectorToBuffer(m_deviceRef, m_directionBuffer, m_testBufferData);

    m_testBufferData = {9, 8, 7, 6, 5, 4, 3, 2, 1, 0};

    m_auxBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                      m_testBufferData.size()*sizeof(m_testBufferData[0]),
                                                      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                      VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                                      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    vkb::Buffer::writeVectorToBuffer(m_deviceRef, m_auxBuffer, m_testBufferData);

    m_alphaBuffer = std::make_unique<vkb::Buffer>(m_deviceRef,
                                                      sizeof(double),
                                                      VK_BUFFER_USAGE_STORAGE_BUFFER_BIT |
                                                      VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                                                      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    std::vector<double> alpha = {-0.5};
    vkb::Buffer::writeVectorToBuffer(m_deviceRef, m_alphaBuffer, alpha);



    m_addScaledKernel.descSets[0] = vkb::DescriptorWriter::createSingleDescriptorSet(globalPool, m_addScaledKernel.layout, {
            {m_computeUniformBuffer->descriptorInfo()},
            {m_directionBuffer->descriptorInfo()},
            {m_directionBuffer->descriptorInfo()},
            {m_alphaBuffer->descriptorInfo()},
            {m_auxBuffer->descriptorInfo()},
    });


//    uint32_t k = 0, na = 3, nb = 3;
//    uint32_t s = numTilesX/4, e = 3*numTilesX/4;
////    uint32_t s = 0, e = numTilesX - 1;
//    for (uint32_t j = 4 ; j < numTilesY-1; j++) {
//        for (uint32_t i = s; i < e; i++) {
//            for (uint32_t a = 0; a < na; a++){
//                for (uint32_t b = 0; b < nb; b++) {
//                    if (k >= particles.size()) return;
//                    auto& particle = particles[k++];
//                    particle.position = glm::vec3(
//                            i*cellSize + float(a*cellSize)/float(na) +
//                            float(cellSize)/float(na*na) + randomFloat(0.0f, 0.3f*cellSize),
//                            j*cellSize + float(b*cellSize)/float(nb) +
//                            float(cellSize)/float(nb*nb) + randomFloat(0.0f, 0.3f*cellSize),
//                            0.0f);
//                    particle.velocity = glm::vec3(0.0f, 0.0f, 0.0f);
//                    particle.color = glm::vec4(0.2f, 0.6f, 1.0f, 1.0f);
//                }
//            }
//        }
//    }
}