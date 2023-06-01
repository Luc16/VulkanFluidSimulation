//
// Created by luc on 18/01/23.
//

#include "Grid2D.h"

namespace vkb {

    Grid2D::Grid2D(const Device& device, uint32_t width, uint32_t height, uint32_t size): m_width(width), m_height(height), m_size(size) {
        uint32_t numW = width/size, numH = height/size;
        auto pos = glm::vec3(0.0f, 0.0f, 0.0f);
        auto fsize = (float) size;

        m_vertices.resize((numW+1) * (numH+1));
        for (auto & vertex : m_vertices) {
            vertex = {pos, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}, {0.0f, 0.0f}};
            pos.x += fsize;

            if (pos.x > (float) width) {
                pos.x = 0.0f;
                pos.y += fsize;
            }
        }

        std::array<uint32_t, 6> defIndices{0, 1, numW+1, numW+1, 1, numW+2};
        m_indices.resize(numW * numH * defIndices.size());

        for (uint32_t i = 0, idx = 0; i < numW*numH; ++i, ++idx){
            if (idx % (numW + 1) == numW) idx++;
            for (uint32_t j = 0; j < defIndices.size(); ++j){
                m_indices[defIndices.size()*i + j] = idx + defIndices[j];
            }
        }


        m_gridModel = std::make_unique<Model>(device, m_vertices, m_indices);

    }

    void Grid2D::updateColor(uint32_t idx, glm::vec3 color) {
        m_vertices[idx].color = color;
    }

    void Grid2D::updateBuffer() {
        m_gridModel->updateVertexBuffer(m_vertices);
    }

    void Grid2D::render(VkCommandBuffer commandBuffer) {
        m_gridModel->bind(commandBuffer);
        m_gridModel->draw(commandBuffer);
    }


}