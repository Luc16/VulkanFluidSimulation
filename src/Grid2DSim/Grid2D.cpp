//
// Created by luc on 18/01/23.
//

#include "Grid2D.h"

namespace vkb {

    Grid2D::Grid2D(const Device& device, uint32_t width, uint32_t height, uint32_t size): m_width(width), m_height(height), m_size(size) {
        uint32_t numW = width/size, numH = height/size;
        auto pos = glm::vec3(0.0f, 0.0f, 0.0f);
        auto fsize = (float) size;

        m_vertices.resize(numW * numH * 4);
        m_indices.resize(numW * numH * 6);

        for (int i = 0; i < numH*numW; ++i) {
            for (int j = 0; j < 4; j++){
                m_vertices[4 * i + j] = {pos + quad[j].pos * fsize, quad[j].color, quad[j].normal, quad[j].texCoord};
            }
            for (int j = 0; j < quadIndices.size(); ++j) {
                m_indices[6 * i + j] = 4*i + quadIndices[j];
            }
            pos.x += fsize;

            if (pos.x + fsize > (float) width) {
                pos.x = 0.0f;
                pos.y += fsize;
            }
        }

        m_gridModel = std::make_unique<Model>(device, m_vertices, m_indices);

    }

    void Grid2D::updateColor(uint32_t idx, glm::vec3 color) {
        for (int j = 0; j < 4; j++){
            m_vertices[4 * idx + j].color = color;
        }
    }

    void Grid2D::updateBuffer() {
        m_gridModel->updateVertexBuffer(m_vertices);
    }

    void Grid2D::render(VkCommandBuffer commandBuffer) {
        m_gridModel->bind(commandBuffer);
        m_gridModel->draw(commandBuffer);
    }


}