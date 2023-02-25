//
// Created by luc on 18/01/23.
//

#ifndef VULKANFLUIDSIMULATION_GRID2D_H
#define VULKANFLUIDSIMULATION_GRID2D_H

#include <string>
#include <vector>
#include "../lib/Model.h"

namespace vkb {
    class Grid2D {

    public:
        Grid2D(const Device& device, uint32_t width, uint32_t height, uint32_t size);
        Grid2D(const Grid2D &) = delete;
        Grid2D &operator=(const Grid2D &) = delete;

        void updateColor(uint32_t idx, glm::vec3 color);
        void updateBuffer();
        void render(VkCommandBuffer commandBuffer);

    private:

        uint32_t m_width, m_height, m_size;
        std::vector<vkb::Model::Vertex> m_vertices;
        std::vector<uint32_t> m_indices;

        std::unique_ptr<vkb::Model> m_gridModel;
    };
}



#endif //VULKANFLUIDSIMULATION_GRID2D_H
