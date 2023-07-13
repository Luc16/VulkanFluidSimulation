//
// Created by luc on 14/05/23.
//

#include "SpatialGrid.h"

namespace vkb {
    SpatialGrid::SpatialGrid(float spacing, glm::vec3 gridBounds):
    m_spacing(spacing),
    m_gridBounds(gridBounds),
    m_gridSize(uint32_t(gridBounds.x/spacing + 1), uint32_t(gridBounds.y/spacing + 1), uint32_t(gridBounds.z/spacing + 1)),
    m_grid(m_gridSize.x * m_gridSize.y * m_gridSize.z + 1)
    {}

    SpatialGrid::~SpatialGrid() = default;

    void SpatialGrid::query(const glm::vec3& pos, float maxDist, const std::function<void(uint32_t)>& func) {
        auto offset = glm::vec3(maxDist);

        uint32_t idx = vecToGrid(pos);

        for (int y = -1; y <= 1; y++){
            int yIdx = y*int(m_gridSize.x*m_gridSize.z);
            if ((y == 1 && (pos.y + m_spacing >= m_gridBounds.y)) || (y == -1 &&  pos.y - m_spacing < 0)) continue;

            for (int z = -1; z <= 1; z++){
                int zIdx = z*int(m_gridSize.x);
                if ((z == 1 && (pos.z + m_spacing >= m_gridBounds.z)) || (z == -1 &&  pos.z - m_spacing < 0)) continue;

                for (int x = -1; x <= 1; x++){
                    if ((x == 1 && (pos.x + m_spacing >= m_gridBounds.x)) || (x == -1 &&  pos.x - m_spacing < 0)) continue;

                    uint32_t finalIdx = idx + x + zIdx + yIdx;

                    for (uint32_t j = m_grid[finalIdx]; j < m_grid[finalIdx + 1]; j++){
                        func(j);
                    }
                }

            }
        }
    }


    uint32_t SpatialGrid::vecToGrid(const glm::vec3 &pos) const {
        return uint32_t(pos.x/m_spacing) + m_gridSize.x*(uint32_t(pos.z/m_spacing) + uint32_t(pos.y/m_spacing)*m_gridSize.z);
    }

} // vkb