//
// Created by luc on 14/05/23.
//

#ifndef VULKANFLUIDSIMULATION_SPACIALGRID_H
#define VULKANFLUIDSIMULATION_SPACIALGRID_H

#include <vector>
#include "../utils.h"

namespace vkb {

    class SpatialGrid {
    public:
        SpatialGrid(float spacing, glm::vec3 gridBounds);
        SpatialGrid() = default;
        SpatialGrid(const SpatialGrid &) = delete;
        SpatialGrid &operator=(const SpatialGrid &) = default;
        ~SpatialGrid();

        template<typename Entity>
        void createAndSort(const std::vector<Entity>& entities, std::vector<Entity>& sortedEntities);

        void query(const glm::vec3& pos, float maxDist, const std::function<void(uint32_t)>& func);

    private:

        float m_spacing{};
        glm::vec3 m_gridBounds{};
        glm::vec<3, uint32_t> m_gridSize{};
        std::vector<uint32_t> m_grid{};

        [[nodiscard]] uint32_t vecToGrid(const glm::vec3 &pos) const;

    };

    template<typename Entity>
    void SpatialGrid::createAndSort(const std::vector<Entity>& entities, std::vector<Entity>& sortedEntities) {
        std::fill(m_grid.begin(), m_grid.end(), 0);

        for (const auto& entity : entities) {
            m_grid[vecToGrid(entity.position)]++;
        }

        // prefix sum
        std::inclusive_scan(m_grid.begin(), m_grid.end(), m_grid.begin());

        for (uint32_t i = 0; i < entities.size(); i++) {
            auto h = vecToGrid(entities[i].position);
            m_grid[h]--;
            sortedEntities[m_grid[h]] = entities[i];
        }

    }

} // vkb



#endif //VULKANFLUIDSIMULATION_SPACIALGRID_H
