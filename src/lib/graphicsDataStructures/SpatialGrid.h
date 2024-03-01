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

        [[nodiscard]] uint32_t size() { return m_grid.size(); }

        template<typename Entity>
        void createAndSort(const Entity& entities, Entity& sortedEntities);
        template<typename Entity>
        void createAndSortVec(const Entity& entities, Entity& sortedEntities);

        void query(const glm::vec3& pos, float maxDist, const std::function<void(uint32_t)>& func);

    private:

        float m_spacing{};
        glm::vec3 m_gridBounds{};
        glm::vec<3, uint32_t> m_gridSize{};
        std::vector<uint32_t> m_grid{};

        [[nodiscard]] uint32_t vecToGrid(const glm::vec3 &pos) const;

    };

    template<typename Entity>
    void SpatialGrid::createAndSort(const Entity& entities, Entity& sortedEntities) {
        std::fill(m_grid.begin(), m_grid.end(), 0);

        for (uint32_t i = 0; i < entities.position.size(); i++) {
            m_grid[vecToGrid(entities.position[i])]++;
        }

        // prefix sum
        std::inclusive_scan(m_grid.begin(), m_grid.end(), m_grid.begin());

        for (uint32_t i = 0; i < entities.position.size(); i++) {
            auto h = vecToGrid(entities.position[i]);
            m_grid[h]--;
            sortedEntities.set(m_grid[h], entities, i);
        }

    }

    template<typename Entity>
    void SpatialGrid::createAndSortVec(const Entity& entities, Entity& sortedEntities) {
        std::fill(m_grid.begin(), m_grid.end(), 0);

        for (uint32_t i = 0; i < entities.predPos.size(); i++) {
            m_grid[vecToGrid(entities.predPos[i])]++;
        }

        // prefix sum
        std::inclusive_scan(m_grid.begin(), m_grid.end(), m_grid.begin());

        for (uint32_t i = 0; i < entities.predPos.size(); i++) {
            auto h = vecToGrid(entities.predPos[i]);
            m_grid[h]--;
            sortedEntities.set(m_grid[h], entities, i);
        }

    }

} // vkbEsqueceu



#endif //VULKANFLUIDSIMULATION_SPACIALGRID_H
