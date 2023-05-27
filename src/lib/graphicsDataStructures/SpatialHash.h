//
// Created by luc on 14/05/23.
//

#ifndef VULKANFLUIDSIMULATION_SPACIALHASH_H
#define VULKANFLUIDSIMULATION_SPACIALHASH_H

#include <vector>
#include "../utils.h"
#include <forward_list>

namespace vkb {

    class SpatialHash {
    public:
        SpatialHash(float spacing, uint32_t maxNumObjects, bool is3D = true);
        SpatialHash(const SpatialHash &) = delete;
        SpatialHash &operator=(const SpatialHash &) = delete;
        ~SpatialHash();

        template<typename Entity>
        void create(const std::vector<Entity>& entities);

        void query(const glm::vec3& pos, float maxDist, const std::function<void(uint32_t)>& func);
        void query2(const glm::vec3& pos, float maxDist);

        std::vector<uint32_t> queryRes;
        uint32_t queryCount;

    private:

        std::vector<uint32_t> m_table, m_cellEntries;
        float m_spacing;
        bool m_is3D;
        uint32_t m_tableSize;

        [[nodiscard]] uint32_t hash(int xi , int yi, int zi) const;
        [[nodiscard]] uint32_t hash(const glm::ivec3& vec) const;
        [[nodiscard]] uint32_t hashVec(const glm::vec3& pos) const;
        [[nodiscard]] glm::ivec3 vecToGrid(const glm::vec3& pos) const;

    };

    template<typename Entity>
    void SpatialHash::create(const std::vector<Entity>& entities) {
        uint32_t numElements = std::min(entities.size(), m_cellEntries.size());
        std::fill(m_table.begin(), m_table.end(), 0);
        std::fill(m_cellEntries.begin(), m_cellEntries.end(), 0);

        for (const auto& entity : entities) {
            m_table[hashVec(entity.position)]++;
        }

        uint32_t start = 0;
        for (uint32_t i = 0; i < m_tableSize; i++) {
            start += m_table[i];
            m_table[i] = start;
        }
        m_table[m_tableSize] = start;


        for (uint32_t i = 0; i < numElements; i++) {
            auto h = hashVec(entities[i].position);
            m_table[h]--;
            m_cellEntries[m_table[h]] = i;
        }

    }

} // vkb



#endif //VULKANFLUIDSIMULATION_SPACIALHASH_H
