//
// Created by luc on 14/05/23.
//

#include "SpatialHash.h"

namespace vkb {
    SpatialHash::SpatialHash(float spacing, uint32_t maxNumObjects): m_spacing(spacing) {
        m_tableSize = 2*maxNumObjects;
        m_table.resize(m_tableSize + 1);
        m_cellEntries.resize(maxNumObjects);
    }

    SpatialHash::~SpatialHash() = default;

    void SpatialHash::query(const glm::vec3& pos, float maxDist, const std::function<void(uint32_t)>& func) {
        auto offset = glm::vec3(maxDist);

        uivec3 pos0 = vecToGrid(pos - offset);
        uivec3 pos1 = vecToGrid(pos + offset);

        for (uint32_t i = pos0.x; i <= pos1.x; i++){
            for (uint32_t j = pos0.y; j <= pos1.y; j++) {
//                for (uint32_t k = pos0.z; k <= pos1.z; k++) {
                    auto h = hash(i, j, 0);
                    for (uint32_t l = m_table[h]; l < m_table[h+1]; l++){
                        func(m_cellEntries[l]);
                    }
//                }
            }
        }
    }

    uint32_t SpatialHash::hash(uint32_t xi, uint32_t yi, uint32_t zi) const {
        return ((xi * 92837111) ^ (yi * 689287499) ^ (zi * 283923481)) % m_tableSize;
    }

    uint32_t SpatialHash::hash(const uivec3& vec) const {
        return hash(vec.x, vec.y, vec.z);
    }

    glm::vec<3, uint32_t> SpatialHash::vecToGrid(const glm::vec3 &pos) const {
        return {uint32_t(pos.x/m_spacing), uint32_t(pos.y/m_spacing), uint32_t(pos.z/m_spacing)};
    }

    uint32_t SpatialHash::hashVec(const glm::vec3 &pos) const {
        return hash(vecToGrid(pos));
    }
} // vkb