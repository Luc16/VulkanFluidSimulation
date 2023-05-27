//
// Created by luc on 14/05/23.
//

#include "SpatialHash.h"

namespace vkb {
    SpatialHash::SpatialHash(float spacing, uint32_t maxNumObjects, bool is3D) :
    m_spacing(spacing),
    m_is3D(is3D),
    m_tableSize(2*maxNumObjects),
    m_table(2*maxNumObjects + 1),
    m_cellEntries(maxNumObjects),
    queryRes(maxNumObjects),
    queryCount(0) {}

    SpatialHash::~SpatialHash() = default;

    void SpatialHash::query(const glm::vec3& pos, float maxDist, const std::function<void(uint32_t)>& func) {
        auto offset = glm::vec3(maxDist);

        glm::ivec3 pos0 = vecToGrid(pos - offset);
        glm::ivec3 pos1 = vecToGrid(pos + offset);

        for (int i = pos0.x; i <= pos1.x; i++){
            for (int j = pos0.y; j <= pos1.y; j++) {
                for (int k = pos0.z*m_is3D; k <= pos1.z*m_is3D; k++) {
                    auto h = hash(i, j, k);
                    for (uint32_t l = m_table[h]; l < m_table[h+1]; l++){
                        func(m_cellEntries[l]);
                    }
                }
            }
        }
    }

    void SpatialHash::query2(const glm::vec3& pos, float maxDist) {
        auto offset = glm::vec3(maxDist);

        glm::ivec3 pos0 = vecToGrid(pos - offset);
        glm::ivec3 pos1 = vecToGrid(pos + offset);
        queryCount = 0;

        for (int i = pos0.x; i <= pos1.x; i++){
            for (int j = pos0.y; j <= pos1.y; j++) {
                for (int k = pos0.z*m_is3D; k <= pos1.z*m_is3D; k++) {
                    auto h = hash(i, j, k);
                    for (uint32_t l = m_table[h]; l < m_table[h+1]; l++){
                        queryRes[queryCount++] = m_cellEntries[l];
                    }
                }
            }
        }
    }

    uint32_t SpatialHash::hash(int xi, int yi, int zi) const {
        return ((xi * 92837111) ^ (yi * 689287499) ^ (zi * 283923481)) % m_tableSize;
    }

    uint32_t SpatialHash::hash(const glm::ivec3& vec) const {
        return hash(vec.x, vec.y, vec.z);
    }

    glm::ivec3 SpatialHash::vecToGrid(const glm::vec3 &pos) const {
        return {int(pos.x/m_spacing), int(pos.y/m_spacing), int(pos.z/m_spacing)};
    }

    uint32_t SpatialHash::hashVec(const glm::vec3 &pos) const {
        return hash(vecToGrid(pos));
    }
} // vkb