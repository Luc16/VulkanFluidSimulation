//
// Created by luc on 20/11/23.
//


#include <unordered_set>
#include "RigidObject.h"
#include "../../../external/objloader/tiny_obj_loader.h"
#include <algorithm>
#include <functional>

RigidObject::RigidObject(const vkb::Device& device, const std::string& modelFile, const std::shared_ptr<vkb::Texture>& tex,
                         float scale): m_scale(scale) {
    m_name = modelFile;
    std::vector<vkb::Model::Vertex> vertices{};

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, modelFile.c_str())) {
        throw std::runtime_error(warn + err);
    }

    std::unordered_map<vkb::Model::Vertex, uint32_t> uniqueVertices{};

    for (const auto& shape: shapes) {
        for (const auto& index : shape.mesh.indices) {
            vkb::Model::Vertex vertex{};
            vertex.pos = scale*glm::vec3(
                    attrib.vertices[3 * index.vertex_index + 0],
                    attrib.vertices[3 * index.vertex_index + 1],
                    attrib.vertices[3 * index.vertex_index + 2]
            );
            vertex.color = {
                    attrib.colors[3 * index.vertex_index + 0],
                    attrib.colors[3 * index.vertex_index + 1],
                    attrib.colors[3 * index.vertex_index + 2],
            };
            if (index.normal_index > 0)
                vertex.normal = {
                        attrib.normals[3 * index.normal_index + 0],
                        attrib.normals[3 * index.normal_index + 1],
                        attrib.normals[3 * index.normal_index + 2],
                };
//            if (index.texcoord_index > 0)
//                vertex.texCoord = {
//                        attrib.texcoords[2 * index.texcoord_index],
//                        1.0f - attrib.texcoords[2 * index.texcoord_index + 1],
//                };

            vertices.push_back(vertex);
            if (uniqueVertices.count(vertex) == 0){
                uniqueVertices[vertex] = static_cast<uint32_t>(m_vertices.size());
                m_vertices.push_back(vertex.pos);
            }

            m_indices.push_back(uniqueVertices[vertex]);
        }
    }

    for (size_t i = 0; i < vertices.size(); i += 3) {
        glm::vec3 v0 = vertices[i].pos;
        glm::vec3 v1 = vertices[i + 1].pos;
        glm::vec3 v2 = vertices[i + 2].pos;

        glm::vec3 normal = glm::normalize(glm::cross(v1 - v0, v2 - v0));

        vertices[i].normal = normal;
        vertices[i + 1].normal = normal;
        vertices[i + 2].normal = normal;
    }


    m_object = std::make_unique<vkb::DrawableObject>(std::make_unique<vkb::Model>(device, vertices), tex);

}


void RigidObject::translate(const glm::vec3 &move) {
    if (glm::dot(move, move) == 0) return;
    m_object->translate(move);
}

void RigidObject::rotate(const glm::vec3 &rotation) {
    m_object->rotation += rotation;
}

void RigidObject::createSdf(const vkb::Device& device, float cellSize, const glm::vec3& gridDimensions) {
    if (m_sdfPos == m_object->translation) return;
    m_sdfPos = m_object->translation;

    std::vector<glm::vec3> verts(m_vertices.size());
    for (uint32_t i = 0; i < m_vertices.size(); i++) {
        verts[i] = m_object->modelMatrix()*glm::vec4(m_vertices[i], 1.0f);
    }

    auto min = [](float x, float y, float z) { return std::min(std::min(x, y), z); };
    auto max = [](float x, float y, float z) { return std::max(std::max(x, y), z); };
    auto distancePointTriangle = [](const glm::vec3& p, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {
        const glm::vec3 ab = b - a;
        const glm::vec3 ac = c - a;
        const glm::vec3 ap = p - a;

        const float d1 = dot(ab, ap);
        const float d2 = dot(ac, ap);
        if (d1 <= 0.f && d2 <= 0.f) return glm::distance(p, a);

        const glm::vec3 bp = p - b;
        const float d3 = dot(ab, bp);
        const float d4 = dot(ac, bp);
        if (d3 >= 0.f && d4 <= d3) return glm::distance(p, b);

        const glm::vec3 cp = p - c;
        const float d5 = dot(ab, cp);
        const float d6 = dot(ac, cp);
        if (d6 >= 0.f && d5 <= d6) return glm::distance(p, c);

        const float vc = d1 * d4 - d3 * d2;
        if (vc <= 0.f && d1 >= 0.f && d3 <= 0.f)
        {
            const float v = d1 / (d1 - d3);
            return glm::distance(a + v * ab, p);
        }

        const float vb = d5 * d2 - d1 * d6;
        if (vb <= 0.f && d2 >= 0.f && d6 <= 0.f)
        {
            const float v = d2 / (d2 - d6);
            return glm::distance(a + v * ac, p);
        }


        const float va = d3 * d6 - d5 * d4;
        if (va <= 0.f && (d4 - d3) >= 0.f && (d5 - d6) >= 0.f)
        {
            const float v = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return glm::distance(b + v * (c - b), p);
        }

        const float denom = 1.f / (va + vb + vc);
        const float v = vb * denom;
        const float w = vc * denom;
        return glm::distance(a + v * ab + w * ac, p);

    };

    auto signedVolume = [](const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d) {
        return dot((d - a), cross(b - a, c - a));
    };


    auto gridDim = glm::ivec3(gridDimensions/cellSize);
    auto getIdx = [&gridDim](glm::ivec3 idx) {
        return idx.x + idx.z*gridDim.x + idx.y*gridDim.x*gridDim.z;
    };

    auto size = uint32_t(gridDim.x*gridDim.y*gridDim.z);
    auto maxFloat = std::numeric_limits<float>::max();
    std::vector<float> sdf(size, maxFloat);
    std::vector<uint32_t> border(size, 0);
    std::vector<int> triangleIdx(size, -1);

    // for each triangle
    for (int l = 0; l < m_indices.size()/3; l++) {
        auto p1 = verts[m_indices[3*l]];
        auto p2 = verts[m_indices[3*l+1]];
        auto p3 = verts[m_indices[3*l+2]];

        std::array<glm::ivec3, 2> bounds = {
                glm::ivec3(int(min(p1.x, p2.x, p3.x)/cellSize), int(min(p1.y, p2.y, p3.y)/cellSize), int(min(p1.z, p2.z, p3.z)/cellSize)),
                glm::ivec3(int(max(p1.x, p2.x, p3.x)/cellSize), int(max(p1.y, p2.y, p3.y)/cellSize), int(max(p1.z, p2.z, p3.z)/cellSize))
        };

        for (int i = bounds[0].x; i <= bounds[1].x; i++) {
            for (int j = bounds[0].y; j <= bounds[1].y; j++) {
                for (int k = bounds[0].z; k <= bounds[1].z; k++) {
                    if (i > gridDim.x - 1 || j > gridDim.y - 1 || k > gridDim.z - 1 || i < 1 || j < 1 || k < 1) continue;

                    auto pa = glm::vec3(i, j, k)*cellSize;
                    auto pb = glm::vec3(i + 1, j, k)*cellSize;

                    // check if line pa pb intersects triangle p1 p2 p3
                    auto sv1 = signedVolume(pa,pb,p1,p2);
                    auto sv2 = signedVolume(pa,pb,p2,p3);
                    auto sv3 = signedVolume(pa,pb,p3,p1);
                    if (signedVolume(pa, p1, p2, p3)*signedVolume(pb, p1, p2, p3) > 0 || sv1*sv2 < 0 || sv2*sv3 < 0) continue;

                    auto d = distancePointTriangle(pa, p1, p2, p3);

                    auto idx = getIdx(glm::ivec3(i, j, k));
                    border[idx]++;
                    if (d < sdf[idx]) {
                        sdf[idx] = d;
                        triangleIdx[idx] = l;
                    }
                }
            }
        }
    }

    // propagate closest triangle and distance to all cells
    std::function<bool(int)> icomp, jcomp, kcomp;

    for (int _ = 0; _ < 10; _++){
        for (int n = 0; n < 8; n++) {
            auto iadd = 2 * (n % 2) - 1;
            auto jadd = 2 * ((n / 2) % 2) - 1;
            auto kadd = 2 * (n < 3) - 1;

            int iStart = iadd > 0 ? 1 : gridDim.x - 2;
            int jStart = jadd > 0 ? 1 : gridDim.y - 2;
            int kStart = kadd > 0 ? 1 : gridDim.z - 2;

            if (iadd > 0) icomp = [&gridDim](int i){ return i < gridDim.x - 1; };
            else icomp = [](int i){return i > 0; };
            if (jadd > 0) jcomp = [&gridDim](int j){ return j < gridDim.y - 1; };
            else jcomp = [](int j){return j > 0; };
            if (kadd > 0) kcomp = [&gridDim](int k){ return k < gridDim.z - 1; };
            else kcomp = [](int k){return k > 0; };

            for (int j = jStart; jcomp(j); j += jadd) {
                for (int k = kStart; kcomp(k); k += kadd) {
                    for (int i = iStart; icomp(i); i += iadd) {
                        auto idx = getIdx(glm::ivec3(i, j, k));
                        auto minDist = sdf[idx];
                        auto minIdx = triangleIdx[idx];
                        if (minIdx == -1) continue;

                        for (int l = 0; l < 6; l++) {
                            auto gPos2 = glm::ivec3(i + int(l == 0) - int(l == 1),j + int(l == 2) - int(l == 3),k + int(l == 4) - int(l == 5));
                            if (gPos2.x > gridDim.x - 1 || gPos2.y > gridDim.y - 1 || gPos2.z > gridDim.z - 1 || gPos2.x < 0 || gPos2.y < 0 || gPos2.z < 0) continue;

                            auto idx2 = getIdx(gPos2);
                            auto tIdx = triangleIdx[idx2];
                            if (tIdx != -1) {
                                auto p1 = verts[m_indices[3 * minIdx]];
                                auto p2 = verts[m_indices[3 * minIdx + 1]];
                                auto p3 = verts[m_indices[3 * minIdx + 2]];
                                auto p = glm::vec3(i, j, k) * cellSize;
                                auto dist = distancePointTriangle(p, p1, p2, p3);
                                if (triangleIdx[idx2] != -1 && dist < minDist) {
                                    minIdx = tIdx;
                                    minDist = dist;
                                }
                            }
                        }
                        sdf[idx] = minDist;
                        triangleIdx[idx] = minIdx;
                    }
                }
            }
        }
    }

    // set negative distances
    for (int j = 1; j < gridDim.y - 1; j++) {
        for (int k = 1; k < gridDim.z - 1; k++) {
            uint32_t c = 0;
            for (int i = 1; i < gridDim.x - 1; i++) {
                auto idx = getIdx(glm::ivec3(i, j, k));
                if (c % 2 == 1) sdf[idx] = -sdf[idx];
                c += border[idx];
            }
        }
    }

    m_sdf = std::make_unique<vkb::Buffer>(device,
                                          sizeof(float)*size,
                                          VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT,
                                          VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    vkb::Buffer::writeVectorToBuffer(device, m_sdf, sdf);

}
