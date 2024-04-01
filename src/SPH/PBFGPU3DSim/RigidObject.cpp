//
// Created by luc on 20/11/23.
//


#include <unordered_set>
#include "RigidObject.h"
#include "../../../external/objloader/tiny_obj_loader.h"

RigidObject::RigidObject(const vkb::Device& device, const std::string& modelFile, const std::shared_ptr<vkb::Texture>& tex,float scale, float particleRadius) {
    std::vector<vkb::Model::Vertex> vertices{};
    std::vector<uint32_t> indices{};

    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::string warn, err;

    if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, modelFile.c_str())) {
        throw std::runtime_error(warn + err);
    }

    std::unordered_map<vkb::Model::Vertex, uint32_t> uniqueVertices{};
    std::unordered_set<glm::vec3> particlePositionSet;

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
            if (index.texcoord_index > 0)
                vertex.texCoord = {
                        attrib.texcoords[2 * index.texcoord_index],
                        1.0f - attrib.texcoords[2 * index.texcoord_index + 1],
                };

            if (uniqueVertices.count(vertex) == 0){
                uniqueVertices[vertex] = static_cast<uint32_t>(vertices.size());
                vertices.push_back(vertex);
                particlePositionSet.insert(vertex.pos);
            }

            indices.push_back(uniqueVertices[vertex]);
        }
    }

    for (uint32_t i = 0; i < indices.size()/3; i++) {

        auto vec = vertices[indices[3*i + 1]].pos - vertices[indices[3*i]].pos;
        auto len = glm::length(vec);
        auto line = (len - 2*particleRadius)/(2*particleRadius);
        if (line < 0.0) continue;
        vec = vec/len;

        auto vec2 = vertices[indices[3*i + 2]].pos - vertices[indices[3*i + 1]].pos;
        auto len2 = glm::length(vec2);
        auto line2 = (len2 - 2*particleRadius)/(2*particleRadius);
        if (line2 < 0.0) continue;
        vec2 = vec2/len2;

        for (uint32_t j = 0; j < uint32_t(line + 3); j++){
            auto s = vertices[indices[3*i+1]].pos - vec*(float(j)*(len - 2*particleRadius)/std::floor(line + 1));
            for (uint32_t k = 0; k < uint32_t(line2 + 2); k++){

                if (j == k && j == 0 || k == 0 && j == uint32_t(line + 2)) continue;

                auto t = s + vec2*(float(k)*(len2 - 2*particleRadius)/std::floor(line2 + 1));

                auto normal = glm::cross(vec, vec2);
                auto vec3 = vertices[indices[3*i + 2]].pos - vertices[indices[3*i]].pos;
                auto otherPoint = t - vertices[indices[3*i]].pos;
                auto crossLinePoint = glm::cross(vec3, otherPoint);

                if (glm::dot(glm::cross(vec3, vec), normal)*glm::dot(crossLinePoint, normal) > 0 ||
                glm::length(crossLinePoint)/glm::length(vec3) < 0.8f*particleRadius) {
                    particlePositionSet.insert(t);
                }

            }
        }
    }

    m_particlePositions.assign( particlePositionSet.begin(), particlePositionSet.end() );
    m_object = std::make_unique<vkb::DrawableObject>(std::make_unique<vkb::Model>(device, vertices, indices), tex);

}


void RigidObject::translate(const glm::vec3 &move) {
    if (glm::dot(move, move) == 0) return;

    m_object->translate(move);
    std::for_each(m_particlePositions.begin(), m_particlePositions.end(), [&move](glm::vec3& vec){
        vec += move;
    });

}