//
// Created by luc on 28/10/22.
//
#define TINYOBJLOADER_IMPLEMENTATION
#define GLM_ENABLE_EXPERIMENTAL
#include "Model.h"
#include "../../external/objloader/tiny_obj_loader.h"
#include <glm/gtx/hash.hpp>

namespace std {
    template <>
    struct hash<vkb::Model::Vertex> {
        size_t operator()(vkb::Model::Vertex const &vertex) const {
            size_t seed = 0;
            hashCombine(seed, vertex.pos, vertex.color, vertex.normal, vertex.texCoord);
            return seed;
        }
    };
}

namespace vkb {

    Model::Model(const Device &device, std::vector<Vertex>& vertices, std::vector<uint32_t>& indices): m_deviceRef(device) {
        m_hasIndexBuffer = true;
        updateVertexBuffer(vertices, true);
        createIndexBuffer(indices);
    }

    Model::Model(const Device &device, std::vector<Vertex>& vertices): m_deviceRef(device) {
        updateVertexBuffer(vertices, true);
    }

    void Model::updateVertexBuffer(std::vector<Vertex>& vertices, bool createBuffer) {
        VkDeviceSize bufferSize = sizeof(vertices[0]) * vertices.size();
        m_vertexCount = static_cast<uint32_t>(vertices.size());

        vkb::Buffer stagingBuffer(m_deviceRef, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                  VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        stagingBuffer.singleWrite(vertices.data());

        if (createBuffer)
            m_vertexBuffer = std::make_unique<vkb::Buffer>(m_deviceRef, bufferSize,
                                                           VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                                                           VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        m_deviceRef.copyBuffer(stagingBuffer.getBuffer(), m_vertexBuffer->getBuffer(), bufferSize);
    }

    void Model::createIndexBuffer(std::vector<uint32_t>& indices){
        VkDeviceSize bufferSize = sizeof(indices[0]) * indices.size();
        m_indexCount = static_cast<uint32_t>(indices.size());

        vkb::Buffer stagingBuffer(m_deviceRef, bufferSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                  VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        stagingBuffer.singleWrite(indices.data());

        m_indexBuffer = std::make_unique<vkb::Buffer>(m_deviceRef, bufferSize,
                                                    VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
                                                    VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        m_deviceRef.copyBuffer(stagingBuffer.getBuffer(), m_indexBuffer->getBuffer(), bufferSize);
    }

    std::unique_ptr<Model> Model::createModelFromFile(const Device &device, const std::string &filepath) {
        std::vector<Vertex> vertices{};
        std::vector<uint32_t> indices{};

        tinyobj::attrib_t attrib;
        std::vector<tinyobj::shape_t> shapes;
        std::vector<tinyobj::material_t> materials;
        std::string warn, err;

        if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filepath.c_str())) {
            throw std::runtime_error(warn + err);
        }

        std::unordered_map<Vertex, uint32_t> uniqueVertices{};

        for (const auto& shape: shapes) {
            for (const auto& index : shape.mesh.indices) {
                Vertex vertex{};
                vertex.pos = {
                        attrib.vertices[3 * index.vertex_index + 0],
                        attrib.vertices[3 * index.vertex_index + 1],
                        attrib.vertices[3 * index.vertex_index + 2],
                };
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
                }

                indices.push_back(uniqueVertices[vertex]);
            }
        }
        return std::make_unique<Model>(device, vertices, indices);

    }

    void Model::bind(VkCommandBuffer commandBuffer) {
        VkBuffer vertexBuffers[] = {m_vertexBuffer->getBuffer()};
        VkDeviceSize offsets[] = {0};
        vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers, offsets);
        if (m_hasIndexBuffer)
            vkCmdBindIndexBuffer(commandBuffer, m_indexBuffer->getBuffer(), 0, VK_INDEX_TYPE_UINT32);
    }

    void Model::draw(VkCommandBuffer commandBuffer, size_t numInstances) const {
        if (m_hasIndexBuffer)
            vkCmdDrawIndexed(commandBuffer, m_indexCount, numInstances, 0, 0, 0);
        else
            vkCmdDraw(commandBuffer, m_vertexCount, numInstances, 0, 0);
    }

    VkVertexInputBindingDescription Model::Vertex::getBindingDescription() {
        VkVertexInputBindingDescription bindingDescription{};
        bindingDescription.binding = 0;
        bindingDescription.stride = sizeof(Vertex);
        bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;

        return bindingDescription;
    }

    std::vector<VkVertexInputAttributeDescription> Model::Vertex::getAttributeDescriptions() {
        std::vector<VkVertexInputAttributeDescription> attributeDescriptions{};
        attributeDescriptions.push_back({0, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, pos)});
        attributeDescriptions.push_back({1, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, color)});
        attributeDescriptions.push_back({2, 0, VK_FORMAT_R32G32B32_SFLOAT, offsetof(Vertex, normal)});
        attributeDescriptions.push_back({3, 0, VK_FORMAT_R32G32_SFLOAT, offsetof(Vertex, texCoord)});

        return attributeDescriptions;
    }


}