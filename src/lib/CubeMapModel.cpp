//
// Created by luc on 24/10/23.
//

#include "CubeMapModel.h"
#include "../../external/stbimage/stb_image.h"

namespace vkb {
    CubeMapModel::CubeMapModel(const Device &device, const std::vector<std::string>& filePaths, bool flip): m_deviceRef(device), m_flipImage(flip) {
        createCubeMapImage(filePaths);
        createVertexBuffer();
    }

    CubeMapModel::~CubeMapModel() {
        vkDestroySampler(m_deviceRef.device(), m_cubeMapSampler, nullptr);
    }

    void CubeMapModel::createCubeMapImage(const std::vector<std::string>& filePaths) {
        std::array<stbi_uc*, NUM_CUBEMAP_IMAGES> localBuffers{};
        int texWidth, texHeight, bitsPerPixel;
        stbi_set_flip_vertically_on_load(m_flipImage);
        for (uint32_t i = 0; i < NUM_CUBEMAP_IMAGES; ++i) {
//            std::cout << "Loading : " << filePaths[i] << "\n";
            localBuffers[i] = stbi_load(filePaths[i].c_str(), &texWidth, &texHeight, &bitsPerPixel, STBI_rgb_alpha);
//            std::cout << "i: " << i << " width: " << texWidth << " height: " << texHeight << " bpp: " << bitsPerPixel << "\n";
            if (!localBuffers[i])
                throw std::runtime_error("failed to load cube map texture: " + std::to_string(i) + "!");

        }
        std::array<VkDeviceSize, NUM_CUBEMAP_IMAGES> bufferSizes{};

        VkDeviceSize layerSize = texWidth * texHeight * 4;
        VkDeviceSize imageSize = layerSize * NUM_CUBEMAP_IMAGES;

        vkb::Buffer stagingBuffer(m_deviceRef, imageSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                                  VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);

        stagingBuffer.map();
        for (uint32_t i = 0; i < NUM_CUBEMAP_IMAGES; ++i) {
            stagingBuffer.write(localBuffers[i], layerSize, i*layerSize);
        }
        stagingBuffer.unmap();

        // for now cube map only works for color images
        m_cubeMapImage = std::make_unique<vkb::Image>(m_deviceRef, texWidth, texHeight, 1, VK_SAMPLE_COUNT_1_BIT,
                                                      VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_TILING_OPTIMAL,
                                                      VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT ,
                                                      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, VK_IMAGE_ASPECT_COLOR_BIT,
                                                      false,VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT, NUM_CUBEMAP_IMAGES);

        m_cubeMapImage->transitionLayout(VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_UNDEFINED,
                                         VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1);

        m_deviceRef.copyBufferToImage(stagingBuffer.getBuffer(), m_cubeMapImage->image(),
                                      static_cast<uint32_t>(texWidth), static_cast<uint32_t>(texHeight),
                                      NUM_CUBEMAP_IMAGES);

        m_cubeMapImage->transitionLayout(VK_FORMAT_R8G8B8A8_SRGB, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                         VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, 1, true);

        createCubeMapSampler();

        VkImageViewCreateInfo viewInfo{};
        viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        viewInfo.image = m_cubeMapImage->image();
        viewInfo.viewType = VK_IMAGE_VIEW_TYPE_CUBE;
        viewInfo.components = {VK_COMPONENT_SWIZZLE_R, VK_COMPONENT_SWIZZLE_G, VK_COMPONENT_SWIZZLE_B, VK_COMPONENT_SWIZZLE_A};
        viewInfo.format = VK_FORMAT_R8G8B8A8_SRGB;
        viewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
        viewInfo.subresourceRange.baseMipLevel = 0;
        viewInfo.subresourceRange.levelCount = 1;
        viewInfo.subresourceRange.baseArrayLayer = 0;
        viewInfo.subresourceRange.layerCount = NUM_CUBEMAP_IMAGES;

        m_cubeMapImage->createImageView(viewInfo);
        for (auto & localBuffer : localBuffers) {
            stbi_image_free(localBuffer);
        }

    }

    void CubeMapModel::createCubeMapSampler() {
        VkSamplerCreateInfo samplerInfo{};
        samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
        samplerInfo.magFilter = VK_FILTER_LINEAR;
        samplerInfo.minFilter = VK_FILTER_LINEAR;
        samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
        samplerInfo.anisotropyEnable = VK_TRUE;

        VkPhysicalDeviceProperties properties{};
        vkGetPhysicalDeviceProperties(m_deviceRef.physicalDevice(), &properties);
        samplerInfo.maxAnisotropy = properties.limits.maxSamplerAnisotropy;

        samplerInfo.borderColor = VK_BORDER_COLOR_FLOAT_OPAQUE_WHITE;
        samplerInfo.unnormalizedCoordinates = VK_FALSE;
        samplerInfo.compareEnable = VK_FALSE;
        samplerInfo.compareOp = VK_COMPARE_OP_NEVER;
        samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_LINEAR;
        samplerInfo.mipLodBias = 0.0f;
        samplerInfo.minLod = 0.0f;
        samplerInfo.maxLod = 1.0f;

        if (vkCreateSampler(m_deviceRef.device(), &samplerInfo, nullptr, &m_cubeMapSampler) != VK_SUCCESS) {
            throw std::runtime_error("failed to create texture sampler!");
        }
    }

    void CubeMapModel::createVertexBuffer() {
        VkDeviceSize bufferSize = sizeof(vertices[0]) * NUM_VERTICES;
        m_vertexBuffer = std::make_unique<vkb::Buffer>(m_deviceRef, bufferSize,
                                                       VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
                                                       VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

        vkb::Buffer::writeVectorToBuffer(m_deviceRef, m_vertexBuffer, vertices);
    }

    void CubeMapModel::bindAndDraw(VkCommandBuffer commandBuffer) const {
        VkBuffer vertexBuffers[] = {m_vertexBuffer->getBuffer()};
        VkDeviceSize offsets[] = {0};
        vkCmdBindVertexBuffers(commandBuffer, 0, 1, vertexBuffers, offsets);
        vkCmdDraw(commandBuffer, NUM_VERTICES, 1, 0, 0);

    }
}


