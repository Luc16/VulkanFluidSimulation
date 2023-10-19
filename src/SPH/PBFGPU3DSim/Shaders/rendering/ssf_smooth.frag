#version 450
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_debug_printf : enable


#include "common.glsl"

layout (location = 0) in vec2 inUV;

layout(binding = 0) uniform UBO {
    UniformBufferObject ubo;
};

// int filterRadius
// blurScale
// blurDepthFalloff

 int filterRadius = 2;
 float blurScale = 0.2;
 float blurDepthFalloff = 2;

layout (binding = 1) uniform sampler2D samplerDepth;

float getDepth(ivec2 texPos) {
    return texelFetch(samplerDepth, texPos, 0).r;
}

float bilateral(ivec2 texPos) {
    float z = getDepth(texPos);
    float sum = 0;
    float wsum = 0;
    for(int dx = -filterRadius; dx <= filterRadius; dx++) {
        for(int dy = -filterRadius; dy <= filterRadius; dy++) {
            ivec2 dPos = ivec2(dx, dy);
            float curZ = getDepth(texPos + dPos);
            // spatial domain
            float w = exp(-dot(dPos, dPos)*blurScale*blurScale);
            // range domain
            float r2 = (curZ - z) * blurDepthFalloff;
            float g = exp(-r2*r2);

            float wg = w * g;
            sum += curZ * wg;
            wsum += wg;
        }
    }
    if (wsum > 0.0) sum /= wsum;
    return sum;
}

float gaussian(ivec2 texPos) {
    float z = getDepth(texPos);
    float sum = 0, wsum = 0;

    for (int dx = -filterRadius; dx <= filterRadius; dx++) {
        for (int dy = -filterRadius; dy <= filterRadius; dy++) {
            ivec2 dPos = ivec2(dx, dy);
            float curZ = getDepth(texPos + dPos);
            float w = exp(-dot(dPos, dPos)*blurScale*blurScale);

            sum += curZ * w;
            wsum += w;
        }
    }

    if (wsum > 0) sum /= wsum;
    return sum;
}

void main()
{
    float smoothDepth = bilateral(ivec2(gl_FragCoord));
    gl_FragDepth = smoothDepth;
}