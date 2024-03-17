#version 450
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_debug_printf : enable


#include "common.glsl"

layout (location = 0) in vec2 inUV;

layout(binding = 0) uniform UBO {
    UniformBufferObject ubo;
};

layout (binding = 1) uniform sampler2D samplerDepth;

float getDepth(ivec2 texPos) {
    return texelFetch(samplerDepth, texPos, 0).r;
}

float bilateral(ivec2 texPos) {
    float z = getDepth(texPos);

    float sum = 0;
    float wsum = 0;
    for(int dx = -ubo.filterRadius; dx <= ubo.filterRadius; dx++) {
        for(int dy = -ubo.filterRadius; dy <= ubo.filterRadius; dy++) {
            ivec2 dPos = ivec2(dx, dy);
            float curZ = getDepth(texPos + dPos);
            // spatial domain
            float w = exp(-dot(dPos, dPos)*ubo.blurScale*ubo.blurScale);
            // range domain
            float r2 = (curZ - z) * ubo.blurDepthFalloff;
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

    for (int dx = -ubo.filterRadius; dx <= ubo.filterRadius; dx++) {
        for (int dy = -ubo.filterRadius; dy <= ubo.filterRadius; dy++) {
            ivec2 dPos = ivec2(dx, dy);
            float curZ = getDepth(texPos + dPos);
            float w = exp(-dot(dPos, dPos)*ubo.blurScale*ubo.blurScale);

            sum += curZ * w;
            wsum += w;
        }
    }

    if (wsum > 0) sum /= wsum;
    return sum;
}

// https://www.shadertoy.com/view/4dfGDH
#define BSIGMA 0.001
#define MSIZE 15
const float kernel[MSIZE] = float[MSIZE](0.031225216, 0.033322271, 0.035206333, 0.036826804, 0.038138565, 0.039104044, 0.039695028, 0.039894000, 0.039695028, 0.039104044, 0.038138565, 0.036826804, 0.035206333, 0.033322271, 0.031225216);
const int k_size = (MSIZE - 1) / 2;

float normpdf(in float x, in float sigma) {
    return 0.39894*exp(-0.5*x*x/(sigma*sigma))/sigma;
}

float bilateral2(ivec2 texPos) {
    float z = getDepth(texPos);

    float wsum = 0;
    float sum = 0;
    float bZ = 1.0 / normpdf(0.0, BSIGMA);
    for (int dx = -k_size; dx <= k_size; ++dx) {
        for (int dy = -k_size; dy <= k_size; ++dy) {
            ivec2 dPos = ivec2(dx, dy);
            float curZ = getDepth(texPos + dPos);
            float w = normpdf(curZ - z, BSIGMA) * bZ * kernel[k_size + dy] * kernel[k_size + dx];
            wsum += w;
            sum += w * curZ;
        }
    }
    if (wsum > 0.0) sum /= wsum;
    return sum;

}

void main()
{
    if (getDepth(ivec2(gl_FragCoord)) == 1) {
        discard;
    }
    float smoothDepth;
    switch (ubo.blurMode) {
        case 0:
            smoothDepth = bilateral(ivec2(gl_FragCoord));
        break;
        case 1:
            smoothDepth = gaussian(ivec2(gl_FragCoord));
        break;
        case 2:
            smoothDepth = bilateral2(ivec2(gl_FragCoord));
        break;
        default:
            smoothDepth = bilateral(ivec2(gl_FragCoord));


    }
    gl_FragDepth = smoothDepth;
}