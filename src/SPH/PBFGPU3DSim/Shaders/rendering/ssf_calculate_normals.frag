#version 450
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_debug_printf : enable


#include "common.glsl"

layout (location = 0) in vec2 inUV;

layout(binding = 0) uniform UBO {
    UniformBufferObject ubo;
};

layout (binding = 1) uniform sampler2D samplerDepth;

layout (location = 0) out vec4 outNormal;

float c_x = 4 * ubo.tanHalfFov / ubo.screenWidth;
float c_y = 4 * ubo.screenWidth * ubo.tanHalfFov / (ubo.screenHeight * ubo.screenHeight);

float linearizeDepth(float depth)
{
    float n = ubo.zNear;
    float f = ubo.zFar;
    float z = depth;
//    return 0.5 * (f + n + z * (f - n));
    return n + z * (f - n);
}

float getDepth(float x, float y) {
    return linearizeDepth(texture(samplerDepth, vec2(x, y)).r);
}

void main()
{
    if (texture(samplerDepth, inUV).r == 1.0) {
        if (ubo.renderType == 3) {
            outNormal = vec4(0, 0, 0, -1);
            return;
        } else {
            discard;
        }
    }
    vec3 pos = vec3(inUV, getDepth(inUV.x, inUV.y));
    float x = inUV.x;
    float y = inUV.y;

    float dx = 1 / ubo.screenWidth;
    float dy = 1 / ubo.screenHeight;

    vec3 ddx = vec3(pos.x + dx, pos.y, getDepth(pos.x + dx, pos.y)) - pos;
    vec3 ddx2 = pos - vec3(pos.x - dx, pos.y, getDepth(pos.x - dx, pos.y));
    if (abs(ddx.z) > abs(ddx2.z)) {
        ddx = ddx2;
    }

    vec3 ddy = vec3(pos.x, pos.y + dy, getDepth(pos.x, pos.y + dy)) - pos;
    vec3 ddy2 = pos - vec3(pos.x, pos.y - dy, getDepth(pos.x, pos.y - dy));
    if (abs(ddy.z) > abs(ddy2.z)) {
        ddy = ddy2;
    }
    vec3 normal = cross(ddx, ddy);

    float len = length(normal);

//    outNormal = vec4(vec3(normal.z/len), texture(samplerDepth, inUV).r);
    outNormal = vec4(normal/len, len);
}