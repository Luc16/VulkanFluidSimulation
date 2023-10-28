#version 450
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_debug_printf : enable


#include "common.glsl"

layout (location = 0) in vec2 inUV;

layout(binding = 0) uniform UBO {
    UniformBufferObject ubo;
};

layout (binding = 1) uniform sampler2D samplerDepth;
layout (binding = 2) uniform sampler2D samplerThick;
layout (binding = 3) uniform sampler2D samplerNormals;
layout (binding = 4) uniform sampler2D samplerSmoothedDepth;
layout (binding = 5) uniform sampler2D samplerPlane;
layout (binding = 6) uniform samplerCube samplerCubeMap;


layout (location = 0) out vec4 outFragColor;

const float r0 = 0.142857;
const vec3 defaultColor = vec3(6, 105, 217) / 256;
const float attenuateConst = 0.01;
const float fresnel_bias = 0;
const float fresnel_scale = 100;
const float fresnel_power = 2.5;

float linearizeDepth(float depth){
    float n = ubo.zNear;
    float f = ubo.zFar;
    float z = depth;
    return (2.0 * n) / (f + n - z * (f - n));
}

float getDepth(vec2 uv) {
//    return ubo.zNear + linearizeDepth(texture(samplerDepth, uv).r)*(ubo.zFar - ubo.zNear);
//    return linearizeDepth(texture(samplerDepth, uv).r);
//    return ubo.zNear + texture(samplerDepth, uv).r*(ubo.zFar - ubo.zNear);
    return 0.5 * (ubo.zFar + ubo.zNear + texture(samplerDepth, uv).r*(ubo.zFar - ubo.zNear));
}

float getThickness(vec2 uv) {
    float thickness = texture(samplerThick, inUV).r;
    return clamp(exp(attenuateConst*-thickness), 0.01, 1.0);
}

vec3 getPos() {
    float z = getDepth(inUV);
    return vec3(
        (inUV.x*2 - 1)*z*ubo.tanHalfFov*ubo.screenWidth/ubo.screenHeight,
        (inUV.y*2 - 1)*z*ubo.tanHalfFov,
        z
    );
}

float fresnelScale(vec3 dir, vec3 normal) {
    // http://developer.download.nvidia.com/CgTutorial/cg_tutorial_chapter07.html
    return clamp(fresnel_bias + fresnel_scale * pow(1 + dot(dir, normal), fresnel_power), 0, 1);
//    return clamp(r0 + (1 - r0) * pow(1 - dot(-dir, normal), 5), 0, 1);
}

vec3 traceColor(vec4 pos, vec3 dir) {
    float t = -pos.y / dir.y;
    pos.z *= -1;
    vec3 world_its = pos.xyz + t * dir;

    if (t >= 0 && world_its.x > 0 && world_its.x < ubo.planeSize.x && world_its.z < 0 && world_its.z > -ubo.planeSize.z) {
        return texture(samplerPlane, vec2(world_its.x, -world_its.z)/150).rgb;
    } else {
        return texture(samplerCubeMap, dir).rgb;
    }
}

vec4 shadingFresnel() {
    vec3 n = texture(samplerNormals, inUV).xyz;
    vec3 p = getPos();
    vec3 dir = -normalize(p);
    vec4 worldPos = ubo.inverseView * vec4(p, 1.0);
    vec3 worldDir = mat3(ubo.inverseView) * dir;
    worldDir.z *= -1;

    // http://developer.download.nvidia.com/CgTutorial/cg_tutorial_chapter07.html
    float r = fresnelScale(dir, n);
    r = (r == 1) ? 0.3 : r;

    vec3 reflectedDir = -worldDir + 2 * n * dot(n, worldDir);
    vec3 refractedDir = worldDir - 0.02*n;

    vec3 refractColor = mix(defaultColor, traceColor(worldPos, refractedDir), getThickness(inUV));
    vec3 reflectColor = traceColor(worldPos, reflectedDir);
//
    return vec4(mix(refractColor, reflectColor, r), 1.0);
//    return vec4(reflectColor, 1);
//    return vec4(refractColor, 1);
}

vec4 shadingFresnelScale() {
    vec3 n = texture(samplerNormals, inUV).xyz;
    vec3 dir = normalize(-getPos());
    vec3 worldDir = mat3(ubo.inverseView) * dir;
    worldDir.z *= -1;

    float r = fresnelScale(dir, n);

    return vec4(vec3(r), 1.0);
}

void main()
{
    float depth = texture(samplerDepth, inUV).r;
    if (depth == 1) {
        discard;
    }
    gl_FragDepth = -depth;
    switch(ubo.renderType) {
        case 1:
            outFragColor = vec4(vec3(linearizeDepth(depth)), 1.0);
        break;
        case 2:
            outFragColor = vec4(getThickness(inUV));
        break;
        case 3:
            vec4 normal = texture(samplerNormals, inUV);
            outFragColor = vec4(0.5*normal.rgba + vec4(0.5));
        break;
        case 4:
            float smoothDepth = texture(samplerSmoothedDepth, inUV).r;
            outFragColor = vec4(vec3(linearizeDepth(smoothDepth)), 1.0);
        break;
        case 5:
            outFragColor = shadingFresnelScale();
        break;
        case 6:
            outFragColor = shadingFresnel();
        break;
    }
}