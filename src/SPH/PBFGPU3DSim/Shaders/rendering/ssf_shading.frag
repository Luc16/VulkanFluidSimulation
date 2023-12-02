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
layout (binding = 3) uniform sampler2D samplerScene;
layout (binding = 4) uniform sampler2D samplerSmoothedDepth;
layout (binding = 5) uniform sampler2D samplerPlane;
layout (binding = 6) uniform samplerCube samplerCubeMap;


layout (location = 0) out vec4 outFragColor;

const float r0 = 0.142857;
const vec3 defaultColor = vec3(6, 105, 217) / 256;
const float attenuateConst = 0.8;
const float fresnel_bias = 0;
const float fresnel_scale = 0.8f;
const float fresnel_power = 20.0f;

float linearizeDepth(float depth){
    float n = ubo.zNear;
    float f = ubo.zFar;
    float z = depth;
    return (2.0 * n) / (f + n - z * (f - n));
}

float getDepth(vec2 uv) {
//    return ubo.zNear + linearizeDepth(texture(samplerSmoothedDepth, uv).r)*(ubo.zFar - ubo.zNear);
//    return linearizeDepth(texture(samplerSmoothedDepth, uv).r);

    float q = ubo.zFar/(ubo.zFar - ubo.zNear);
    return -ubo.zNear*q/(texture(samplerSmoothedDepth, uv).r - q);
//    return ubo.zNear + texture(samplerSmoothedDepth, uv).r*(ubo.zFar - ubo.zNear);
//    return 0.5 * (ubo.zFar + ubo.zNear + texture(samplerSmoothedDepth, uv).r*(ubo.zFar - ubo.zNear));
}


float getThickness(vec2 uv) {
    float thickness = texture(samplerThick, inUV).r;
    return clamp(exp(attenuateConst*-thickness), 0.01, 1.0);
}

vec3 toEyeSpace(vec2 uv) {
    float z = getDepth(uv);
    return vec3(
        -(uv.x*2.0 - 1.0)*z*ubo.tanHalfFov*ubo.screenWidth/ubo.screenHeight,
        -(uv.y*2.0 - 1.0)*z*ubo.tanHalfFov,
        z
    );
}

float fresnelScale(vec3 dir, vec3 normal) {
    // http://developer.download.nvidia.com/CgTutorial/cg_tutorial_chapter07.html
//    return clamp(fresnel_bias + fresnel_scale * pow(1 + dot(dir, normal), fresnel_power), 0, 1);
//    return clamp(r0 + (1 - r0) * pow(1 - dot(dir, normal), 10), 0, 1);
    return 0.1 + (1.0 - 0.1)*pow(1.0-max(dot(normal, -dir), 0.0), 3);
}

vec3 traceColor(vec4 pos, vec3 dir) {
    float t = -pos.y / dir.y;
    vec3 planeIntersect = pos.xyz + t * dir;

    if (t >= 0 && planeIntersect.x > 0 && planeIntersect.x < ubo.planeSize.x && planeIntersect.z > 0 && planeIntersect.z < ubo.planeSize.z) {
        return texture(samplerPlane, vec2(planeIntersect.x/ubo.planeSize.x, planeIntersect.z/ubo.planeSize.z)).rgb;
    } else {
        dir.z *= -1; // flip z for cube map
        return texture(samplerCubeMap, dir).rgb;
    }
}

void main()
{
    float depth = texture(samplerSmoothedDepth, inUV).r;
    if (depth == 1) {
        discard;
    }
    gl_FragDepth = depth;

    if (ubo.renderType == 1) {
        outFragColor = vec4(vec3(linearizeDepth(depth)), 1.0);
        return;
    } else if (ubo.renderType == 2) {
        outFragColor = vec4(getThickness(inUV));
        return;
    }  else if (ubo.renderType == 4) {
//        float smoothDepth = texture(samplerSmoothedDepth, inUV).r;
//        outFragColor = vec4(vec3(linearizeDepth(smoothDepth)), 1.0);
        outFragColor = texture(samplerScene, inUV);
        return;
    }

    vec3 pos = toEyeSpace(inUV);
    float dx = 1 / ubo.screenWidth;
    float dy = 1 / ubo.screenHeight;

    vec3 ddx = toEyeSpace(vec2(inUV.x + dx, inUV.y)) - pos;
    vec3 ddx2 = pos - toEyeSpace(vec2(inUV.x - dx, inUV.y));
    if (abs(ddx.z) > abs(ddx2.z)) {
        ddx = ddx2;
    }

    vec3 ddy = toEyeSpace(vec2(inUV.x, inUV.y + dy)) - pos;
    vec3 ddy2 = pos - toEyeSpace(vec2(inUV.x, inUV.y - dy));
    if (abs(ddy.z) > abs(ddy2.z)) {
        ddy = ddy2;
    }
    vec3 normal = cross(ddx, ddy);
    normal = normalize(normal);

    if (ubo.renderType == 3) {
        outFragColor = vec4(0.5*normal.rgb + vec3(0.5), 1.0);
        return;
    }


    vec3 dir = -normalize(pos);
    vec4 worldPos = ubo.inverseView * vec4(pos, 1.0);
    vec3 worldDir = mat3(ubo.inverseView) * dir;

    float r = fresnelScale(dir, normal);
//    r = (r == 1) ? 0.25 : r;

    vec3 reflectedDir = worldDir - 2 * normal * dot(normal, worldDir);

    float ior = 1.333;
    float refractScale = ior*0.025;
    // attenuate refraction near ground (hack)
    refractScale *= smoothstep(0.1, 0.4, worldPos.y);
    vec2 texScale = vec2(0.75, 1.0);	// to account for backbuffer aspect ratio

    vec2 refractCoord = inUV + normal.xy*refractScale*texScale;

    vec3 refractColor = mix(defaultColor, texture(samplerScene, refractCoord).rgb, getThickness(inUV));
    vec3 reflectColor = traceColor(worldPos, reflectedDir);


    if (ubo.renderType == 5) {
        outFragColor = vec4(reflectColor, 1.0);
    } else if (ubo.renderType == 6) {
        outFragColor = vec4(refractColor, 1.0);
    } else if (ubo.renderType == 7) {
        outFragColor = vec4(vec3(r), 1.0);
    } else {
        outFragColor = vec4(mix(refractColor, reflectColor, r), 1.0);
    }
}