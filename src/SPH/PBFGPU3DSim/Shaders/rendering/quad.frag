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
const vec4 defaultColor = vec4(0.2f, 0.6f, 1.0f, 1.0f);

float linearizeDepth(float depth){
    float n = ubo.zNear;
    float f = ubo.zFar;
    float z = depth;
    return (2.0 * n) / (f + n - z * (f - n));
}

vec3 getPos() {
    float z = texture(samplerDepth, inUV).r;
    return vec3(
        (inUV.x*2 - 1)*z*ubo.screenWidth/ubo.screenHeight,
        (inUV.y*2 - 1)*z*ubo.tanHalfFov,
        z
    );
}

vec3 traceColor(vec3 pos, vec3 dir) {
    vec4 worldPos = mat4(ubo.inverseView) * vec4(pos, 1.0);
    vec3 worldDir = mat3(ubo.inverseView) * dir;
    float t = worldPos.z / worldDir.z;
    vec3 world_its = worldPos.xyz + t * worldDir;

//    if (t >= 0 && abs(world_its.x) < 150 && abs(world_its.y) < 150) {
//       return vec3(0.3, 0.3, 0.3);//texture(samplerPlane, world_its.xy).rgb;
//    } else {
        return texture(samplerCubeMap, dir).rgb;
//    }
}

vec4 shadingFresnel() {
    vec3 n = texture(samplerNormals, inUV).xyz;
    vec3 p = getPos();
    vec3 e = -normalize(p);
//    float r = r0 + (1 - r0)*pow(1 - dot(n, e), 3);
//
//    vec3 view_reflect = -e + 2 * n * dot(n, e);
//    vec3 view_refract = -e - 0.2*n;
//
//    float thickness = texture(samplerThick, inUV).r;
//    float attenuate = max(exp(0.5*-thickness), 0.2);
//    vec3 tint_color = vec3(6, 105, 217) / 256;
//     vec3 refract_color = mix(tint_color, traceColor(p, view_refract), 0.99);
////    vec3 refract_color = mix(tint_color, traceColor(p, view_refract), thickness);
//    vec3 reflect_color = traceColor(p, view_reflect);
//
////    return vec4(mix(refract_color, reflect_color, r), 1);
    return vec4(traceColor(p, e), 1);
}

vec4 shadingFresnelScale() {
    vec3 n = texture(samplerNormals, inUV).xyz;
    vec3 e = normalize(-getPos());
    float r = r0 + (1 - r0)*pow(1 - dot(n, e), 2);
    return r*defaultColor;
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
            vec4 thickColor = vec4(texture(samplerThick, inUV).r);
            outFragColor = vec4(vec3(thickColor), 1.0 - thickColor.r);
        break;
        case 3:
            vec4 normal = texture(samplerNormals, inUV);
            outFragColor = vec4(0.5*normal.rgba + vec4(0.5));
        break;
        case 4:
            float smoothDepth = texture(samplerSmoothedDepth, inUV).r;
            outFragColor = vec4(vec3(linearizeDepth(smoothDepth)), 1.0);
        case 5:
            outFragColor = shadingFresnelScale();
        case 6:
            outFragColor = shadingFresnel();
        break;
    }
}