#version 450
#extension GL_GOOGLE_include_directive : require
#extension GL_EXT_debug_printf : enable

#include "common.glsl"

layout(binding = 1) uniform sampler2D texSampler;

layout(location = 0) in vec2 fragTexCoord;
layout(location = 1) in vec3 fragPosWorld;
layout(location = 2) in vec3 fragNormalWorld;

layout(binding = 0) uniform UBO {
    UniformBufferObject ubo;
};

layout(location = 0) out vec4 outColor;

const float AMBIENT = 0.05;

void main() {
    vec3 n = normalize(fragNormalWorld);
    vec3 l = normalize(-ubo.lightDir);
    float lightIntensity = AMBIENT + max(dot(n, l), 0);
    debugPrintfEXT("normal: %f %f %f, lightDir: %f %f %f, lightIntensity: %f\n", n.x, n.y, n.z, l.x, l.y, l.z, dot(n, l));
    outColor = vec4(vec3(texture(texSampler, fragTexCoord))*lightIntensity, 1.0);
}