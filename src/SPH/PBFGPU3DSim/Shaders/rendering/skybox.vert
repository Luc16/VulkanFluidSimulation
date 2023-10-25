#version 450
#extension GL_GOOGLE_include_directive : require

#include "common.glsl"

layout(location = 0) in vec3 inPosition;

layout(binding = 0) uniform UBO {
    UniformBufferObject ubo;
};

layout(location = 0) out vec3 fragUVW;

void main() {

    vec4 posWorld = vec4(inPosition, 1.0);
    gl_Position = ubo.proj * mat4(mat3(ubo.view)) * posWorld;

    fragUVW = vec3(inPosition);
    fragUVW.xy *= -1;
}