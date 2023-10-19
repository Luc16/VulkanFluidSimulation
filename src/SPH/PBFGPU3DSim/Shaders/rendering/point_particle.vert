#version 450
#extension GL_GOOGLE_include_directive : require

#include "common.glsl"

layout(location = 0) in vec3 inPosition;

layout(binding = 0) uniform UBO {
    UniformBufferObject ubo;
};

layout(location = 0) out vec4 fragColor;

void main() {

    vec4 posWorld = vec4(inPosition, 1.0);
    vec4 viewPos = ubo.view * posWorld;

    gl_PointSize = ubo.screenHeight*ubo.radius/(viewPos.z*ubo.tanHalfFov);
    gl_Position = ubo.proj * viewPos;

    fragColor = vec4(0.2f, 0.6f, 1.0f, 1.0f);
}