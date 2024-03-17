#version 450
#extension GL_GOOGLE_include_directive : require

#include "common.glsl"

layout (location = 0) in vec3 inPos;

layout(binding = 0) uniform UBO {
    UniformBufferObject ubo;
};

void main()
{
    vec4 posWorld = vec4(inPos, 1.0);
    vec4 viewPos = ubo.view * posWorld;

    gl_PointSize = ubo.screenHeight*ubo.radius/(viewPos.z*ubo.tanHalfFov);
    gl_Position = ubo.proj * viewPos;
}