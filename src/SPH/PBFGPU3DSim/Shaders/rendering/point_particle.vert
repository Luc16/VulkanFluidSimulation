#version 450
#extension GL_EXT_debug_printf : enable
#extension GL_GOOGLE_include_directive : require

#include "common.glsl"

layout(location = 0) in vec3 inPosition;
layout(location = 1) in float density;
layout(location = 2) in uint type;

layout(binding = 0) uniform UBO {
    UniformBufferObject ubo;
};

layout(location = 0) out vec4 fragColor;

void main() {

    vec4 posWorld = vec4(inPosition, 1.0);
    vec4 viewPos = ubo.view * posWorld;

    gl_PointSize = ubo.screenHeight*ubo.radius/(viewPos.z*ubo.tanHalfFov);
    gl_Position = ubo.proj * viewPos;

    if (density < 2000) {
        fragColor = vec4(0.2f, 0.6f, 1.0f, 1);
    } else if (density > ubo.restDens) {
        fragColor = (2 - density/ubo.restDens)*vec4(1, 0, 0, 1);
    } else {
        fragColor = density/ubo.restDens * vec4(1, 1, 1, 1);
    }

    if (type != 0) {
        fragColor = vec4(0, 1, 0, 1);
    } else {
        fragColor = vec4(0, 0, 0, 0);
    }
}