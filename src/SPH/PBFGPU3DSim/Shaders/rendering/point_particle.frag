#version 450
#extension GL_EXT_debug_printf : enable

layout(binding = 1) uniform sampler2D depthSampler;

layout(location = 0) in vec3 worldPos;
layout(location = 1) in vec4 fragColor;

layout(location = 0) out vec4 outColor;

layout(binding = 0) uniform UniformBufferObject {
    mat4 view;
    mat4 proj;
    vec3 cameraPos;
    vec3 lightDir;
    float radius;
    uint renderType;
    float zNear;
    float zFar;
} ubo;

const float AMBIENT = 0.05;

const uint RENDER_DEPTH = 0x1;

void main() {
    vec3 normal;
    normal.xy = gl_PointCoord*2.0 - vec2(1.0);
    float mag = dot(normal.xy, normal.xy);
    if (mag > 1.0) discard;   // kill pixels outside circle

    normal.z = -sqrt(1.0-mag);

    vec4 eyePos = ubo.view * vec4(worldPos, 1);
    vec3 no = 100*normal*ubo.radius;
    vec4 pixelPos = vec4(eyePos.xyz + no, 1.0);
    vec4 clipSpacePos = ubo.proj * eyePos;

    float fragDepth = clipSpacePos.z / 500;

    // calculate lighting
    float diffuse = AMBIENT + max(0.0, dot(normal, ubo.lightDir));

    if (ubo.renderType == RENDER_DEPTH) {
        outColor = vec4(vec3(fragDepth), 1);
    }
    else {
        outColor = fragColor*diffuse;
    }

}