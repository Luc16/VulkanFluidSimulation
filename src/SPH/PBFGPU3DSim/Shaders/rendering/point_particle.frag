#version 450
#extension GL_EXT_debug_printf : enable

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

//    float depth = (((1000-0.1) * fragDepth) + 0.1 + 1000) / 2000.0;

//    debugPrintfEXT("eye: (%f, %f, %f, %f) -> normal: (%f, %f, %f) -> pixel: (%f, %f, %f)\n", eyePos.x, eyePos.y, eyePos.z, eyePos.w, no.x, no.y, no.z, pixelPos.x, pixelPos.y, pixelPos.z);

    // calculate lighting
    float diffuse = AMBIENT + max(0.0, dot(normal, ubo.lightDir));

    if (ubo.renderType == RENDER_DEPTH) {
        outColor = vec4(vec3(fragDepth), 1);
    }
    else {
        outColor = fragColor*diffuse;
    }

}