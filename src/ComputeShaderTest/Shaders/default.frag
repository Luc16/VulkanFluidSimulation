#version 450

layout(location = 0) in vec3 fragColor;

layout(location = 0) out vec4 outColor;

void main() {

    vec2 coord = gl_PointCoord - vec2(0.5);
    const float radius = 0.25;
    if (length(gl_PointCoord - vec2(0.5)) > radius) {
        discard;
    }
//    outColor = vec4(fragColor, 1);
    outColor = vec4(fragColor, 0.8 - length(coord));
}