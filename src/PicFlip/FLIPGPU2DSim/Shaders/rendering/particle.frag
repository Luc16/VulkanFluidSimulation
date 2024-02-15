#version 450

layout(location = 0) out vec4 outColor;

void main() {

    vec2 coord = gl_PointCoord - vec2(0.5);
    const float radius = 0.25;
    if (length(gl_PointCoord - vec2(0.5)) > radius) {
        discard;
    }
    outColor = vec4(0.2f, 0.6f, 1.0f, 1.0f);
}