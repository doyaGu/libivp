#version 450
layout(location=0) in vec3 vCol;
layout(location=0) out vec4 FragColor;
void main() {
    FragColor = vec4(vCol, 1.0);
}
