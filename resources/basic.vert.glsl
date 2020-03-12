#version 150

in vec4 vs_Position;
in vec3 vs_Color;

out vec3 fs_Position;
out vec3 fs_Color;

void main()
{
    fs_Color = vs_Color;
    fs_Position = vs_Position.xyz;

    gl_Position = vs_Position;
}
