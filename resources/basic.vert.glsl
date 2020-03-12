#version 150

in vec4 vs_Position;
in vec3 vs_Color;

out vec3 fs_Color;

uniform mat4 u_ViewProj;
uniform mat4 u_Model;

void main()
{
    fs_Color = vs_Color;

    gl_Position = u_ViewProj * u_Model * vs_Position;
}
