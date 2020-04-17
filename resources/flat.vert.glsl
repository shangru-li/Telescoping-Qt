#version 150

in vec4 vs_Position;
in vec3 vs_Color;
in vec4 vs_Normal;

out vec3 fs_Color;
out vec4 fs_Normal;
out vec4 fs_LightVector;

uniform mat4 u_ViewProj;
uniform mat4 u_Model;
uniform mat4 u_ModelInvTr;
uniform vec4 u_CamPos;

void main()
{
    fs_Color = vs_Color;
    gl_Position = u_ViewProj * u_Model * vs_Position;
}
