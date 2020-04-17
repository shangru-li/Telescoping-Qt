#version 150

in vec3 fs_Color;
in vec4 fs_LightVector;
in vec4 fs_Normal;

out vec3 out_Color;

void main()
{
    out_Color = fs_Color;
}
