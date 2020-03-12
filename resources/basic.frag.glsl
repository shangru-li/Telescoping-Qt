#version 150

in vec3 fs_Position;
in vec3 fs_Color;

out vec3 out_Col;

void main()
{
    out_Col = fs_Color;
}
