#version 150

in vec3 fs_Color;
in vec4 fs_LightVector;
in vec4 fs_Normal;

out vec3 out_Color;

void main()
{
    float diffuseTerm = dot(fs_Normal, fs_LightVector);
    diffuseTerm = clamp(diffuseTerm, 0, 1);

    int blinnExp = 20;
    float specularIntensity = max(pow(dot(fs_LightVector, fs_Normal), blinnExp), 0);
    float specularTerm = specularIntensity * 20;
    specularTerm = clamp(specularTerm, 0, 1);

    float ambientTerm = 0.2;

    out_Color = vec3(fs_Color * (diffuseTerm + specularTerm + ambientTerm));
    //out_Color = fs_Color;
}
