#ifndef SHADERPROGRAM_H
#define SHADERPROGRAM_H

#include "global.h"
#include "Geometry/drawable.h"
class GLContext;
class ShaderProgram
{
public:
    ShaderProgram(GLContext *context);

    void create(const char *vertexFile, const char *fragmentFile);
    void draw(Drawable &drawable);

    void setModelViewProj(glm::mat4 viewProj, glm::mat4 model = glm::mat4(1.f));

    inline void useMe();

    GLuint hVertexShader, hFragmentShader, hProgram;

    // handles to the shader variables
    int attrPosition, attrColor, attrNormal;
    int unifViewProj, unifModel;

protected:
    GLContext *context;
};

#endif // SHADERPROGRAM_H
