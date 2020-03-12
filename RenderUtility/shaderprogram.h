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

    GLuint hVertexShader, hFragmentShader, hProgram;

    int attrPosition, attrColor, attrNormal;

    inline void useMe();

protected:
    GLContext *context;
};

#endif // SHADERPROGRAM_H
