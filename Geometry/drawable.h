#ifndef DRAWABLE_H
#define DRAWABLE_H

#include <global.h>

class GLContext;

class Drawable
{
public:
    Drawable(GLContext *context);

    std::vector<int> indexBuffer;
    std::vector<glm::vec4> vertexBuffer;

    virtual void createGeometry() = 0;
    virtual glm::mat4 getModel();
    void create();
    void destroy();
    void update();

    GLuint hIndexBuffer, hVertexBuffer;
    int indexCount; // for drawing in shader program

protected:
    GLContext *context;

private:
    bool hasGeneratedBuffer;
};

#endif // DRAWABLE_H
