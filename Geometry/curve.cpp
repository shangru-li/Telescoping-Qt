#include "curve.h"

Curve::Curve(GLContext *context): Drawable(context), points(nullptr)
{

}

void Curve::createGeometry()
{
    indexBuffer.clear();
    vertexBuffer.clear();
    if (points && points->size() > 1)
    {
        for(int i = 0; i < points->size() - 1; ++i)
        {
            indexBuffer.push_back(i);
            indexBuffer.push_back(i + 1);
            vertexBuffer.push_back((*points)[i]);
            vertexBuffer.push_back(_white);
            vertexBuffer.push_back(_normal);
        }
        vertexBuffer.push_back((*points)[points->size() - 1]);
        vertexBuffer.push_back(_white);
        vertexBuffer.push_back(_normal);
    }
}

int Curve::drawMode()
{
    return GL_LINES;
}
