#include "RenderUtility/glcontext.h"

Drawable::Drawable(GLContext *context)
    : context(context), hIndexBuffer(0), hVertexBuffer(0), hasGeneratedBuffer(false),
      indexCount(0)
{

}

void Drawable::create()
{
    if (indexBuffer.size() == 0) return;

    indexCount = indexBuffer.size();

    context->glGenBuffers(1, &hIndexBuffer);
    context->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, hIndexBuffer);
    context->glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexBuffer.size() * sizeof(GLuint), indexBuffer.data(), GL_STATIC_DRAW);

    context->glGenBuffers(1, &hVertexBuffer);
    context->glBindBuffer(GL_ARRAY_BUFFER, hVertexBuffer);
    context->glBufferData(GL_ARRAY_BUFFER, vertexBuffer.size() * sizeof(glm::vec4), vertexBuffer.data(), GL_STATIC_DRAW);

    hasGeneratedBuffer = true;
}

void Drawable::destroy()
{
    if (hasGeneratedBuffer)
    {
        context->glDeleteBuffers(1, &hIndexBuffer);
        context->glDeleteBuffers(1, &hVertexBuffer);
        hasGeneratedBuffer = false;
        indexCount = 0;
    }
}
