#include "RenderUtility/glcontext.h"

ShaderProgram::ShaderProgram(GLContext *context)
    : context(context), hVertexShader(0), hFragmentShader(0), hProgram(0),
      attrPosition(-1), attrColor(-1), attrNormal(-1)
{

}

void ShaderProgram::useMe()
{
    context->glUseProgram(hProgram);
}

void ShaderProgram::draw(Drawable &drawable)
{
    useMe();

    context->glBindBuffer(GL_ARRAY_BUFFER, drawable.hVertexBuffer);
    int sizeOfVertexBufferItem = 12;
    if (attrPosition != -1)
    {
        context->glEnableVertexAttribArray(attrPosition);
        context->glVertexAttribPointer(attrPosition, 4, GL_FLOAT, false,
                                       sizeOfVertexBufferItem * sizeof(float), NULL);
    }
    if (attrColor != -1)
    {
        context->glEnableVertexAttribArray(attrColor);
        context->glVertexAttribPointer(attrColor, 4, GL_FLOAT, false,
                                       sizeOfVertexBufferItem * sizeof(float), (void*)(4 * sizeof(float)));
    }
    if (attrNormal != -1)
    {
        context->glEnableVertexAttribArray(attrNormal);
        context->glVertexAttribPointer(attrNormal, 4, GL_FLOAT, false,
                                       sizeOfVertexBufferItem * sizeof(float), (void*)(8 * sizeof(float)));
    }

    context->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, drawable.hIndexBuffer);
    context->glDrawElements(GL_TRIANGLES, drawable.indexCount, GL_UNSIGNED_INT, 0);

    if (attrPosition != -1) context->glDisableVertexAttribArray(attrPosition);
    if (attrColor != -1) context->glDisableVertexAttribArray(attrColor);
    if (attrNormal != -1) context->glDisableVertexAttribArray(attrNormal);
}

void ShaderProgram::setModelViewProj(glm::mat4 viewProj, glm::mat4 model)
{
    useMe();
    if (unifViewProj != -1) context->glUniformMatrix4fv(unifViewProj, 1, GL_FALSE, &viewProj[0][0]);
    useMe();
    if (unifModel != -1) context->glUniformMatrix4fv(unifModel, 1, GL_FALSE, &model[0][0]);
}

void ShaderProgram::create(const char *vertexFile, const char *fragmentFile)
{
    hVertexShader = context->glCreateShader(GL_VERTEX_SHADER);
    hFragmentShader = context->glCreateShader(GL_FRAGMENT_SHADER);
    hProgram = context->glCreateProgram();

    std::string vertexSource = fromFile(vertexFile);
    std::string fragmentSource = fromFile(fragmentFile);

    const char *vertexSourceCString = vertexSource.c_str();
    const char *fragmentSourceCString = fragmentSource.c_str();

    context->glShaderSource(hVertexShader, 1, &vertexSourceCString, 0);
    context->glShaderSource(hFragmentShader, 1, &fragmentSourceCString, 0);

    context->glCompileShader(hVertexShader);
    context->glCompileShader(hFragmentShader);

    context->glAttachShader(hProgram, hVertexShader);
    context->glAttachShader(hProgram, hFragmentShader);
    context->glLinkProgram(hProgram);

    // handles to shader variables
    attrPosition = context->glGetAttribLocation(hProgram, "vs_Position");
    attrColor = context->glGetAttribLocation(hProgram, "vs_Color");
    attrNormal = context->glGetAttribLocation(hProgram, "vs_Normal");

    unifModel = context->glGetUniformLocation(hProgram, "u_Model");
    unifViewProj = context->glGetUniformLocation(hProgram, "u_ViewProj");
}
