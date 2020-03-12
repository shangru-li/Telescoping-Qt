#ifndef GLCONTEXT_H
#define GLCONTEXT_H

#include <QTimer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLShaderProgram>

#include <global.h>

#include "Geometry/squareplane.h"
#include "shaderprogram.h"
#include "camera.h"

class GLContext: public QOpenGLWidget, public QOpenGLFunctions_3_2_Core
{
    Q_OBJECT

public:
    GLContext(QWidget *parent);
    ~GLContext();

    void initializeGL();
    void paintGL();

    // geometries
    SquarePlane squarePlane;

    ShaderProgram shaderProgram;

    Camera camera;

private:
    QTimer timer; // controls frame rate
    GLuint vao; // handle for vao

private slots:
    void timerUpdate(); // called every frame
};

#endif // GLCONTEXT_H
