#ifndef GLCONTEXT_H
#define GLCONTEXT_H

#include <QTimer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLShaderProgram>

#include <global.h>

#include "Geometry/squareplane.h"
#include "Geometry/cubearray.h"
#include "Geometry/curve.h"
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

    void keyPressEvent(QKeyEvent *e);
    void keyReleaseEvent(QKeyEvent *e);
    void mousePressEvent(QMouseEvent *e);
    void mouseMoveEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);

    // move control
    glm::vec2 moveControlVector, pressPoint;
    glm::vec4 offsetVector;
    glm::mat4 pressTransform;
    bool movingCube;

    // cube generation
    bool canGenerate;

    glm::vec4 getScreenCoords(glm::vec4 pointWorld);
    glm::vec4 getWorldCoords(glm::vec4 pointScreen);

    // geometries
    SquarePlane squarePlane;

    CubeArray cubeArray;
    Cube *selectedCube;
    Curve curve;

    ShaderProgram shaderProgram;

    std::unique_ptr<Camera> camera;

    float fps;

    std::array<bool, 256> keyboardStates;

private:
    QTimer timer; // controls frame rate
    GLuint vao; // handle for vao

private slots:
    void timerUpdate(); // called every frame
};

#endif // GLCONTEXT_H
