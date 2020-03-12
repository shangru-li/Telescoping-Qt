#include "glcontext.h"

GLContext::GLContext(QWidget *parent)
    : QOpenGLWidget(parent), shaderProgram(this),
      squarePlane(this)
{
    connect(&timer, SIGNAL(timeout()), this, SLOT(timerUpdate())); // when it's time to update a frame
    timer.start(16); // update every 16 ms
}

GLContext::~GLContext() {
    glDeleteVertexArrays(1, &vao);
}

void GLContext::initializeGL()
{
    // initialize
    initializeOpenGLFunctions();

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LINE_SMOOTH);
    glEnable(GL_POLYGON_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

    glPointSize(5);

    glClearColor(0, 0, 0, 1);

    glGenVertexArrays(1, &vao);

    shaderProgram.create(":/resources/basic.vert.glsl", ":/resources/basic.frag.glsl");
}

void GLContext::paintGL()
{
    squarePlane.createGeometry();
    squarePlane.destroy();
    squarePlane.create();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    shaderProgram.draw(squarePlane);
}


void GLContext::timerUpdate()
{
    update(); // update the widget
}
