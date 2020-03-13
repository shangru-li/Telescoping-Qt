#include "glcontext.h"

GLContext::GLContext(QWidget *parent)
    : QOpenGLWidget(parent), shaderProgram(this), fps(60.f),
      squarePlane(this), cubeArray(this), selectedCube(false), movingCube(false), canGenerate(false)
{
    connect(&timer, SIGNAL(timeout()), this, SLOT(timerUpdate())); // when it's time to update a frame
    timer.start(glm::round(1000 / fps)); // update every 16 ms
    for (bool &state : keyboardStates) state = false;
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
    camera = std::make_unique<Camera>((float)width() / height(), 90.f, glm::vec3(0, 0, -5), glm::vec3(0, 0, 0));
    setFocus();

    cubeArray.addCube(glm::scale(glm::mat4(1.f), glm::vec3(0.5,0.5,0.5)), Cube::GENERATOR);
}

void GLContext::paintGL()
{
    cubeArray.update();

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    shaderProgram.setModelViewProj(camera->getViewProj());
    shaderProgram.draw(cubeArray);
}

void GLContext::keyPressEvent(QKeyEvent *e)
{
    // from a to z
    if (e->key() >= 0x30 && e->key() <= 0x5a) keyboardStates[e->key()] = true;
    if (e->key() == 'F' && canGenerate)
    {
        canGenerate = false;
        cubeArray.addCube(selectedCube->transform);
    }
}

void GLContext::keyReleaseEvent(QKeyEvent *e)
{
    // from a to z
    if (e->key() >= 0x30 && e->key() <= 0x5a) keyboardStates[e->key()] = false;
    if (e->key() == 'F' && !canGenerate) canGenerate = true;
}

void GLContext::mousePressEvent(QMouseEvent *e)
{
    QPoint mousePos = mapFromGlobal(QCursor::pos());
    float ndcX = 2 * mousePos.x() / (float)width() - 1,
            ndcY = 1 - 2 * mousePos.y() / (float)height();
    Intersection intersection;
    Camera *pCamera = camera.get();
    Ray r = pCamera->castRay(ndcX, ndcY);
    if (cubeArray.intersect(r, intersection))
    {
        Cube *hit = intersection.cubeHit;
        if (hit->type == Cube::NORMAL || hit->type == Cube::GENERATOR)
        {
            if (selectedCube) selectedCube->selected = false;
            hit->selected = true;
            selectedCube = intersection.cubeHit;
        }
        else
        {
            movingCube = true;
            if (hit->type == Cube::RED) offsetVector = _red;
            else if (hit->type == Cube::GREEN) offsetVector = _green;
            else if (hit->type == Cube::BLUE) offsetVector = _blue;
            glm::vec4 screenCoordsOrigin = getScreenCoords(selectedCube->transform * _black);
            glm::vec4 screenCoordsOffset = getScreenCoords(selectedCube->transform * offsetVector);

            moveControlVector = glm::vec2(screenCoordsOffset) - glm::vec2(screenCoordsOrigin);
            pressPoint = glm::vec2(mousePos.x(), mousePos.y());
            pressTransform = selectedCube->transform;
        }
    }
}

void GLContext::mouseMoveEvent(QMouseEvent *e)
{
    if (movingCube)
    {
        QPoint mousePos = mapFromGlobal(QCursor::pos());

        glm::vec2 moveVector = glm::vec2(mousePos.x(), mousePos.y()) - pressPoint;
        float translateAmount = glm::dot(moveVector, moveControlVector) / glm::length2(moveControlVector);
        selectedCube->transform = glm::translate(pressTransform, glm::vec3(offsetVector) * translateAmount);
    }
}

void GLContext::mouseReleaseEvent(QMouseEvent *e)
{
    if (movingCube) movingCube = false;
}

glm::vec4 GLContext::getScreenCoords(glm::vec4 pointWorld)
{
    pointWorld = camera->getViewProj() * pointWorld;
    float z = pointWorld.z;
    pointWorld /= z;
    // w stores z for perspective divide
    return glm::vec4((pointWorld.x + 1) * (float)width() / 2, (1 - pointWorld.y) * (float)height() / 2, pointWorld.z, z);
}

glm::vec4 GLContext::getWorldCoords(glm::vec4 pointScreen)
{
    // w stores z for perspective divide
    float z = pointScreen.w;
    glm::vec4 pointWorld(pointScreen.x * 2 / (float)width() - 1, 1 - pointScreen.y * 2 / (float)height(), pointScreen.z, 1.f);
    pointWorld *= z;
    return glm::inverse(camera->getViewProj()) * pointWorld;
}


void GLContext::timerUpdate()
{
    float roundsPerSecond = 0.25, rotateDegree = 360 * roundsPerSecond / fps, zoomLength = 0.1;
    if (keyboardStates['A']) camera->rotateSpherical(rotateDegree, camera->up);
    if (keyboardStates['D']) camera->rotateSpherical(-rotateDegree, camera->up);
    if (keyboardStates['W']) camera->rotateSpherical(rotateDegree, camera->right);
    if (keyboardStates['S']) camera->rotateSpherical(-rotateDegree, camera->right);
    if (keyboardStates['Q']) camera->zoom(-zoomLength);
    if (keyboardStates['E']) camera->zoom(zoomLength);

    update(); // update the widget
}
