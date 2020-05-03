#include "glcontext.h"

GLContext::GLContext(QWidget *parent)
    : QOpenGLWidget(parent),
      shaderProgram(this), shaderProgramFlat(this),
      fps(60.f),
      squarePlane(this), cubeArray(this), curve(this),
      selectedCube(nullptr), movingCube(false), canGenerate(true),
      torsionStage(false), discreteStage(false)
{
    connect(&timer, SIGNAL(timeout()), this, SLOT(timerUpdate())); // when it's time to update a frame
    timer.start(glm::round(1000 / fps)); // update every 16 ms
    for (bool &state : keyboardStates) state = false;
    for (int i = 0; i < Curve::numImpulses; ++i) curve.shells.push_back(make_unique<Shell>(this));
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

    glClearColor(1, 1, 1, 1);

    glGenVertexArrays(1, &vao);

    shaderProgram.create(":/resources/basic.vert.glsl", ":/resources/basic.frag.glsl");
    shaderProgramFlat.create(":/resources/flat.vert.glsl", ":/resources/flat.frag.glsl");
    camera = std::make_unique<Camera>((float)width() / height(), 90.f, glm::vec3(0, 0, -5), glm::vec3(0, 0, 0));
    setFocus();

    cubeArray.addCube(glm::scale(glm::mat4(1.f), glm::vec3(0.5,0.5,0.5)), Cube::GENERATOR);
}

float getCurrentState(float t, int num, int grid)
{
    float step = 1.f / num;
    int currNum = t / step;
    if (grid < currNum) return 1;
    if (grid > currNum) return 0;
    return (t - currNum * step) / step;
}

void GLContext::paintGL()
{
    cubeArray.update();
    if(!torsionStage && !discreteStage)
    {
        cubeArray.updateCurve();
        curve.points = &cubeArray.curve;
    }

    curve.update();
    for (unique_ptr<Shell> &pShell : curve.shells) pShell->update();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    printGLErrorLog();

    shaderProgram.setCamPos(glm::vec4(camera->eye, 1));
    shaderProgramFlat.setModelViewProj(camera->getViewProj());
    shaderProgramFlat.draw(cubeArray);
    shaderProgramFlat.draw(curve);


    if (!curve.tParams.empty())for (int i = 0; i < curve.tParams.size() - 1; ++i)
    {
        //float currentState = curve.extensionExtent;
        float currentState = getCurrentState(curve.extensionExtent, curve.numImpulses - 1, i);
        pl(currentState);
        //glm::vec3 position = curve.transformedHelixPoint(curve.pSegments->at(i+1), currentState * curve.pSegments->at(i+1).arcLength);
        //OrthonormalFrame f = transformedHelixFrame(curve.pSegments->at(i+1), currentState * curve.pSegments->at(i+1).arcLength);
        glm::vec3 position = curve.transformedHelixPoint(curve.pSegments->at(i), currentState * curve.pSegments->at(i+1).arcLength);
        OrthonormalFrame f = transformedHelixFrame(curve.pSegments->at(i), currentState * curve.pSegments->at(i+1).arcLength);
        glm::mat4 relative = glm::mat4(glm::vec4(f.B, 0), glm::vec4(f.N, 0), glm::vec4(f.T, 0), glm::vec4(position, 1));
        curve.shells[i+1]->animatedTransform = curve.shells[i]->animatedTransform * glm::inverse(curve.shells[i]->transform) * relative;

    }
    for (unique_ptr<Shell> &pShell : curve.shells)
    {
        shaderProgram.setModelViewProj(camera->getViewProj(), pShell->animatedTransform);
        shaderProgram.draw(*pShell);
    }
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
    if (e->key() == 'P')
    {
        curve.reAssignPoints();
        curve.discretilize();
        discreteStage = true;
    }
    if (e->key() == 'O')
    {
        curve.makeImpulseCurve();
        torsionStage = true;
    }
    if (e->key() == 'I')
    {
        curve.makeTelescope();
    }
    if (e->key() == 'Z')
    {
        if (curve.extensionState == Curve::EXTENDED) curve.extensionState = Curve::RETRACTING;
        else if (curve.extensionState == Curve::RETRACTED) curve.extensionState = Curve::EXTENDING;
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

    if (curve.extensionState == Curve::EXTENDING)
    {
        curve.extensionExtent += 6.f / Curve::numImpulses / fps;
        if (curve.extensionExtent > 1.f)
        {
            curve.extensionState = Curve::EXTENDED;
            curve.extensionExtent = 1.f;
        }
    }
    else if (curve.extensionState == Curve::RETRACTING)
    {
        curve.extensionExtent -= 6.f / Curve::numImpulses / fps;
        if (curve.extensionExtent < 0.f)
        {
            curve.extensionState = Curve::RETRACTED;
            curve.extensionExtent = 0.f;
        }
    }
    //pl(curve.extensionExtent);
    //pl(curve.extensionState, "state");
    update(); // update the widget
}

void GLContext::printGLErrorLog()
{
    GLenum error = glGetError();
    if (error != GL_NO_ERROR) {
        std::cerr << "OpenGL error " << error << ": ";
        const char *e =
            error == GL_INVALID_OPERATION             ? "GL_INVALID_OPERATION" :
            error == GL_INVALID_ENUM                  ? "GL_INVALID_ENUM" :
            error == GL_INVALID_VALUE                 ? "GL_INVALID_VALUE" :
            error == GL_INVALID_INDEX                 ? "GL_INVALID_INDEX" :
            error == GL_INVALID_OPERATION             ? "GL_INVALID_OPERATION" :
            QString::number(error).toUtf8().constData();
        std::cerr << e << std::endl;
    }
}
