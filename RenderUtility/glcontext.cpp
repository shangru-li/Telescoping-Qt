#include "glcontext.h"

GLContext::GLContext(QWidget *parent)
    : QOpenGLWidget(parent),
      shaderProgram(this), shaderProgramFlat(this),
      fps(60.f),
      squarePlane(this), cubeArray(this),
      selectedCube(nullptr), currentJuncture(nullptr), movingCube(false), canGenerate(true), canGenerateJuncture(true),
      torsionStage(false), discreteStage(false)
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

    glClearColor(1, 1, 1, 1);

    glGenVertexArrays(1, &vao);

    shaderProgram.create(":/resources/basic.vert.glsl", ":/resources/basic.frag.glsl");
    shaderProgramFlat.create(":/resources/flat.vert.glsl", ":/resources/flat.frag.glsl");
    camera = std::make_unique<Camera>((float)width() / height(), 90.f, glm::vec3(0, 0, -5), glm::vec3(0, 0, 0));
    setFocus();

    currentJuncture = &cubeArray.addCube(glm::scale(glm::mat4(1.f), glm::vec3(0.5,0.5,0.5)), nullptr, Cube::JUNCTURE);
    currentJuncture->rootCube = currentJuncture;
}

void updateChildrenTransforms(Curve &curve)
{
    for (Curve *child: curve.childrenCurves)
    {
        child->updateSegmentTransforms();
        updateChildrenTransforms(*child);
    }
}

void GLContext::paintGL()
{
    cubeArray.update();
    if(!torsionStage && !discreteStage)
    {
        for (unique_ptr<Curve> &curve: curves)
        {
            curve->updateCurve();
            curve->points = &curve->curve;
        }
    }

    for (unique_ptr<Curve> &curve: curves)
    {
        if (!curve->parentCube->parentCurve)
        {
            curve->updateSegmentTransforms();
            updateChildrenTransforms(*curve);
        }
    }

    for (unique_ptr<Curve> &curve: curves)
    {
        curve->update();
        for (unique_ptr<Shell> &pShell : curve->shells) pShell->update();
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    printGLErrorLog();

    shaderProgram.setCamPos(glm::vec4(camera->eye, 1));
    shaderProgramFlat.setModelViewProj(camera->getViewProj());
    shaderProgramFlat.draw(cubeArray);
    for (unique_ptr<Curve> &curve: curves)
    {
        shaderProgramFlat.draw(*curve);
        for (unique_ptr<Shell> &pShell : curve->shells)
        {
            shaderProgram.setModelViewProj(camera->getViewProj(), pShell->junctureAnimatedTransform);
            shaderProgram.draw(*pShell);
        }
    }
}

void makeChildrenTelescopes(Curve &curve)
{
    for (Curve *child: curve.childrenCurves)
    {
        child->makeTelescope(curve.endRadius);
        makeChildrenTelescopes(*child);
    }
}

void GLContext::keyPressEvent(QKeyEvent *e)
{
    // from a to z
    if (e->key() >= 0x30 && e->key() <= 0x5a) keyboardStates[e->key()] = true;
    if (e->key() == 'F' && canGenerate)
    {
        canGenerate = false;
        if (selectedCube->type == Cube::JUNCTURE)
        {
            Cube *newCube = &cubeArray.addCube(selectedCube->transform, currentJuncture, Cube::GENERATOR);
            Curve *curve = &addCurve(selectedCube, newCube);
            newCube->parentCurve = curve;
            updateSelectedCube(newCube);
        }
        else if (selectedCube->type == Cube::GENERATOR)
        {
            Cube *newCube = &cubeArray.addCube(selectedCube->transform, currentJuncture, Cube::GENERATOR);
            selectedCube->parentCurve->curveCubes.push_back(newCube);
            selectedCube->parentCurve->childCube = newCube;
            newCube->parentCurve = selectedCube->parentCurve;
            selectedCube->parentCurve = nullptr;
            selectedCube->type = Cube::NORMAL;
            updateSelectedCube(newCube);
        }
    }
    else if (e->key() == 'G' && canGenerateJuncture)
    {
        canGenerateJuncture = false;
        if (selectedCube->type == Cube::GENERATOR)
        {
            selectedCube->type = Cube::JUNCTURE;
            selectedCube->parentJuncture = selectedCube->rootCube;
            selectedCube->rootCube = selectedCube;
        }
    }
    if (e->key() == 'P')
    {
        for (unique_ptr<Curve> &curve: curves)
        {
            curve->reAssignPoints();
            curve->discretilize();
        }
        discreteStage = true;
    }
    if (e->key() == 'O')
    {
        for (unique_ptr<Curve> &curve: curves) curve->makeImpulseCurve();
        torsionStage = true;
    }
    if (e->key() == 'I')
    {
        for (unique_ptr<Curve> &curve: curves)
        {
            if (!curve->parentCube->parentCurve)
            {
                curve->makeTelescope();
                makeChildrenTelescopes(*curve);
            }
        }
    }
    if (e->key() == 'Z')
    {
        for (unique_ptr<Curve> &curve: curves)
            if (curve->extensionState == Curve::EXTENDED) curve->extensionState = Curve::RETRACTING;
            else if (curve->extensionState == Curve::RETRACTED) curve->extensionState = Curve::EXTENDING;
    }
}

void GLContext::keyReleaseEvent(QKeyEvent *e)
{
    // from a to z
    if (e->key() >= 0x30 && e->key() <= 0x5a) keyboardStates[e->key()] = false;
    if (e->key() == 'F' && !canGenerate) canGenerate = true;
    if (e->key() == 'G' && !canGenerateJuncture) canGenerateJuncture = true;
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
        if (!isOperatingCube(*hit))
        {
            updateSelectedCube(hit);
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

void GLContext::updateSelectedCube(Cube *newSelected)
{
    if (selectedCube) selectedCube->selected = false;
    newSelected->selected = true;
    selectedCube = newSelected;
    currentJuncture = selectedCube->rootCube;
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

Curve &GLContext::addCurve(Cube *parentCube, Cube *childCube)
{
    unique_ptr<Curve> pCurve = make_unique<Curve>(this, parentCube, childCube);
    Curve &curve = *pCurve;
    curve.curveCubes.push_back(parentCube);
    curve.curveCubes.push_back(childCube);
    curves.push_back(std::move(pCurve));
    if (parentCube->parentCurve) parentCube->parentCurve->childrenCurves.push_back(&curve);
    return curve;
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

    for (unique_ptr<Curve> &curve: curves)
        if (curve->extensionState == Curve::EXTENDING)
        {
            curve->extensionExtent += 6.f / Curve::numImpulses / fps;
            if (curve->extensionExtent > 1.f)
            {
                curve->extensionState = Curve::EXTENDED;
                curve->extensionExtent = 1.f;
            }
        }
        else if (curve->extensionState == Curve::RETRACTING)
        {
            curve->extensionExtent -= 6.f / Curve::numImpulses / fps;
            if (curve->extensionExtent < 0.f)
            {
                curve->extensionState = Curve::RETRACTED;
                curve->extensionExtent = 0.f;
            }
        }

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
