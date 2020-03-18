#include "cubearray.h"

glm::vec4 CubeArray::p0{-SIZE, -SIZE, -SIZE, 1};
glm::vec4 CubeArray::p1{-SIZE, -SIZE, SIZE, 1};
glm::vec4 CubeArray::p2{-SIZE, SIZE, -SIZE, 1};
glm::vec4 CubeArray::p3{-SIZE, SIZE, SIZE, 1};
glm::vec4 CubeArray::p4{SIZE, -SIZE, -SIZE, 1};
glm::vec4 CubeArray::p5{SIZE, -SIZE, SIZE, 1};
glm::vec4 CubeArray::p6{SIZE, SIZE, -SIZE, 1};
glm::vec4 CubeArray::p7{SIZE, SIZE, SIZE, 1};

CubeArray::CubeArray(GLContext *context): Drawable(context)
{

}

void CubeArray::createGeometry()
{
    vertexBuffer.clear();
    indexBuffer.clear();

    for (int i = 0; i < cubes.size(); )
    {
        Cube *c = cubes[i].get();
        if (c->type != Cube::NORMAL && c->type != Cube::GENERATOR) cubes.erase(cubes.begin() + i);
        else ++i;
    }

    for (int i = 0; i < cubes.size(); ++i)
    {
        Cube *c = cubes[i].get();

        if (c->selected)
        {
            addColoredCube(c);
            redArrow = std::make_unique<Cube>(c->transform * glm::translate(glm::scale(glm::mat4(1.f), glm::vec3(0.3f, 0.3f, 0.3f)), glm::vec3(5, 0, 0)), Cube::RED);
            addColoredCube(redArrow.get(), _red, false);
            greenArrow = std::make_unique<Cube>(c->transform * glm::translate(glm::scale(glm::mat4(1.f), glm::vec3(0.3f, 0.3f, 0.3f)), glm::vec3(0, 5, 0)), Cube::GREEN);
            addColoredCube(greenArrow.get(), _green, false);
            blueArrow = std::make_unique<Cube>(c->transform * glm::translate(glm::scale(glm::mat4(1.f), glm::vec3(0.3f, 0.3f, 0.3f)), glm::vec3(0, 0, 5)), Cube::BLUE);
            addColoredCube(blueArrow.get(), _blue, false);
        }
        else addColoredCube(c, _grey, false);
    }

    if (redArrow && greenArrow && blueArrow)
    {
        cubes.push_back(std::move(redArrow));
        cubes.push_back(std::move(greenArrow));
        cubes.push_back(std::move(blueArrow));
    }
}

void CubeArray::addCube(glm::mat4 transform, int type)
{
    cubes.push_back(std::make_unique<Cube>(transform, type));
}

bool CubeArray::intersect(const Ray &r, Intersection &intersection) const
{
    intersection.t = std::numeric_limits<float>::max();
    for (int i = 0; i < cubes.size(); ++i)
    {
        Cube *c = cubes[i].get();
        Intersection intersectionCube;
        if (c->intersect(r, intersectionCube) && intersectionCube.t < intersection.t) intersection = intersectionCube;
    }
    if (intersection.cubeHit) return true;
    intersection.t = 0.f;
    return false;
}

void CubeArray::generateKeys()
{
    keys.clear();
    for (int i = 0; i < cubes.size(); ++i)
    {
        Cube *c = cubes[i].get();
        if (c->type == Cube::NORMAL || c->type == Cube::GENERATOR)
        {
            keys.push_back(c->transform * _black);
        }
    }
}

void CubeArray::computeCtrlPoints()
{
    ctrlPoints.clear();
    if (keys.size() <= 1) return;
    glm::vec4 startPoint = keys[0] + 0.25f * (keys[0] - keys[1]);
    glm::vec4 endPoint = keys[keys.size() - 1] + 0.25f * (keys[keys.size() - 1] - keys[keys.size() - 2]);
    for (int i = 1; i < keys.size(); ++i)
    {
        glm::vec4 b0, b1, b2, b3, t0, t3;
        b0 = keys[i - 1], b3 = keys[i];
        if (i == 1) t0 = (keys[i] - startPoint) / 2.f;
        else t0 = (keys[i] - keys[i - 2]) / 2.f;
        b1 = b0 + t0 / 3.f;
        if (i == keys.size() - 1) t3 = (endPoint - keys[i - 1]) / 2.f;
        else t3 = (keys[i + 1] - keys[i - 1]) / 2.f;
        b2 = b3 - t3 / 3.f;
        std::vector<glm::vec4> v{b0, b1, b2, b3};
        ctrlPoints.insert(ctrlPoints.end(), v.begin(), v.end());
    }
}

void CubeArray::interpolate()
{
    curve.clear();
    if (keys.size() <= 1) return;
    for (int segment = 0; segment < keys.size() - 1; ++segment)
    {
        for (float t = 0.f; t < 1.f - std::numeric_limits<float>::min(); t += 0.1f)
        {
            curve.push_back(interpolateSegment(segment, t));
        }
    }
    curve.push_back(interpolateSegment(keys.size() - 2, 1.f));
}

glm::vec4 CubeArray::interpolateSegment(int segment, float t)
{
    glm::vec4 b0 = ctrlPoints[segment * 4 + 0],
            b1 = ctrlPoints[segment * 4 + 1],
            b2 = ctrlPoints[segment * 4 + 2],
            b3 = ctrlPoints[segment * 4 + 3],
            p00 = b0 + (b1 - b0) * t,
            p10 = b1 + (b2 - b1) * t,
            p20 = b2 + (b3 - b2) * t,
            p01 = p00 + (p10 - p00) * t,
            p11 = p10 + (p20 - p10) * t,
            p02 = p01 + (p11 - p01) * t;
    return p02;
}

void CubeArray::updateCurve()
{
    generateKeys();
    computeCtrlPoints();
    interpolate();
}

void CubeArray::addColoredCube(Cube *c, glm::vec4 color, bool isColorful)
{
    glm::vec4 n(0, 0, 1, 0);
    std::vector<glm::vec4> v{c->transform * p0, (isColorful? _red: color), n,
                c->transform * p1, (isColorful? _green: color), n,
                c->transform * p2, (isColorful? _blue: color), n,
                c->transform * p3, (isColorful? _black: color), n,
                c->transform * p4, (isColorful? _white: color), n,
                c->transform * p5, (isColorful? _grey: color), n,
                c->transform * p6, (isColorful? _purple: color), n,
                c->transform * p7, (isColorful? _yellow: color), n};
    std::vector<int> ib{2, 3, 6, 3, 6, 7,
                        1, 5, 3, 5, 7, 3,
                        0, 2, 6, 0, 6, 4,
                        0, 3, 2, 0, 1, 3,
                        4, 6, 7, 4, 7, 5,
                        1, 0, 4, 1, 4, 5};
    for (int &j : ib) j += vertexBuffer.size() / 3;
    vertexBuffer.insert(vertexBuffer.end(), v.begin(), v.end());
    indexBuffer.insert(indexBuffer.end(), ib.begin(), ib.end());
}
