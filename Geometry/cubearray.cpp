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
            redArrow = std::make_unique<Cube>(c->transform * glm::translate(glm::scale(glm::mat4(1.f), glm::vec3(0.3f, 0.3f, 0.3f)), glm::vec3(5, 0, 0)), 1);
            addColoredCube(redArrow.get(), _red, false);
            greenArrow = std::make_unique<Cube>(c->transform * glm::translate(glm::scale(glm::mat4(1.f), glm::vec3(0.3f, 0.3f, 0.3f)), glm::vec3(0, 5, 0)), 2);
            addColoredCube(greenArrow.get(), _green, false);
            blueArrow = std::make_unique<Cube>(c->transform * glm::translate(glm::scale(glm::mat4(1.f), glm::vec3(0.3f, 0.3f, 0.3f)), glm::vec3(0, 0, 5)), 3);
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
