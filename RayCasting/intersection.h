#pragma once

#include "global.h"
class Cube;
class Intersection
{
public:
    Intersection();
    Intersection(const Intersection &i);

    glm::vec4 point;
    Cube *cubeHit;
    float t;
};
