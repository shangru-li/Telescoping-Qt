#include "intersection.h"

Intersection::Intersection()
    : cubeHit(nullptr), point(glm::vec4(0, 0, 0, 1)), t(0.f)
{

}

Intersection::Intersection(const Intersection &i)
    : point(i.point), cubeHit(i.cubeHit), t(i.t)
{

}
