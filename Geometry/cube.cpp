#include "cube.h"

Cube::Cube(glm::mat4 transform, int type)
    : transform(transform), selected(false), type(type), rootCube(nullptr), parentJuncture(nullptr), parentCurve(nullptr)
{}

bool Cube::intersect(const Ray &r, Intersection &i)
{
    Ray localRay = r.transformRay(glm::inverse(transform));
    float t_n = std::numeric_limits<float>::lowest(),
            t_f = std::numeric_limits<float>::max();

    for (int i = 0; i < 3; ++i)
    {
        if (localRay.direction[i] == 0)
        {
            if (localRay.origin[i] < -0.5f || localRay.origin[i] > 0.5f) return false;
        }

        float t0 = (-0.5f - localRay.origin[i]) / localRay.direction[i],
                t1 = (0.5f - localRay.origin[i]) / localRay.direction[i];
        if (t0 > t1) std::swap(t0, t1);
        if (t0 > t_n) t_n = t0;
        if (t1 < t_f) t_f = t1;
    }
    if (t_n > t_f) return false;

    float t = (t_n > 0) ? t_n: t_f;
    if (t < 0) return false;

    glm::vec4 localPoint = localRay.origin + t * localRay.direction;
    i.point = transform * localPoint;
    i.cubeHit = this;
    i.t = t;
}

bool isOperatingCube(const Cube &c)
{
    return c.type == Cube::RED || c.type == Cube::GREEN || c.type == Cube::BLUE;
}
