#ifndef CUBE_H
#define CUBE_H

#include "drawable.h"
#include "RayCasting/intersection.h"
#include "RayCasting/ray.h"

#define SIZE 0.5f

class Cube
{
public:
    Cube(glm::mat4 transform, int type = NORMAL);

    bool intersect(const Ray &r, Intersection &i);

    glm::mat4 transform;

    bool selected;
    int type;

    enum CUBETYPE {RED, GREEN, BLUE, NORMAL, GENERATOR};
};

#endif // CUBE_H
