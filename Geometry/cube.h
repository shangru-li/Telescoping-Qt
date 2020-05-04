#ifndef CUBE_H
#define CUBE_H

#include "drawable.h"
#include "RayCasting/intersection.h"
#include "RayCasting/ray.h"

#define SIZE 0.5f

class Curve;
class Cube
{
public:
    Cube(glm::mat4 transform, int type = NORMAL);

    bool intersect(const Ray &r, Intersection &i);

    glm::mat4 transform;

    bool selected;
    int type;

    Cube *rootCube, *parentJuncture;
    Curve *parentCurve;

    enum CUBETYPE {RED, GREEN, BLUE, NORMAL, GENERATOR, JUNCTURE};

};

bool isOperatingCube(const Cube &c);
#endif // CUBE_H
