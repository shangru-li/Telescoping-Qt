#pragma once
#include "global.h"

class Ray
{
public:
    Ray(glm::vec4 origin, glm::vec4 direction);

    Ray transformRay(glm::mat4 t) const;

    glm::vec4 origin, direction;
};
