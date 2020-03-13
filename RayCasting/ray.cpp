#include "ray.h"

Ray::Ray(glm::vec4 origin, glm::vec4 direction)
    : origin(origin), direction(direction)
{

}

Ray Ray::transformRay(glm::mat4 t) const
{
    return Ray(t * origin, t * direction);
}
