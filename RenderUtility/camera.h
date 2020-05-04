#ifndef CAMERA_H
#define CAMERA_H

#include "global.h"
#include "RayCasting/ray.h"
class Camera
{
public:
    Camera(float aspectRatio, float fovy, glm::vec3 eye, glm::vec3 ref);
    glm::mat4 getViewProj();

    void rotateSpherical(float deg, glm::vec3 axis);
    void zoom(float length);
    void pan(glm::vec3 direction);

    Ray castRay(float x, float y);

    glm::vec3 eye, ref;
    glm::vec3 right, front, up;
    float aspectRatio, fovy;
};

#endif // CAMERA_H
