#ifndef CAMERA_H
#define CAMERA_H

#include "global.h"
class Camera
{
public:
    Camera(float aspectRatio, float fovy, glm::vec3 eye, glm::vec3 ref);
    glm::mat4 getViewProj();

    glm::vec3 eye;
    glm::vec3 right, front, up;
    float aspectRatio, fovy;
};

#endif // CAMERA_H
