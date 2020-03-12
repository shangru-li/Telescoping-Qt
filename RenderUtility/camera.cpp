#include "camera.h"

Camera::Camera(float aspectRatio, float fovy, glm::vec3 eye, glm::vec3 ref)
    : eye(eye), aspectRatio(aspectRatio), fovy(fovy)
{
    front = glm::normalize(ref - eye);
    right = glm::cross(front, glm::vec3(0, 1, 0));
    up = glm::cross(right, front);
}

glm::mat4 Camera::getViewProj()
{
    //pl(glm::perspective(glm::radians(fovy), aspectRatio, 0.1f, 1000.f));
    //pl(glm::lookAt(eye, eye - front, up));
    return glm::perspective(glm::radians(fovy), aspectRatio, 0.1f, 1000.f) * glm::lookAt(eye, eye + front, up);

    return glm::mat4{glm::vec4{-1, 0, 0, 0},
        glm::vec4{0, 1, 0, 0},
        glm::vec4{0, 0, 1, 0},
        glm::vec4{0, 0, 1, 1},
       } * glm::mat4{glm::vec4{0.6667, 0, 0, 0},
        glm::vec4{0, 1, 0, 0},
        glm::vec4{0, 0, 1.0001, 1},
        glm::vec4{0, 0, -0.10001, 0}
        };
}
