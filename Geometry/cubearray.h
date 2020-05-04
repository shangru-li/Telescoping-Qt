#ifndef CUBEARRAY_H
#define CUBEARRAY_H

#include "cube.h"
class CubeArray : public Drawable
{
public:
    CubeArray(GLContext *context);

    void createGeometry();

    static glm::vec4 p0;
    static glm::vec4 p1;
    static glm::vec4 p2;
    static glm::vec4 p3;
    static glm::vec4 p4;
    static glm::vec4 p5;
    static glm::vec4 p6;
    static glm::vec4 p7;

    std::vector<std::unique_ptr<Cube>> cubes;
    std::unique_ptr<Cube> redArrow, greenArrow, blueArrow;

    Cube &addCube(glm::mat4 transform, Cube *rootCube, int type = Cube::NORMAL);
    bool intersect(const Ray &r, Intersection &intersection) const;



private:
    void addColoredCube(Cube *c, glm::vec4 color = glm::vec4(1, 1, 1, 1), bool isColorful = true);
};

#endif // CUBEARRAY_H
