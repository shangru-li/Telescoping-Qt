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

    void addCube(glm::mat4 transform, int type = Cube::NORMAL);
    bool intersect(const Ray &r, Intersection &intersection) const;


    std::vector<glm::vec4> keys, ctrlPoints, curve;
    void updateCurve();

private:
    void addColoredCube(Cube *c, glm::vec4 color = glm::vec4(1, 1, 1, 1), bool isColorful = true);

    // interpolation
    void generateKeys();
    void computeCtrlPoints();
    void interpolate();

    glm::vec4 interpolateSegment(int segment, float t);
};

#endif // CUBEARRAY_H
