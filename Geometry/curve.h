#ifndef CURVE_H
#define CURVE_H

#include "drawable.h"

class Curve : public Drawable
{
public:
    Curve(GLContext *context);
    std::vector<glm::vec4> *points;
    void createGeometry();
    int drawMode();
};

#endif // CURVE_H
