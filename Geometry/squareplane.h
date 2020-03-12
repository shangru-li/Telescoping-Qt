#ifndef SQUAREPLANE_H
#define SQUAREPLANE_H
#include "drawable.h"

class SquarePlane : public Drawable
{
public:
    SquarePlane(GLContext *context);

    void createGeometry() override;
};

#endif // SQUAREPLANE_H
