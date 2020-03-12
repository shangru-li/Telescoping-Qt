#include "squareplane.h"

SquarePlane::SquarePlane(GLContext *context): Drawable(context)
{

}

void SquarePlane::createGeometry()
{
    vertexBuffer.clear();
    indexBuffer.clear();

    std::vector<int> idx{0, 1, 2, 0, 2, 3};
    std::vector<int> idx2{4, 5, 6, 4, 6, 7};
    indexBuffer.insert(indexBuffer.end(), idx.begin(), idx.end());
    indexBuffer.insert(indexBuffer.end(), idx2.begin(), idx2.end());

    glm::vec4 p0{0.5, 0.5, 0, 1}, p1{0.5, -0.5, 0, 1}, p2{-0.5, -0.5, 0, 1}, p3{-0.5, 0.5, 0, 1};
    glm::vec4 c0{1, 0, 0, 1}, c1{1, 1, 0, 1}, c2{0, 0, 0.9, 1}, c3{0.1, 0.1, 0.1, 1};
    glm::vec4 n(0, 0, 1, 0);

    std::vector<glm::vec4> v{p0, c0, n, p1, c1, n, p2, c2, n, p3, c3, n};
    glm::vec4 offset(-0.2, 0.3, -1.91, 0);
    std::vector<glm::vec4> v1{p0 + offset, c0, n, p1 + offset, c1, n, p2 + offset, c2, n, p3 + offset, c3, n};
    vertexBuffer.insert(vertexBuffer.end(), v1.begin(), v1.end());
    vertexBuffer.insert(vertexBuffer.end(), v.begin(), v.end());
}
