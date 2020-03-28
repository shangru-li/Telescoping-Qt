#ifndef CURVE_H
#define CURVE_H

#include "drawable.h"
#include "discrete.h"

class Curve : public Drawable
{
public:
    Curve(GLContext *context);
    vector<glm::vec4> *points;
    vector<DCurvePoint> discretePoints;
    vector<glm::vec4> torsionImpulsePoints;
    void createGeometry();
    int drawMode();
    glm::vec3 startingPoint;
    glm::vec3 startingTangent;
    glm::vec3 startingBinormal;
    glm::vec3 targetEndPoint;

//    // Compute discrete points
    void discretilize(float segLength = 0.1f);
//    // Calculte shell fragment number of given curve
    int computeNumImpulses();

    static int numImpulses;

    vector<float> evenlySpacePoints(int num);

    float arcLength;

    void makeImpulseCurve();

    float segmentLength;

    glm::vec3 reconstructFromAngles();
    void reAssignPoints();
private:
    // Calculate arc length
    float calcArcLength();
    void ComputeFrenetFrames();
    void ComputeBishopFrames();
    void FixFrenetFrames();
    void FixFrenetForward(int start);
    void FixFrenetBackward(int start);
    void AddPointsOfSegment(CurveSegment seg);
    glm::vec3 transformedHelixPoint(CurveSegment cs, float arcLen);

    bool reAssignFlag;

};



#endif // CURVE_H

