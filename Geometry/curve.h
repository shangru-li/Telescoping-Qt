#ifndef CURVE_H
#define CURVE_H

#include "drawable.h"
#include "discrete.h"
class Shell : public Drawable
{
public:
    Shell(GLContext *context);

    void createGeometry() override;

    void addCylinder(std::vector<std::vector<glm::vec4>> &cylinder);

    vector<int> ib;
    vector<glm::vec4> vb;
};

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

    void makeTelescope();
    void makeShells();
    void generateGeometry(TelescopeParameters theParams, TelescopeParameters nextParams);
    vector<vector<glm::vec4>> generateCylinder(TelescopeParameters tParams);
    void generateInnerCylinder(TelescopeParameters tParams, float arcOffset);
    vector<glm::vec4> generateCircle(int circNum, glm::vec3 centerPoint, glm::vec3 direction,
                        glm::vec3 normal, float radius);
    int VERTS_PER_CIRCLE = 60;
    int CUTS_PER_CYLINDER = 40;
    glm::fquat getLocalRotationAlongPath(float t, float curvature, float torsion, float length);
    Shell *shell;

    vector<TelescopeParameters> tParams;

    float segmentLength;

    // glm::vec3 reconstructFromAngles();
    void reAssignPoints();

    unique_ptr<vector<CurveSegment>> pSegments;
private:
    // Calculate arc length
    float calcArcLength();
    /*
    void ComputeFrenetFrames();
    void ComputeBishopFrames();

    void FixFrenetFrames();
    void FixFrenetForward(int start);
    void FixFrenetBackward(int start);
    */

    void AddPointsOfSegment(CurveSegment seg);
    glm::vec3 transformedHelixPoint(CurveSegment cs, float arcLen);

    bool hasAssigned, hasTelescope;

};



#endif // CURVE_H

