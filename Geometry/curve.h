#ifndef CURVE_H
#define CURVE_H

#include "drawable.h"
#include "discrete.h"
class Shell : public Drawable
{
public:
    Shell(GLContext *context);

    void createGeometry() override;

    void addCylinder(const std::vector<std::vector<glm::vec4>> &cylinder, glm::vec4 color = _red);

    vector<int> ib;
    vector<glm::vec4> vb;

    vector<vector<vector<glm::vec4>>> cylinders;

    glm::mat4 transform, animatedTransform;
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

    // Compute discrete points
    void discretilize(float segLength = 0.1f);

    // Calculte shell fragment number of given curve
    int computeNumImpulses();

    static int numImpulses;

    vector<float> evenlySpacePoints(int num);

    float arcLength;

    void makeImpulseCurve();

    void makeTelescope();
    void makeShells();
    vector<vector<glm::vec4>> generateCylinder(TelescopeParameters tParams, float nextRadius = 0.f);
    vector<glm::vec4> generateCircle(int circNum, glm::vec3 centerPoint, glm::vec3 direction,
                        glm::vec3 normal, float radius);
    int VERTS_PER_CIRCLE = 100;
    int CUTS_PER_CYLINDER = 40;
    glm::fquat getLocalRotationAlongPath(float t, float curvature, float torsion, float length);
    vector<unique_ptr<Shell>> shells;

    glm::mat4 getCurrentTransform(TelescopeParameters tParams, float t);

    enum ExtensionState { RETRACTED, EXTENDED, RETRACTING, EXTENDING };
    ExtensionState extensionState;
    float extensionExtent;

    vector<TelescopeParameters> tParams;

    float segmentLength;

    // glm::vec3 reconstructFromAngles();
    void reAssignPoints();

    unique_ptr<vector<CurveSegment>> pSegments;

    // Calculate arc length
    float calcArcLength();

    void AddPointsOfSegment(CurveSegment seg);
    glm::vec3 transformedHelixPoint(CurveSegment cs, float arcLen);

    bool hasAssigned, hasTelescope;


    glm::vec3 childBasedPosition(CurveSegment parent, CurveSegment child);
    glm::mat3 childBasedRotation(CurveSegment parent, CurveSegment child);
};



#endif // CURVE_H

