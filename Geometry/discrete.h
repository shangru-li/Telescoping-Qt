#pragma once
#ifndef DISCRETE_H
#define DISCRETE_H
#include "global.h"

class OrthonormalFrame
{
public:
    glm::vec3 T;
    glm::vec3 B;
    glm::vec3 N;

    OrthonormalFrame() {}

    OrthonormalFrame(glm::vec3 t, glm::vec3 n, glm::vec3 b) :
        T(t), B(b), N(n) {}

    OrthonormalFrame RotateBy(glm::fquat rot)
    {
        glm::vec3 rotT = glm::mat3_cast(rot) * T;
        glm::vec3 rotB = glm::mat3_cast(rot) * B;
        glm::vec3 rotN = glm::mat3_cast(rot) * N;

        return OrthonormalFrame(rotT, rotN, rotB);
    }

    /*
    static OrthonormalFrame Slerp(OrthonormalFrame frame1, OrthonormalFrame frame2, float t)
    {
        glm::vec3 tangent1 = frame1.T;
        glm::vec3 tangent2 = frame2.T;
        glm::fquat tangentQuat = glm::slerp(glm::fquat(tangent1.x, tangent1.y, tangent1.z, 0.0f),
                                          glm::fquat(tangent2.x, tangent2.y, tangent2.z, 0.0f), t);
        glm::vec3 tangent = glm::vec3(tangentQuat.x, tangentQuat.y, tangentQuat.z);

        glm::vec3 binormal1 = frame1.B;
        glm::vec3 binormal2 = frame2.B;
        glm::fquat binormalQuat = glm::slerp(glm::fquat(binormal1.x, binormal1.y, binormal1.z, 0.0f),
            glm::fquat(binormal2.x, binormal2.y, binormal2.z, 0.0f), t);
        glm::vec3 binormal = glm::vec3(binormalQuat.x, binormalQuat.y, binormalQuat.z);

        glm::vec3 normal1 = frame1.T;
        glm::vec3 normal2 = frame2.T;
        glm::fquat normalQuat = glm::slerp(glm::fquat(normal1.x, normal1.y, normal1.z, 0.0f),
            glm::fquat(normal2.x, normal2.y, normal2.z, 0.0f), t);
        glm::vec3 normal = glm::vec3(normalQuat.x, normalQuat.y, normalQuat.z);

        return OrthonormalFrame(tangent, normal, binormal);
    }
    */
};

// User defined curve point
class DCurvePoint
{
public:
    glm::vec3 binormal;
    float bendingAngle;
    float twistingAngle;

    float cumulativeTwist;

    glm::vec3 position;
    glm::vec3 currentGradient;

    // Tangent space
    OrthonormalFrame frenetFrame;
    OrthonormalFrame bishopFrame;

    DCurvePoint(glm::vec3 dir, float bendAngle, float twistAngle) :
        binormal(dir), bendingAngle(bendAngle), twistingAngle(twistAngle) {}

    /*
    void ComputeFrenet(glm::vec3 prevPos, glm::vec3 nextPos)
    {
        pl(position, "pos");
        glm::vec3 tangent = glm::normalize(nextPos - position);
        glm::vec3 prevTangent = glm::normalize(position - prevPos);
        glm::vec3 binormal = glm::normalize(glm::cross(prevTangent, tangent));

        glm::vec3 normal = glm::cross(binormal, tangent);

        frenetFrame = OrthonormalFrame(tangent, normal, binormal);
    }

    void ComputeFrenet(glm::vec3 prevPos, DCurvePoint next)
    {
        ComputeFrenet(prevPos, next.position);
    }

    void ComputeFrenet(DCurvePoint prev, DCurvePoint next)
    {
        ComputeFrenet(prev.position, next.position);
    }

    OrthonormalFrame PropagateBishop(glm::vec3 prevPos, glm::vec3 nextPos, OrthonormalFrame prevFrame)
    {
        glm::vec3 tangent = glm::normalize(nextPos - position);
        glm::vec3 prevTangent = glm::normalize(position - prevPos);
        glm::vec3 binormal = glm::normalize(glm::cross(prevTangent, tangent));

        float angle = angleBetween(prevTangent, tangent, binormal);

        glm::fquat rot = glm::angleAxis(glm::degrees(angle), binormal);

        OrthonormalFrame rotated = prevFrame.RotateBy(rot);
        bishopFrame = rotated;
        return rotated;
    }

    OrthonormalFrame PropagateBishop(DCurvePoint prev, DCurvePoint next, OrthonormalFrame prevFrame)
    {
        return PropagateBishop(prev.position, next.position, prevFrame);
    }
    */
};

class CurveSegment
{
public:
    glm::vec3 startPosition;
    float curvature;
    float impulse;
    float arcLength;
    float torsion;
    OrthonormalFrame frame;

    CurveSegment(glm::vec3 pos, float curv, float imp, float arc, float tor, OrthonormalFrame _frame):
        startPosition(pos), curvature(curv), impulse(imp), arcLength(arc), torsion(tor), frame(_frame) {}
};

struct TelescopeParameters
{
    float length;
    float radius;
    float thickness;
    float curvature;
    float torsion;
    float twistFromParent;

    glm::vec3 startPosition;
    OrthonormalFrame frame;

    TelescopeParameters(float length, float radius, float thickness,
        float curvature, float torsion, float twistImpulse,
                        glm::vec3 startPosition, OrthonormalFrame frame):
            length(length),
            radius(radius),
            thickness(thickness),
            curvature(curvature),
            torsion(torsion),
            twistFromParent(twistImpulse),
    startPosition(startPosition), frame(frame){}
};

static glm::fquat rotateAlongCircle(float curvatureAmount, float arcLength);
static glm::vec3 translateAlongCircle(float curvatureAmount, float arcLength);

static OrthonormalFrame frameAlongHelix(float curvature, float torsion, float arcLength)
{
    if (fabs(torsion) < 1e-6)
    {
        OrthonormalFrame defaultFrame = OrthonormalFrame(glm::vec3(0.0f, 0.0f, 1.0f),
                                                    glm::vec3(0.0f, 1.0f, 0.0f),
                                                    glm::cross(glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 1.0f, 0.0f)));
        glm::fquat r = rotateAlongCircle(curvature, arcLength);
        return defaultFrame.RotateBy(r);
    }
    // Torsion but no curvature = rotate about forward axis in a screw motion
    if (curvature < 1e-6)
    {
        OrthonormalFrame defaultFrame = OrthonormalFrame(glm::vec3(0.0f, 0.0f, 1.0f),
                                                         glm::vec3(0.0f, 1.0f, 0.0f),
                                                         glm::cross(glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 1.0f, 0.0f)));
        float rotationAngle = torsion * arcLength;
        glm::fquat r = glm::angleAxis(glm::degrees(rotationAngle), glm::vec3(0.0f, 0.0f, 1.0f));
        return defaultFrame.RotateBy(r);
    }

    float sumSq = curvature * curvature + torsion * torsion;
    float a = curvature / sumSq;
    float b = torsion / sumSq;
    float abSqrt = sqrt(a * a + b * b);

    float t = arcLength;

    glm::vec3 tangent = glm::vec3(b,
                                  -a * sin(t / abSqrt),
                                  a * cos(t / abSqrt)) / abSqrt;
    tangent.y *= -1;
    tangent = glm::normalize(tangent);

    glm::vec3 normal = glm::vec3(0,
                                 cos(t / abSqrt),
                                 sin(t / abSqrt)) * -1.0f;
    normal.y *= -1;
    normal = glm::normalize(normal);

    glm::vec3 binormal = glm::cross(tangent, normal);

    return OrthonormalFrame(tangent, normal, binormal);
}

static glm::fquat rotateAlongCircle(float curvatureAmount, float arcLength)
{
    if (curvatureAmount < 1e-6) return glm::fquat();
    // Compute how many radians we moved.
    float radians = arcLength * curvatureAmount; // + baseRadians;

    // Now rotate the forward vector by that amount.
    glm::vec3 axisOfRotation = glm::vec3(0.0f, 0.0f, 1.0f);
    //float degrees = radians / (M_PI / 180.0f);
    glm::fquat rotation = glm::angleAxis(glm::degrees(radians), axisOfRotation);
    return rotation;
}

static glm::fquat rotateAlongHelix(float curvature, float torsion, float arcLength)
{
    if (fabs(torsion) < 1e-6)
    {
        return rotateAlongCircle(curvature, arcLength);
    }
    // Torsion but no curvature = rotate about forward axis in a screw motion
    if (curvature < 1e-6)
    {
        float rotationAngle = torsion * arcLength;
        return glm::angleAxis(glm::degrees(rotationAngle), glm::vec3(0.0f, 0.0f, 1.0f));
    }

    // Correct because the initial tangent of a helix isn't (0, 0, 1),
    // but we want it to be for linking up of helical pieces.
    OrthonormalFrame zeroFrame = frameAlongHelix(curvature, torsion, 0);
    glm::fquat correctiveR = glm::inverse(lookRotation(zeroFrame.T, zeroFrame.N));

    OrthonormalFrame frame = frameAlongHelix(curvature, torsion, arcLength);

    // Corrective rotation so that initial tangent is forward.
    //Quaternion r = Quaternion.FromToRotation(Vector3.forward, Vector3.down);

    return correctiveR * lookRotation(frame.T, frame.N);
}

static glm::vec3 translateAlongHelix(float curvature, float torsion, float arcLength)
{
    if(torsion < 1e-6) return translateAlongCircle(curvature, arcLength);
    if(curvature < 1e-6) return glm::vec3(0.0f, 0.0f, 1.0f) * arcLength;

    OrthonormalFrame zeroFrame = frameAlongHelix(curvature, torsion, 0.0f);
    glm::fquat correctiveR = glm::inverse(lookRotation(zeroFrame.T, zeroFrame.N));

    float sumSq = curvature * curvature + torsion * torsion;
    float a = curvature / sumSq;
    float b = torsion / sumSq;

    float abSqrt = sqrt(a * a + b * b);

    float t = arcLength;

    glm::vec3 pos = glm::vec3(b * t / abSqrt,
                              a * cos(t / abSqrt),
                              a * sin(t / abSqrt));

    pos.y *= -1;

    // Shift so that (0, a, 0) is the center and (0, 0, 0) is the first point.
    pos += (a * glm::vec3(0.0f, 1.0f, 0.0f));
    return correctiveR * pos;
}

static glm::vec3 translateAlongCircle(float curvatureAmount, float arcLength)
{
    if (curvatureAmount > 1e-6)
    {
        float curvatureRadius = 1.0f / curvatureAmount;

        // Start at the bottom of the circle.
        float baseAngle = 3.0f * M_PI / 2.0f; // + baseRadians;

        // Compute how many radians we moved.
        float radians = arcLength * curvatureAmount;
        float finalAngle = baseAngle + radians;

        // Compute on the circle centered at (0, 0, 0),
        // and then add (0, r, 0).
        glm::vec3 center = glm::vec3(0.0f, 1.0f, 0.0f) * curvatureRadius;
        glm::vec3 displacement = glm::vec3(0.0f);
        displacement.z = cos(finalAngle) * curvatureRadius;
        displacement.y = sin(finalAngle) * curvatureRadius;
        displacement += center;
        return displacement;
    }
    else
    {
        glm::vec3 displacement = arcLength * glm::vec3(0.0f, 0.0f, 1.0f);
        return displacement;
    }
}

static OrthonormalFrame transformedHelixFrame(CurveSegment cs, float arcLen)
{
   // Apply the local helix rotation.
   glm::fquat oldRot = lookRotation(cs.frame.T, cs.frame.N);
   glm::fquat newRot = oldRot * rotateAlongHelix(cs.curvature, cs.torsion, arcLen) * glm::inverse(oldRot);
   OrthonormalFrame newFrame = cs.frame.RotateBy(newRot);
   return newFrame;
}

#endif // DISCRETE_H
