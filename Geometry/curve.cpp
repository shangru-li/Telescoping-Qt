#include "curve.h"
#include "gurobi_c++.h"
#include "gurobi_c.h"
#include "global.h"


Shell::Shell(GLContext *context)
    : Drawable(context), transform(glm::mat4(1.f)), animatedTransform(glm::mat4(1.f)), junctureAnimatedTransform(glm::mat4(1.f)) {}

void Shell::createGeometry()
{
    indexBuffer.clear();
    vertexBuffer.clear();
    indexBuffer = ib;
    vertexBuffer = vb;
}

void Shell::addCylinder(const std::vector<std::vector<glm::vec4>> &cylinder, glm::vec4 color)
{
    if (!ib.empty()) return;
    for (int i = 0; i < cylinder.size() - 1; ++i)
    {
        for (int j = 0; j < cylinder[0].size() - 1; ++j)
        {
            vector<int> vi{0, 1, 2, 0, 2, 3};
            for (int &k: vi) k += vb.size() / 3;
            ib.insert(ib.end(), vi.begin(), vi.end());
            glm::vec3 v0(cylinder[i][j]), v1(cylinder[i][j+1]), v2(cylinder[i+1][j]);
            glm::vec4 thisNormal = glm::vec4(glm::normalize(glm::cross(v1 - v0, v2 - v0)), 0);
            vector<glm::vec4> vv{cylinder[i][j], color, thisNormal,
                        cylinder[i][j+1], color, thisNormal,
                        cylinder[i+1][j+1], color, thisNormal,
                        cylinder[i+1][j], color, thisNormal};
            vb.insert(vb.end(), vv.begin(), vv.end());
        }

        vector<int> vi{0, 1, 2, 0, 2, 3};
        for (int &k: vi) k += vb.size() / 3;
        ib.insert(ib.end(), vi.begin(), vi.end());
        glm::vec3 v0(cylinder[i][cylinder[0].size() - 1]), v1(cylinder[i][0]), v2(cylinder[i+1][cylinder[0].size() - 1]);
        glm::vec4 thisNormal = glm::vec4(glm::normalize(glm::cross(v1 - v0, v2 - v0)), 0);
        vector<glm::vec4> vv{cylinder[i][cylinder[0].size() - 1], color, thisNormal,
                    cylinder[i][0], color, thisNormal,
                    cylinder[i+1][0], color, thisNormal,
                    cylinder[i+1][cylinder[0].size() - 1], color, thisNormal};
        vb.insert(vb.end(), vv.begin(), vv.end());
    }
}

int Curve::numImpulses = 10;
Curve::Curve(GLContext *context, Cube *parentCube, Cube *childCube): Drawable(context), points(nullptr),
    hasAssigned(false), hasTelescope(false), pSegments(nullptr),
    extensionState(EXTENDED), extensionExtent(1.f),
    parentCube(parentCube), childCube(childCube), endRadius(0.8f)
{
    for (int i = 0; i < numImpulses; ++i)
        shells.push_back(make_unique<Shell>(context));
}

void Curve::createGeometry()
{
    indexBuffer.clear();
    vertexBuffer.clear();
    if (points && points->size() > 1)
    {
        for (unsigned int i = 0; i < points->size() - 1; ++i)
        {
            indexBuffer.push_back(i);
            indexBuffer.push_back(i + 1);
            vertexBuffer.push_back(points->at(i));
            vertexBuffer.push_back(_black);
            vertexBuffer.push_back(_normal);
        }
        vertexBuffer.push_back(points->at(points->size() - 1));
        vertexBuffer.push_back(_black);
        vertexBuffer.push_back(_normal);
    }
}

int Curve::drawMode()
{
    return GL_LINES;
}

int Curve::computeNumImpulses()
{
    return numImpulses;
}

float Curve::calcArcLength()
{
    float re = 0.0f;
    for (unsigned int i = 0; i < points->size() - 1; ++i)
    {
        glm::vec4 p1 = points->at(i);
        glm::vec4 p2 = points->at(i + 1);
        re += glm::length(p2 - p1);
    }
    return re;
}

std::vector<float> Curve::evenlySpacePoints(int numSegments)
{
    float step = arcLength / numSegments;
    vector<float> steps;

    for(int i = 0; i < numSegments; i++)
    {
        steps.push_back(i * step);
    }
    steps.push_back(arcLength);

    return steps;
}

void Curve::makeImpulseCurve()
{
    if (!torsionImpulsePoints.empty()) return;

    int numImpulses = computeNumImpulses();
    vector<float> points = evenlySpacePoints(numImpulses);

    // Given impulses points, solve impulses curve
    GRBEnv env = GRBEnv("impulseQP.log");
    GRBModel model = GRBModel(env);

    vector<GRBVar> sigma;

    GRBVar slope = model.addVar(-1000.0, 1000.0, 0, GRB_CONTINUOUS, "slope");

    // Make each points torsion as variables
    for(int i = 1; i < points.size() - 1; i++)
    {
        string name = "sigma" + to_string(i);
        GRBVar v = model.addVar(-1000.0, 1000.0, 0, GRB_CONTINUOUS, name.c_str());

        sigma.push_back(v);
    }

    vector<GRBVar> absDiffsPlus;
    vector<GRBVar> absDiffsMinus;

    // Variables for absolute diffs
    for(int i = 0; i < sigma.size(); i++)
    {
        string plusName = "diff" + to_string(i) + "+";
        string minusName = "diff" + to_string(i) + "-";
        GRBVar vPlus = model.addVar(0, 1000.0, 0, GRB_CONTINUOUS, plusName.c_str());
        GRBVar vMinus = model.addVar(0, 1000.0, 0, GRB_CONTINUOUS, minusName.c_str());
        absDiffsPlus.push_back(vPlus);
        absDiffsMinus.push_back(vMinus);
    }

    model.update();

    // Add differences constraints
    for(unsigned int i = 0; i < absDiffsMinus.size(); i++)
    {
        GRBVar vPlus = absDiffsPlus.at(i);
        GRBVar vMinus = absDiffsMinus.at(i);
        GRBLinExpr expectedTwist;
        float segLength = points[i + 1] - points[i];
        if(i == 0)
            expectedTwist = slope * segLength;
        else
            expectedTwist = sigma[i - 1] + slope * segLength;
        GRBLinExpr diff = sigma[i] - expectedTwist;

        string constrName = "diffConstr" + to_string(i);
        GRBTempConstr tempConstr = (diff == vPlus - vMinus);
        model.addConstr(tempConstr, constrName.c_str());
    }

    GRBQuadExpr objective = 0;
    float cumulativeTwist = 0;

    // Add least square error component
    for(int i = 0; i < discretePoints.size(); i++)
    {
        cumulativeTwist += glm::radians(discretePoints.at(i).twistingAngle);
        float arcPosition = (i + 1) * segmentLength;

        int stepNum = 0;
        for(int j = 0; j < points.size(); j++)
        {
            if(points[j] > arcPosition) break;
            stepNum = j;
        }

        float arcDistanceFromPt = arcPosition - points.at(stepNum);

        GRBQuadExpr error;
        GRBLinExpr sigma_slope;
        if(stepNum == 0)
        {
            sigma_slope = 0 + slope * arcDistanceFromPt;
        }
        else
        {
            int j = stepNum - 1;
            sigma_slope = sigma[j] + slope * arcDistanceFromPt;
        }
        error = (cumulativeTwist - sigma_slope) * (cumulativeTwist - sigma_slope);
        objective += error;
    }

    double epsilon = 0.1;

    for(int i = 1; i < sigma.size(); i++)
    {
        double arcStep = points.at(i + 1) - points.at(i);
        GRBLinExpr expectedI = sigma.at(i - 1) + arcStep * slope;
        GRBLinExpr impulse = sigma.at(i) - expectedI;

        objective += epsilon * impulse * impulse;
    }

    model.setObjective(objective);
    model.optimize();

    // Read out constant torsion
    float constTorsion = (float)slope.get(GRB_DoubleAttr_X);

    vector<float> impulses;
    impulses.push_back(0);
    double expectedInitial = points.at(1) * constTorsion;
    double initDiff = sigma.at(0).get(GRB_DoubleAttr_X) - expectedInitial;
    impulses.push_back((float)initDiff);

    double sumImpulses = 0;
    int numNonzeroImpulses = 0;

    for(int i = 1; i < sigma.size(); i++)
    {
        double arcStep = points.at(i + 1) - points.at(i);
        double expectedI = sigma.at(i - 1).get(GRB_DoubleAttr_X) + arcStep * constTorsion;
        double diff = sigma.at(i).get(GRB_DoubleAttr_X) - expectedI;
        if(fabs(diff) > 1e-6)
        {
            numNonzeroImpulses++;
            sumImpulses += fabs(diff);
        }
        impulses.push_back((float)diff);
    }

    // Calculate average curvature;
    float averageCurvature = 0.0f;
    for(int i = 0; i < discretePoints.size(); i++)
    {
        averageCurvature += discretePoints.at(i).bendingAngle * M_PI / 180.0f;
    }
    averageCurvature /= discretePoints.size();
    averageCurvature /= segmentLength;

    // Construct curve
    glm::vec3 startingNormal = glm::cross(startingBinormal, startingTangent);
    OrthonormalFrame startFrame = OrthonormalFrame(startingTangent, startingNormal, startingBinormal);

    vector<float> arcSteps;
    for(int i = 0; i < points.size() - 1; i++)
    {
        arcSteps.push_back(points.at(i + 1) - points.at(i));
    }

    // Construct new curve
    if(glm::length(startFrame.T) < 0.01f
            || glm::length(startFrame.B) < 0.01f
            || glm::length(startFrame.N) < 0.01f)
    {
        std::cout<<"Initial frame error!"<<std::endl;
    }

    pSegments = make_unique<vector<CurveSegment>>();
    
    // Create initial segment
    CurveSegment prevHelix = CurveSegment(startingPoint, averageCurvature, 0, arcSteps[0], -constTorsion, startFrame);
    pSegments->push_back(prevHelix);
    AddPointsOfSegment(prevHelix);

    float len = arcSteps[0];

    for(int i = 1; i < impulses.size(); i++)
    {
        float impulse = impulses.at(i);
        float arcStep = arcSteps.at(i);

        glm::vec3 newBase = transformedHelixPoint(prevHelix, prevHelix.arcLength);
        OrthonormalFrame newFrame = transformedHelixFrame(prevHelix, prevHelix.arcLength);
        glm::fquat impulseRot = glm::angleAxis(glm::degrees(impulse), newFrame.T);
        newFrame = newFrame.RotateBy(impulseRot);

        prevHelix = CurveSegment(newBase, averageCurvature, 0, arcStep, -constTorsion, newFrame);

        len += arcStep;
        AddPointsOfSegment(prevHelix);
        pSegments->push_back(prevHelix);
    }

    // Add last point
    torsionImpulsePoints.push_back(glm::vec4(transformedHelixPoint(prevHelix, arcSteps.at(arcSteps.size() - 1)), 1.0f));
    this->points->clear();
    this->points->insert(this->points->end(), torsionImpulsePoints.begin(), torsionImpulsePoints.end());
}

void Curve::makeTelescope(float radius)
{
    if (hasTelescope || !pSegments) return;
    hasTelescope = true;

    float currRadius = radius, WALL_THICKNESS = radius / 1.5f / (Curve::numImpulses - 1);
    pl(WALL_THICKNESS);
    tParams.clear();

    if (parentCube->parentCurve)
    {
        CurveSegment lastShell = parentCube->parentCurve->pSegments->back();
        glm::vec3 offset = translateAlongHelix(lastShell.curvature, lastShell.torsion, lastShell.arcLength);
        glm::mat3 frame = glm::mat3_cast(rotateAlongHelix(lastShell.curvature, lastShell.torsion, lastShell.arcLength));
        glm::mat4 localFrame(glm::vec4(frame[0], 0), glm::vec4(frame[1], 0), glm::vec4(frame[2], 0), glm::vec4(offset, 1));

        glm::vec4 ref = parentCube->parentCurve->shells.back()->transform * localFrame * _black;
        glm::vec3 diff = glm::vec3(ref) - pSegments->at(0).startPosition;
        for (CurveSegment &cs: *pSegments)
        {
            cs.startPosition += diff;
        }
    }

    CurveSegment initialSeg = pSegments->at(0);
    TelescopeParameters inital(initialSeg.arcLength, currRadius,
                               WALL_THICKNESS, initialSeg.curvature, initialSeg.torsion, 0, initialSeg.startPosition, initialSeg.frame);
    tParams.push_back(inital);

    for (int i = 1; i < pSegments->size(); ++i)
    {
        currRadius -= WALL_THICKNESS;
        TelescopeParameters p(pSegments->at(i).arcLength, currRadius,
                              WALL_THICKNESS, pSegments->at(i).curvature, pSegments->at(i).torsion, 0, pSegments->at(i).startPosition, pSegments->at(i).frame);
        tParams.push_back(p);
    }

    endRadius = currRadius;

    makeShells();
}

void Curve::makeShells()
{
    for (int i = 0; i < tParams.size(); ++i)
    {
        glm::vec4 color1 = glm::vec4(1, 0.6, 0.6, 1), color2 = glm::vec4(0.6, 0.6, 1, 1);
        shells[i]->addCylinder(generateCylinder(tParams[i]), i % 2 ? color1 : color2);
        OrthonormalFrame f = tParams[i].frame;
        shells[i]->transform = glm::mat4(glm::vec4(f.B, 0), glm::vec4(f.N, 0), glm::vec4(f.T, 0), glm::vec4(tParams[i].startPosition, 1));
        shells[i]->animatedTransform = shells[i]->transform;
    }
}

vector<vector<glm::vec4>> Curve::generateCylinder(TelescopeParameters tParams, float nextRadius)
{
    vector<vector<glm::vec4>> circles;
    float lengthStep = 1.f / (CUTS_PER_CYLINDER - 1);

    for (int i = 0; i < CUTS_PER_CYLINDER; i++)
    {
        float currArcLength = i * lengthStep;
        glm::vec3 centerPoint = translateAlongHelix(tParams.curvature, tParams.torsion, currArcLength * tParams.length);
        glm::vec3 facingDirection = glm::mat3_cast(getLocalRotationAlongPath(i * lengthStep, tParams.curvature, tParams.torsion, tParams.length)) * glm::vec3(0, 0, 1);
        glm::vec3 normalDirection = glm::mat3_cast(getLocalRotationAlongPath(i * lengthStep, tParams.curvature, tParams.torsion, tParams.length)) * glm::vec3(0, 1, 0);
        vector<glm::vec4> circle;
        circle = generateCircle(i, centerPoint, facingDirection, normalDirection, nextRadius == 0.f ? tParams.radius : nextRadius);
        circles.push_back(circle);
    }

    return circles;
}

vector<glm::vec4> Curve::generateCircle(int circNum, glm::vec3 centerPoint, glm::vec3 direction, glm::vec3 normal, float radius)
{
    vector<glm::vec4> theCirc;

    float angleStep = (2 * glm::pi<float>()) / VERTS_PER_CIRCLE;

    glm::fquat circleRotation = glm::rotation(glm::vec3(0, 0, 1), direction);
    for (int i = 0; i < VERTS_PER_CIRCLE; ++i)
    {
        float currentAngle = i * angleStep;
        glm::vec3 vert(glm::cos(currentAngle), glm::sin(currentAngle), 0);
        vert *= radius;
        vert = glm::mat3_cast(circleRotation) * vert;
        vert += centerPoint;
        theCirc.push_back(glm::vec4(vert, 1));
    }
    return theCirc;
}

glm::fquat Curve::getLocalRotationAlongPath(float t, float curvature, float torsion, float length)
{
    if (curvature > 1e-6)
    {
        // Convert normalized length to arc length.
        float arcLength = t * length;
        glm::fquat rotation = rotateAlongHelix(curvature, torsion, arcLength);
        return rotation;
    }
    else return glm::fquat(1, 0, 0, 0);
}

glm::mat4 Curve::getCurrentTransform(TelescopeParameters tParams, float t)
{
    glm::vec3 location = translateAlongHelix(tParams.curvature, tParams.torsion, t * tParams.length);
    glm::mat3 rotation = glm::mat3_cast(getLocalRotationAlongPath(t, tParams.curvature, tParams.torsion, tParams.length));
    return glm::mat4(glm::vec4(rotation[0], 0), glm::vec4(rotation[1], 0), glm::vec4(rotation[2], 0), glm::vec4(location, 1));
}

void Curve::discretilize(float segLength)
{
    if(!discretePoints.empty()) return;

    segmentLength = segLength;
    startingPoint = glm::vec3(points->at(0));
    startingTangent = glm::normalize(glm::vec3(points->at(1) - points->at(0)));

    glm::vec3 prevBinormal = glm::vec3(0.0f);

    for (int i = 1; i < points->size() - 1; i++)
    {
        glm::vec3 previousVec = glm::normalize(glm::vec3(points->at(i) - points->at(i - 1)));
        glm::vec3 nextVec = glm::normalize(glm::vec3(points->at(i + 1) - points->at(i)));

        glm::vec3 curvatureBinormal = glm::normalize(glm::cross(previousVec, nextVec));
        if (i == 1) startingBinormal = curvatureBinormal;

        // Compute bending angles(curvature)
        float dot = glm::dot(previousVec, nextVec);
        float bendAngle = (dot >= 1) ? 0 : 180.0f * acos(dot) / M_PI;

        // Compute twist angles （discrete torsion）
        float twistAngle;
        if (i == 1) twistAngle = 0;
        else twistAngle = angleBetween(prevBinormal, curvatureBinormal, previousVec);

        if (isnan(bendAngle)) cout<<"Bend angle is nan"<<endl;
        if (isnan(twistAngle)) cout<<"Twist angle is nan"<<endl;

        prevBinormal = curvatureBinormal;

        DCurvePoint dcp = DCurvePoint(curvatureBinormal, bendAngle, twistAngle);
        discretePoints.push_back(dcp);
    }
}

void Curve::AddPointsOfSegment(CurveSegment seg)
{
    int numSegments = glm::ceil(seg.arcLength / 0.1f);
    float segLength = seg.arcLength / numSegments;
    float cumulativeLength = 0;
    for(int i = 0; i < numSegments; i++)
    {
        torsionImpulsePoints.push_back(glm::vec4(transformedHelixPoint(seg, cumulativeLength), 1.0f));
        cumulativeLength += segLength;
    }
}

glm::vec3 Curve::transformedHelixPoint(CurveSegment cs, float arcLen)
{
    glm::vec3 helixPoint = translateAlongHelix(cs.curvature, cs.torsion, arcLen);
    glm::fquat rotatedToWorld = lookRotation(cs.frame.T, cs.frame.N);
    glm::vec3 world = glm::mat3_cast(rotatedToWorld) * helixPoint + cs.startPosition;
    return world;
}

glm::vec3 Curve::childBasedPosition(CurveSegment parent, CurveSegment child)
{
    glm::vec3 translationToBase = translateAlongHelix(parent.curvature, parent.torsion, parent.arcLength);
    glm::fquat rotationToBase = rotateAlongHelix(parent.curvature, parent.torsion, parent.arcLength);
    glm::vec3 translationBackwards = translateAlongHelix(child.curvature, child.torsion, -child.arcLength);

    translationToBase = translationToBase + (glm::mat3_cast(rotationToBase) * translationBackwards);
    return translationToBase;
}

glm::mat3 Curve::childBasedRotation(CurveSegment parent, CurveSegment child)
{
    glm::fquat rotationToBase = rotateAlongHelix(parent.curvature, parent.torsion, parent.arcLength);
    glm::fquat rotationBack = rotateAlongHelix(child.curvature, child.torsion, -child.arcLength);
    return glm::mat3_cast(rotationToBase * rotationBack);
}


float getCurrentState(float t, int num, int grid, bool junctureMode = false)
{
    if (junctureMode && grid == 0) return 1;
    float step = 1.f / num;
    int currNum = t / step;
    if (grid < currNum) return 1;
    if (grid > currNum) return 0;
    return (t - currNum * step) / step;
}

void Curve::updateSegmentTransforms()
{
    if (!tParams.empty())
    {
        for (int i = 0; i < tParams.size() - 1; ++i)
        {
            float currentState = getCurrentState(extensionExtent, numImpulses - 1, i, true);
            currentState *= 3.f;
            if (currentState > 1.f) currentState = 1.f;

            CurveSegment cs1 = pSegments->at(i+1);
            glm::vec3 position = transformedHelixPoint(cs1, (1 - currentState) * -cs1.arcLength);
            OrthonormalFrame f = transformedHelixFrame(cs1, (1 - currentState) * -cs1.arcLength);

            glm::mat4 relative = glm::mat4(glm::vec4(f.B, 0), glm::vec4(f.N, 0), glm::vec4(f.T, 0), glm::vec4(position, 1));
            shells[i+1]->animatedTransform = shells[i]->animatedTransform * glm::inverse(shells[i]->transform) * relative;
        }
        Curve *parentCurve = parentCube->parentCurve;
        if (parentCurve)
        {
            glm::mat4 parentTransform = parentCurve->shells.back()->transform;
            glm::mat4 parentAnimatedTransform = parentCurve->shells.back()->junctureAnimatedTransform;
            for (unique_ptr<Shell> &pShell: shells)
            {
                pShell->junctureAnimatedTransform = parentAnimatedTransform * glm::inverse(parentTransform) * pShell->animatedTransform;
            }
        }
        else
        {
            for (unique_ptr<Shell> &pShell: shells)
            {
                pShell->junctureAnimatedTransform = pShell->animatedTransform;
            }
        }
    }
}

void Curve::reAssignPoints()
{
    if (hasAssigned) return;
    hasAssigned = true;

    arcLength = calcArcLength();
    float cumuArc = 0.0f;
    int curSeg = 0;
    std::vector<glm::vec4> curPoints;
    std::vector<glm::vec4> newPoints;

    curPoints.push_back(points->at(0));

    for(int i = 1; i < points->size(); i++)
    {
        float length = glm::length(points->at(i) - points->at(i - 1));
        cumuArc += length;
        int seg = cumuArc / 0.1f;
        if(seg > curSeg)
        {
            curSeg++;
            glm::vec4 newPoint = glm::vec4(0.0f);
            for(int j = 0; j < curPoints.size(); j++)
            {
                newPoint += curPoints.at(j);
            }
            newPoint /= (float)curPoints.size();
            newPoints.push_back(newPoint);
            curPoints.clear();
            curPoints.push_back(points->at(i));
        }
        else
        {
            curPoints.push_back(points->at(i));
        }
    }

    // add last point
    glm::vec4 newPoint = glm::vec4(0.0f);
    for(int j = 0; j < curPoints.size(); j++)
    {
        newPoint += curPoints.at(j);
    }
    newPoint /= (float)curPoints.size();
    newPoints.push_back(newPoint);

    points->clear();
    points->insert(points->end(), newPoints.begin(), newPoints.end());
}






void Curve::generateKeys()
{
    keys.clear();
    for (int i = 0; i < curveCubes.size(); ++i)
    {
        Cube *c = curveCubes[i];
        if (!isOperatingCube(*c))
        {
            keys.push_back(c->transform * _black);
        }
    }
}

void Curve::computeCtrlPoints()
{
    ctrlPoints.clear();
    if (keys.size() <= 1) return;
    glm::vec4 startPoint = keys[0] + 0.25f * (keys[0] - keys[1]);
    glm::vec4 endPoint = keys[keys.size() - 1] + 0.25f * (keys[keys.size() - 1] - keys[keys.size() - 2]);
    for (int i = 1; i < keys.size(); ++i)
    {
        glm::vec4 b0, b1, b2, b3, t0, t3;
        b0 = keys[i - 1], b3 = keys[i];
        if (i == 1) t0 = (keys[i] - startPoint) / 2.f;
        else t0 = (keys[i] - keys[i - 2]) / 2.f;
        b1 = b0 + t0 / 3.f;
        if (i == keys.size() - 1) t3 = (endPoint - keys[i - 1]) / 2.f;
        else t3 = (keys[i + 1] - keys[i - 1]) / 2.f;
        b2 = b3 - t3 / 3.f;
        std::vector<glm::vec4> v{b0, b1, b2, b3};
        ctrlPoints.insert(ctrlPoints.end(), v.begin(), v.end());
    }
}

void Curve::interpolate()
{
    curve.clear();
    if (keys.size() <= 1) return;
    for (int segment = 0; segment < keys.size() - 1; ++segment)
    {
        for (float t = 0.f; t < 1.f - std::numeric_limits<float>::min(); t += 0.01f)
        {
            curve.push_back(interpolateSegment(segment, t));
        }
    }
    curve.push_back(interpolateSegment(keys.size() - 2, 1.f));
}

glm::vec4 Curve::interpolateSegment(int segment, float t)
{
    glm::vec4 b0 = ctrlPoints[segment * 4 + 0],
            b1 = ctrlPoints[segment * 4 + 1],
            b2 = ctrlPoints[segment * 4 + 2],
            b3 = ctrlPoints[segment * 4 + 3],
            p00 = b0 + (b1 - b0) * t,
            p10 = b1 + (b2 - b1) * t,
            p20 = b2 + (b3 - b2) * t,
            p01 = p00 + (p10 - p00) * t,
            p11 = p10 + (p20 - p10) * t,
            p02 = p01 + (p11 - p01) * t;
    return p02;
}

void Curve::updateCurve()
{
    generateKeys();
    computeCtrlPoints();
    interpolate();
}
