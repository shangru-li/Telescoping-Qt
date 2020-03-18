#include "curve.h"
#include "gurobi_c++.h"
#include "gurobi_c.h"
#include "global.h"

int Curve::numImpulses = 6;
Curve::Curve(GLContext *context): Drawable(context), points(nullptr)
{

}

void Curve::createGeometry()
{
    indexBuffer.clear();
    vertexBuffer.clear();
    if (points && points->size() > 1)
    {
        for(int i = 0; i < points->size() - 1; ++i)
        {
            indexBuffer.push_back(i);
            indexBuffer.push_back(i + 1);
            vertexBuffer.push_back((*points)[i]);
            vertexBuffer.push_back(_white);
            vertexBuffer.push_back(_normal);
        }
        vertexBuffer.push_back((*points)[points->size() - 1]);
        vertexBuffer.push_back(_white);
        vertexBuffer.push_back(_normal);
    }
    //calcArcLength();
}

int Curve::drawMode()
{
    return GL_LINES;
}

int Curve::computeNumImpulses()
{
    int maxNumShells = std::floor(arcLength / _MIN_SHELL_LENGTH);

    // Deal with junctures (implment later)

    return numImpulses;
}

float Curve::calcArcLength()
{
    float re = 0.0f;
    for(unsigned int i = 0; i < points->size() - 1; i++)
    {
        glm::vec4 p1 = points->at(i);
        glm::vec4 p2 = points->at(i + 1);
        re += glm::length(p2 - p1);
    }
    return re;
}

std::vector<float> Curve::evenlySpacePoints(int num)
{
    float step = arcLength / num;
    vector<float> steps;

    for(int i = 0; i < num; i++)
    {
        steps.push_back(i * step);
    }
    steps.push_back(arcLength);

    return steps;
}

void Curve::makeImpulseCurve()
{
    if(!torsionImpulsePoints.empty())
    {
        return;
    }
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
    for(int i = 0; i < absDiffsMinus.size(); i++)
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
        cumulativeTwist += M_PI * discretePoints.at(i).twistingAngle / 180.0f;
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

    vector<glm::vec4> newPoints;
    vector<CurveSegment> segments;
    
    // Create initial segment
    CurveSegment prevHelix = CurveSegment(startingPoint, averageCurvature, 0, arcSteps[0], constTorsion, startFrame);
    segments.push_back(prevHelix);
    AddPointsOfSegment(prevHelix);

    float len = arcSteps[0];

    for(int i = 1; i < impulses.size(); i++)
    {
        float impulse = impulses.at(i);
        float arcStep = arcSteps.at(i);

        glm::vec3 newBase = transformedHelixPoint(prevHelix, prevHelix.arcLength);
        OrthonormalFrame newFrame = transformedHelixFrame(prevHelix, prevHelix.arcLength);
        glm::fquat impulseRot = glm::fquat(180.0f * impulse / M_PI, newFrame.T.x, newFrame.T.y, newFrame.T.z);
        newFrame = newFrame.RotateBy(impulseRot);

        prevHelix = CurveSegment(newBase, averageCurvature, 0, arcStep, constTorsion, newFrame);

        len += arcStep;
        AddPointsOfSegment(prevHelix);
        segments.push_back(prevHelix);
    }

    // Add last point
    torsionImpulsePoints.push_back(glm::vec4(transformedHelixPoint(prevHelix, arcSteps.at(arcSteps.size() - 1)), 1.0f));
    this->points->clear();
    for(int i = 0; i < torsionImpulsePoints.size(); i++)
    {
        this->points->push_back(torsionImpulsePoints.at(i));
    }

}

void Curve::discretilize(float segLength)
{
    //std::cout<"Discretilize!"<<std::endl;
    if(!discretePoints.empty())
    {
        return;
    }
    arcLength = calcArcLength();
    segmentLength = segLength;
    startingPoint = glm::vec3(points->at(0));
    startingTangent = glm::normalize(glm::vec3(points->at(1) - points->at(0)));

    glm::vec3 prevBinormal = glm::vec3(0.0f);

    for(int i = 1; i < points->size() - 1; i++)
    {
        glm::vec3 previousVec = glm::normalize(glm::vec3(points->at(i) - points->at(i - 1)));
        glm::vec3 nextVec = glm::normalize(glm::vec3(points->at(i + 1) - points->at(i)));

        if(glm::length(nextVec) < 0.5f)
        {
            DCurvePoint d = DCurvePoint(prevBinormal, 0.0f, 0.0f);
            discretePoints.push_back(d);
            continue;
        }

        glm::vec3 curvatureBinormal = glm::normalize(glm::cross(previousVec, nextVec));
        if(i == 1)
            startingBinormal = curvatureBinormal;

        // Compute bending angles(curvature)
        float dot = glm::dot(previousVec, nextVec);
        float bendAngle = (dot >= 1) ? 0 : 180 * acos(dot) / M_PI;

        // Compute twist angles （discrete torsion）
        float twistAngle;
        if(i == 1)
            twistAngle = 0;
        else
        {
            twistAngle = angleBetween(prevBinormal, curvatureBinormal, previousVec);
        }

        if(isnan(bendAngle))
            cout<<"Bend angle is nan"<<endl;

        if(isnan(twistAngle))
            cout<<"Twist angle is nan"<<endl;

        prevBinormal = curvatureBinormal;

        DCurvePoint dcp = DCurvePoint(glm::normalize(curvatureBinormal), bendAngle, twistAngle);
        discretePoints.push_back(dcp);
    }

    if(glm::length(startingBinormal) < 0.001f)
    {
        if(startingTangent == glm::vec3(0.0f, 1.0f, 0.0f))
            startingTangent = glm::vec3(1.0f, 0.0f, 0.0f);
        else
        {
            startingBinormal = glm::vec3(0.0f, 1.0f, 0.0f);
            glm::vec3 orthogonal = glm::dot(startingBinormal, startingTangent) * startingTangent;
            startingBinormal = glm::normalize(startingBinormal - orthogonal);
        }
    }

    targetEndPoint = reconstructFromAngles();
    ComputeFrenetFrames();
    ComputeBishopFrames();

}

glm::vec3 Curve::reconstructFromAngles()
{
    points->clear();

    glm::vec3 currentPoint = startingPoint;
    points->push_back(glm::vec4(currentPoint, 1.0f));
    glm::vec3 currentDir = startingTangent;

    currentPoint += currentDir * segmentLength;
    points->push_back(glm::vec4(currentPoint, 1.0f));

    glm::vec3 currentBinormal = startingBinormal;

    for(int i = 0; i < discretePoints.size(); i++)
    {
        DCurvePoint dcp = discretePoints.at(i);
        dcp.position = currentPoint;
        currentBinormal = glm::mat3_cast(glm::fquat(dcp.twistingAngle, currentDir.x, currentDir.y, currentDir.z)) * currentBinormal;
        currentDir = glm::mat3_cast(glm::fquat(dcp.bendingAngle, currentBinormal.x, currentBinormal.y, currentBinormal.z)) * currentDir;
        currentPoint += currentDir * segmentLength;

        if(isnan(currentPoint.x))
        {
            std::cout<<"Binormal nan error"<<std::endl;
        }

        points->push_back(glm::vec4(currentPoint, 1.0f));
    }

    createGeometry();
    return currentPoint;
}

void Curve::ComputeFrenetFrames()
{
    if(discretePoints.size() <= 2) return;
    for(int i = 0; i < discretePoints.size(); i++)
    {
        DCurvePoint dcp = discretePoints.at(i);
        if(i == 0)
        {
            glm::vec3 prevPos = startingPoint;
            dcp.ComputeFrenet(prevPos, discretePoints.at(i + 1));
        }
        else if(i == discretePoints.size() - 1)
        {
            glm::fquat r = glm::fquat(discretePoints.at(i).bendingAngle,
                                      discretePoints.at(i - 1).frenetFrame.B.x,
                                      discretePoints.at(i - 1).frenetFrame.B.y,
                                      discretePoints.at(i - 1).frenetFrame.B.z);
            dcp.frenetFrame = discretePoints.at(i - 1).frenetFrame.RotateBy(r);
        }
        else
        {
            DCurvePoint prevPoint = discretePoints.at(i - 1);
            DCurvePoint nextPoint = discretePoints.at(i + 1);
            dcp.ComputeFrenet(prevPoint, nextPoint);
        }
    }

    //FixFrenetFrames();
}

void Curve::FixFrenetFrames()
{
    int firstFrameIndex = -1;
    for(int i = 0; i < discretePoints.size(); i++)
    {
        OrthonormalFrame frame = discretePoints.at(i).frenetFrame;
        if(glm::length(frame.B) > 0)
        {
            firstFrameIndex = i;
            break;
        }
    }

    if(firstFrameIndex == -1)
    {
        glm::vec3 tangent = discretePoints.at(0).frenetFrame.T;
        glm::vec3 normal, binormal;

        if(tangent == glm::vec3(0.0f, 1.0f, 0.0f))
        {
            binormal = glm::vec3(1.0f, 0.0f, 0.0f);
            normal = glm::cross(binormal, tangent);
        }
        else
        {
            binormal = glm::vec3(0.0f, 1.0f, 0.0f);
            binormal = glm::normalize(binormal - glm::dot(binormal, tangent) * tangent);
            normal = glm::cross(binormal, tangent);
        }

        startingBinormal = binormal;
        OrthonormalFrame defaultFrame = OrthonormalFrame(tangent, normal, binormal);

        for(int i = 0; i < discretePoints.size(); i++)
        {
            discretePoints.at(i).frenetFrame = defaultFrame;
        }
    }
    else
    {
        for(int i = 0; i < discretePoints.size(); i++)
        {
            if((glm::length(discretePoints.at(i).frenetFrame.B) - 0.0f) < FLT_EPSILON)
            {
                if(i < firstFrameIndex) FixFrenetForward(i);
                else if(i > firstFrameIndex) FixFrenetBackward(i);
            }
        }
    }
}

void Curve::FixFrenetForward(int start) {}

void Curve::FixFrenetBackward(int start) {}

void Curve::ComputeBishopFrames()
{
    if(discretePoints.size() <= 2) return;
    for(int i = 0; i < discretePoints.size(); i++)
    {
        DCurvePoint dcp = discretePoints.at(i);
        if(i == 0)
        {
            dcp.bishopFrame = dcp.frenetFrame;
        }
        else if(i == discretePoints.size() - 1)
        {
            dcp.bishopFrame = discretePoints.at(i - 1).bishopFrame;
        }
        else
        {
            DCurvePoint prevPoint = discretePoints.at(i - 1);
            DCurvePoint nextPoint = discretePoints.at(i + 1);
            dcp.PropagateBishop(prevPoint, nextPoint, prevPoint.bishopFrame);
        }
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
    glm::vec3 world = rotatedToWorld * helixPoint + cs.startPosition;
    return world;
}
