using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;

[RequireComponent(typeof(DroneController))]
public class AgentRVO : MonoBehaviour
{

    IList<KeyValuePair<float, AgentRVO>> agentNeighbors_ = new List<KeyValuePair<float, AgentRVO>>();
    //IList<KeyValuePair<float, Obstacle>> obstacleNeighbors_ = new List<KeyValuePair<float, Obstacle>>();
    IList<VLine> orcaLines_ = new List<VLine>();
    Vector3 position_;
    Vector3 prefVelocity_;
    Vector3 velocity_;
    int id_ = 0;
    int maxNeighbors_ = 40;
    public int numNeighbors_ = 0;
    float maxSpeed_ = 0.0f;
    public float minumneighborDist_ = float.MaxValue;
    float radius_ = 1.0f;
    float timeHorizon_ = 1.0f;
    float timeHorizonObst_ = 1.0f;

    private Vector3 newVelocity_;

    public AgentRVO() { }

    public AgentRVO(float radius, float maxSpeed, Vector3 velocity, Vector3 position)
    {
        this.maxSpeed_ = maxSpeed;
        this.radius_ = radius;
        this.velocity_ = velocity;
        this.position_ = position;

    }
    public void setAgent(float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, Vector3 velocity, Vector3 position)
    {
        this.maxSpeed_ = maxSpeed;
        this.radius_ = radius;
        this.timeHorizon_ = timeHorizon;
        this.timeHorizonObst_ = timeHorizonObst;
        this.velocity_ = velocity;
        this.position_ = position;
    }

    public void setprefVelocity_(Vector3 prefVelocity_)
    {
        this.prefVelocity_ = prefVelocity_;
    }

    public Vector3 updateVelocity()
    {
        computeNewVelocity();

        return newVelocity_;
    }
    void computeNewVelocity()
    {
        orcaLines_.Clear();

        int numObstLines = orcaLines_.Count;

        float invTimeHorizon = 1.0f;

        /* Create agent ORCA lines. */
        for (int i = 0; i < agentNeighbors_.Count; ++i)
        {
            AgentRVO other = agentNeighbors_[i].Value;

            Vector3 relativePosition = other.position_ - position_;
            Vector3 relativeVelocity = velocity_ - other.velocity_;
            float distSq = MathTool.absSq(relativePosition);
            float combinedRadius = radius_ + other.radius_;
            float combinedRadiusSq = MathTool.sqr(combinedRadius);

            VLine line;
            Vector3 u;

            if (distSq > combinedRadiusSq)
            {
                /* No collision. */
                Vector3 w = relativeVelocity - invTimeHorizon * relativePosition;

                /* Vector from cutoff center to relative velocity. */
                float wLengthSq = MathTool.absSq(w);
                float dotProduct1 = MathTool.multiply(w, relativePosition);

                if (dotProduct1 < 0.0f && MathTool.sqr(dotProduct1) > combinedRadiusSq * wLengthSq)
                {
                    /* Project on cut-off circle. */
                    float wLength = MathTool.sqrt(wLengthSq);
                    Vector3 unitW = w / wLength;

                    line.direction = new Vector3(unitW.z,0, -unitW.x);
                    u = (combinedRadius * invTimeHorizon - wLength) * unitW;
                }
                else
                {
                    /* Project on legs. */
                    float leg = MathTool.sqrt(distSq - combinedRadiusSq);

                    if (MathTool.det(relativePosition, w) > 0.0f)
                    {
                        /* Project on left leg. */
                        line.direction = new Vector3(relativePosition.x * leg - relativePosition.z * combinedRadius, 0,relativePosition.x * combinedRadius + relativePosition.z * leg) / distSq;
                    }
                    else
                    {
                        /* Project on right leg. */
                        line.direction = -new Vector3(relativePosition.x * leg + relativePosition.z * combinedRadius, 0, -relativePosition.x * combinedRadius + relativePosition.z * leg) / distSq;
                    }

                    float dotProduct2 = MathTool.multiply(relativeVelocity, line.direction) ;
                    u = dotProduct2 * line.direction - relativeVelocity;
                }
            }
            else
            {
                /* Collision. Project on cut-off circle of time timeStep. */
                float invTimeStep = 1.0f / 0.1f;

                /* Vector from cutoff center to relative velocity. */
                Vector3 w = relativeVelocity - invTimeStep * relativePosition;

                float wLength = MathTool.abs(w);
                Vector3 unitW = w / wLength;

                line.direction = new Vector3(unitW.z, 0, -unitW.x);
                u = (combinedRadius * invTimeStep - wLength) * unitW;
            }

            line.point = velocity_ + 0.5f * u;
            orcaLines_.Add(line);
        }

        int lineFail = linearProgram2(orcaLines_, maxSpeed_, prefVelocity_, false, ref newVelocity_);

        if (lineFail < orcaLines_.Count)
        {
            linearProgram3(orcaLines_, numObstLines, lineFail, maxSpeed_, ref newVelocity_);
        }
    }

    private bool linearProgram1(IList<VLine> lines, int lineNo, float radius, Vector3 optVelocity, bool directionOpt, ref Vector3 result)
    {
        float dotProduct = MathTool.multiply( lines[lineNo].point, lines[lineNo].direction); 
        float discriminant = MathTool.sqr(dotProduct) + MathTool.sqr(radius) - MathTool.absSq(lines[lineNo].point);

        if (discriminant < 0.0f)
        {
            /* Max speed circle fully invalidates line lineNo. */
            return false;
        }

        float sqrtDiscriminant = MathTool.sqrt(discriminant);
        float tLeft = -dotProduct - sqrtDiscriminant;
        float tRight = -dotProduct + sqrtDiscriminant;

        for (int i = 0; i < lineNo; ++i)
        {
            float denominator = MathTool.det(lines[lineNo].direction, lines[i].direction);
            float numerator = MathTool.det(lines[i].direction, lines[lineNo].point - lines[i].point);

            if (MathTool.fabs(denominator) <= MathTool.RVO_EPSILON)
            {
                /* Lines lineNo and i are (almost) parallel. */
                if (numerator < 0.0f)
                {
                    return false;
                }

                continue;
            }

            float t = numerator / denominator;

            if (denominator >= 0.0f)
            {
                /* Line i bounds line lineNo on the right. */
                tRight = Math.Min(tRight, t);
            }
            else
            {
                /* Line i bounds line lineNo on the left. */
                tLeft = Math.Max(tLeft, t);
            }

            if (tLeft > tRight)
            {
                return false;
            }
        }

        if (directionOpt)
        {
            /* Optimize direction. */
            if (MathTool.multiply(optVelocity, lines[lineNo].direction)  > 0.0f)
            {
                /* Take right extreme. */
                result = lines[lineNo].point + tRight * lines[lineNo].direction;
            }
            else
            {
                /* Take left extreme. */
                result = lines[lineNo].point + tLeft * lines[lineNo].direction;
            }
        }
        else
        {
            /* Optimize closest point. */
            float t = MathTool.multiply(lines[lineNo].direction, (optVelocity - lines[lineNo].point));  

            if (t < tLeft)
            {
                result = lines[lineNo].point + tLeft * lines[lineNo].direction;
            }
            else if (t > tRight)
            {
                result = lines[lineNo].point + tRight * lines[lineNo].direction;
            }
            else
            {
                result = lines[lineNo].point + t * lines[lineNo].direction;
            }
        }

        return true;
    }

    private int linearProgram2(IList<VLine> lines, float radius, Vector3 optVelocity, bool directionOpt, ref Vector3 result)
    {
        if (directionOpt)
        {
            /*
             * Optimize direction. Note that the optimization velocity is of
             * unit length in this case.
             */
            result = optVelocity * radius;
        }
        else if (MathTool.absSq(optVelocity) > MathTool.sqr(radius))
        {
            /* Optimize closest point and outside circle. */
            result = MathTool.normalize(optVelocity) * radius;
        }
        else
        {
            /* Optimize closest point and inside circle. */
            result = optVelocity;
        }

        for (int i = 0; i < lines.Count; ++i)
        {
            if (MathTool.det(lines[i].direction, lines[i].point - result) > 0.0f)
            {
                /* Result does not satisfy constraint i. Compute new optimal result. */
                Vector3 tempResult = result;
                if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, ref result))
                {
                    result = tempResult;

                    return i;
                }
            }
        }

        return lines.Count;
    }

    private void linearProgram3(IList<VLine> lines, int numObstLines, int beginLine, float radius, ref Vector3 result)
    {
        float distance = 0.0f;

        for (int i = beginLine; i < lines.Count; ++i)
        {
            if (MathTool.det(lines[i].direction, lines[i].point - result) > distance)
            {
                /* Result does not satisfy constraint of line i. */
                IList<VLine> projLines = new List<VLine>();
                for (int ii = 0; ii < numObstLines; ++ii)
                {
                    projLines.Add(lines[ii]);
                }

                for (int j = numObstLines; j < i; ++j)
                {
                    VLine line;

                    float determinant = MathTool.det(lines[i].direction, lines[j].direction);

                    if (MathTool.fabs(determinant) <= MathTool.RVO_EPSILON)
                    {
                        /* Line i and line j are parallel. */
                        if (MathTool.multiply(lines[i].direction,lines[j].direction)  > 0.0f)
                        {
                            /* Line i and line j point in the same direction. */
                            continue;
                        }
                        else
                        {
                            /* Line i and line j point in opposite direction. */
                            line.point = 0.5f * (lines[i].point + lines[j].point);
                        }
                    }
                    else
                    {
                        line.point = lines[i].point + (MathTool.det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
                    }

                    line.direction = MathTool.normalize(lines[j].direction - lines[i].direction);
                    projLines.Add(line);
                }

                Vector3 tempResult = result;
                if (linearProgram2(projLines, radius, new Vector3(-lines[i].direction.z,0, lines[i].direction.x), true, ref result) < projLines.Count)
                {
                    /*
                     * This should in principle not happen. The result is by
                     * definition already in the feasible region of this
                     * linear program. If it fails, it is due to small
                     * floating point error, and the current result is kept.
                     */
                    result = tempResult;
                }

                distance = MathTool.det(lines[i].direction, lines[i].point - result);
            }
        }
    }

    public void addNeighbor(AgentRVO neighbor)
    {
        float dist = (neighbor.position_ - this.position_).magnitude;
        KeyValuePair<float, AgentRVO> newone = new KeyValuePair<float, AgentRVO>(dist, neighbor);
        agentNeighbors_.Add(newone);
        numNeighbors_ = agentNeighbors_.Count;
        if(dist < minumneighborDist_) { minumneighborDist_ = dist; }
    }
    public void clearNeighbor()
    {
        agentNeighbors_.Clear();
        numNeighbors_ = 0;
        minumneighborDist_ = 10000.0f;
    }
}

public struct VLine
{
    public Vector3 direction;
    public Vector3 point;
}


public class MathTool
{
    public const float RVO_EPSILON = 0.00001f;

    public static float abs(Vector3 vector)
    {
        return vector.magnitude;
    }

    public static float absSq(Vector3 vector)
    {
        return vector.sqrMagnitude;
    }

    public static Vector3 normalize(Vector3 vector)
    {
        return vector.normalized;
    }

    public static float det(Vector3 vector1, Vector3 vector2)
    {
        return vector1.x * vector2.z - vector1.z * vector2.x;
    }
    public static float multiply(Vector3 vector1, Vector3 vector2)
    {
        return vector1.x * vector2.x + vector1.z * vector2.z;
    }

    public static float distSqPointLineSegment(Vector3 vector1, Vector3 vector2, Vector3 vector3)
    {
        float v2v3Distance = Vector3.Distance(vector2, vector3);
        //the dot
        float dotResult = Vector3.Dot(vector2- vector1, vector2- vector3);
        //the theta
        float seitaRad = Mathf.Acos(dotResult / (abs(vector2 - vector1) * v2v3Distance));
        //the distance 
        float distance = v2v3Distance * Mathf.Sin(seitaRad);

        return sqr(distance);
    }
    public static float fabs(float scalar)
    {
        return Math.Abs(scalar);
    }

    public static float leftOf(Vector3 a, Vector3 b, Vector3 c)
    {
        return det(a - c, b - a);
    }

    public static float sqr(float scalar)
    {
        return scalar * scalar;
    }

    public static float sqrt(float scalar)
    {
        return (float)Math.Sqrt(scalar);
    }
}