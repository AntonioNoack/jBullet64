package com.bulletphysics.collision.narrowphase;

import com.bulletphysics.collision.narrowphase.DiscreteCollisionDetectorInterface.ClosestPointInput;
import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import com.bulletphysics.util.ObjectPool;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * GjkConvexCast performs a raycast on a convex object using support mapping.
 *
 * @author jezek2
 */
public class GjkConvexCast implements ConvexCast {

    protected final ObjectPool<ClosestPointInput> pointInputsPool = ObjectPool.get(ClosestPointInput.class);

    private static final int MAX_ITERATIONS = 32;

    private final SimplexSolverInterface simplexSolver = new VoronoiSimplexSolver();
    private ConvexShape convexA;
    private ConvexShape convexB;

    private final GjkPairDetector gjk = new GjkPairDetector();

    public void init(ConvexShape convexA, ConvexShape convexB) {
        this.convexA = convexA;
        this.convexB = convexB;
    }

    public boolean calcTimeOfImpact(Transform fromA, Transform toA, Transform fromB, Transform toB, CastResult result) {
        simplexSolver.reset();

        // compute linear velocity for this interval, to interpolate
        // assume no rotation/angular velocity, assert here?
        Vector3d linVelA = Stack.newVec();
        Vector3d linVelB = Stack.newVec();

        linVelA.sub(toA.origin, fromA.origin);
        linVelB.sub(toB.origin, fromB.origin);

        double radius = 0.001f;
        double lambda = 0.0;
        Vector3d v = Stack.newVec();
        v.set(1.0, 0.0, 0.0);

        Vector3d n = Stack.newVec();
        n.set(0.0, 0.0, 0.0);
        boolean hasResult;
        Vector3d c = Stack.newVec();
        Vector3d r = Stack.newVec();
        r.sub(linVelA, linVelB);

        double lastLambda = lambda;
        //btScalar epsilon = btScalar(0.001);

        int numIter = 0;
        // first solution, using GJK

        Transform identityTrans = Stack.newTrans();
        identityTrans.setIdentity();

        //result.drawCoordSystem(sphereTr);

        PointCollector pointCollector = Stack.newPointCollector();

        gjk.init(convexA, convexB, simplexSolver, null); // penetrationDepthSolver);
        ClosestPointInput input = pointInputsPool.get();
        input.init();
        try {
            // we don't use margins during CCD
            //	gjk.setIgnoreMargin(true);

            input.transformA.set(fromA);
            input.transformB.set(fromB);
            gjk.getClosestPoints(input, pointCollector, null);

            hasResult = pointCollector.hasResult;
            c.set(pointCollector.pointInWorld);

            if (hasResult) {
                double dist;
                dist = pointCollector.distance;
                n.set(pointCollector.normalOnBInWorld);

                // not close enough
                while (dist > radius) {
                    numIter++;
                    if (numIter > MAX_ITERATIONS) {
                        return false; // todo: report a failure
                    }

                    double projectedLinearVelocity = r.dot(n);

                    double dLambda = dist / (projectedLinearVelocity);

                    lambda = lambda - dLambda;

                    if (lambda > 1.0) {
                        return false;
                    }
                    if (lambda < 0.0) {
                        return false;                    // todo: next check with relative epsilon
                    }

                    if (lambda <= lastLambda) {
                        return false;
                    }
                    lastLambda = lambda;

                    // interpolate to next lambda
                    result.debugDraw(lambda);
                    VectorUtil.setInterpolate3(input.transformA.origin, fromA.origin, toA.origin, lambda);
                    VectorUtil.setInterpolate3(input.transformB.origin, fromB.origin, toB.origin, lambda);

                    gjk.getClosestPoints(input, pointCollector, null);
                    if (pointCollector.hasResult) {
                        if (pointCollector.distance < 0.0) {
                            result.fraction = lastLambda;
                            n.set(pointCollector.normalOnBInWorld);
                            result.normal.set(n);
                            result.hitPoint.set(pointCollector.pointInWorld);
                            return true;
                        }
                        c.set(pointCollector.pointInWorld);
                        n.set(pointCollector.normalOnBInWorld);
                        dist = pointCollector.distance;
                    } else {
                        // ??
                        return false;
                    }

                }

                // is n normalized?
                // don't report time of impact for motion away from the contact normal (or causes minor penetration)
                if (n.dot(r) >= -result.allowedPenetration) {
                    return false;
                }
                result.fraction = lambda;
                result.normal.set(n);
                result.hitPoint.set(c);
                return true;
            }

            return false;
        } finally {
            pointInputsPool.release(input);
            Stack.subVec(6);
            Stack.subTrans(1);
            Stack.subPointCollector(1);
        }
    }

}
