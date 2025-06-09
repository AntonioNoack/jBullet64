package com.bulletphysics.collision.narrowphase;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * SubsimplexConvexCast implements Gino van den Bergens' paper
 * "Ray Casting against bteral Convex Objects with Application to Continuous Collision Detection"
 * GJK based Ray Cast, optimized version
 * Objects should not start in overlap, otherwise results are not defined.
 *
 * @author jezek2
 */
public class SubSimplexConvexCast implements ConvexCast {

    // Typically the conservative advancement reaches solution in a few iterations, clip it to 32 for degenerate cases.
    // See discussion about this here http://www.bulletphysics.com/phpBB2/viewtopic.php?t=565

    private static final int MAX_ITERATIONS = 32;

    private final SimplexSolverInterface simplexSolver;
    private final ConvexShape convexA;
    private final ConvexShape convexB;

    public SubSimplexConvexCast(ConvexShape shapeA, ConvexShape shapeB, SimplexSolverInterface simplexSolver) {
        this.convexA = shapeA;
        this.convexB = shapeB;
        this.simplexSolver = simplexSolver;
    }

    public boolean calcTimeOfImpact(Transform fromA, Transform toA, Transform fromB, Transform toB, CastResult result) {
        Vector3d tmp = Stack.newVec();

        simplexSolver.reset();

        Vector3d linVelA = Stack.newVec();
        Vector3d linVelB = Stack.newVec();
        linVelA.sub(toA.origin, fromA.origin);
        linVelB.sub(toB.origin, fromB.origin);

        double lambda = 0.0;

        Transform interpolatedTransA = Stack.newTrans(fromA);
        Transform interpolatedTransB = Stack.newTrans(fromB);

        // take relative motion
        Vector3d r = Stack.newVec();
        r.sub(linVelA, linVelB);

        Vector3d v = Stack.newVec();

        tmp.negate(r);
        MatrixUtil.transposeTransform(tmp, tmp, fromA.basis);
        Vector3d supVertexA = convexA.localGetSupportingVertex(tmp, Stack.newVec());
        fromA.transform(supVertexA);

        MatrixUtil.transposeTransform(tmp, r, fromB.basis);
        Vector3d supVertexB = convexB.localGetSupportingVertex(tmp, Stack.newVec());
        fromB.transform(supVertexB);

        v.sub(supVertexA, supVertexB);

        int maxIter = MAX_ITERATIONS;

        Vector3d n = Stack.newVec();
        n.set(0.0, 0.0, 0.0);

        double dist2 = v.lengthSquared();
        double epsilon = 0.0001f;
        Vector3d w = Stack.newVec();
        double VdotR;

        while ((dist2 > epsilon) && (maxIter--) != 0) {
            tmp.negate(v);
            MatrixUtil.transposeTransform(tmp, tmp, interpolatedTransA.basis);
            convexA.localGetSupportingVertex(tmp, supVertexA);
            interpolatedTransA.transform(supVertexA);

            MatrixUtil.transposeTransform(tmp, v, interpolatedTransB.basis);
            convexB.localGetSupportingVertex(tmp, supVertexB);
            interpolatedTransB.transform(supVertexB);

            w.sub(supVertexA, supVertexB);

            double VdotW = v.dot(w);

            if (lambda > 1.0) {
                return false;
            }

            if (VdotW > 0.0) {
                VdotR = v.dot(r);

                if (VdotR >= -(BulletGlobals.FLT_EPSILON * BulletGlobals.FLT_EPSILON)) {
                    return false;
                } else {
                    lambda = lambda - VdotW / VdotR;
                    // interpolate to next lambda
                    //	x = s + lambda * r;
                    VectorUtil.setInterpolate3(interpolatedTransA.origin, fromA.origin, toA.origin, lambda);
                    VectorUtil.setInterpolate3(interpolatedTransB.origin, fromB.origin, toB.origin, lambda);
                    // check next line
                    w.sub(supVertexA, supVertexB);
                    n.set(v);
                }
            }
            simplexSolver.addVertex(w, supVertexA, supVertexB);
            if (simplexSolver.closest(v)) {
                dist2 = v.lengthSquared();
                // todo: check this normal for validity
                //n.set(v);
            } else {
                dist2 = 0.0;
            }
        }

        // don't report a time of impact when moving 'away' from the hitnormal

        result.fraction = lambda;
        if (n.lengthSquared() >= BulletGlobals.SIMD_EPSILON * BulletGlobals.SIMD_EPSILON) {
            result.normal.normalize(n);
        } else {
            result.normal.set(0.0, 0.0, 0.0);
        }

        // don't report time of impact for motion away from the contact normal (or causes minor penetration)
        if (result.normal.dot(r) >= -result.allowedPenetration)
            return false;

        Vector3d hitA = Stack.newVec();
        Vector3d hitB = Stack.newVec();
        simplexSolver.compute_points(hitA, hitB);
        result.hitPoint.set(hitB);
        return true;
    }

}
