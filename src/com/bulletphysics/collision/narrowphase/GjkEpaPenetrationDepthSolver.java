package com.bulletphysics.collision.narrowphase;

import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.linearmath.IDebugDraw;
import com.bulletphysics.linearmath.Transform;

import javax.vecmath.Vector3d;

/**
 * GjkEpaPenetrationDepthSolver uses the Expanding Polytope Algorithm to calculate
 * the penetration depth between two convex shapes.
 *
 * @author jezek2
 */
public class GjkEpaPenetrationDepthSolver implements ConvexPenetrationDepthSolver {

    private final GjkEpaSolver gjkEpaSolver = new GjkEpaSolver();

    public boolean calculatePenetrationDepth(
            SimplexSolverInterface simplexSolver,
            ConvexShape pConvexA, ConvexShape pConvexB,
            Transform transformA, Transform transformB,
            Vector3d v, Vector3d wWitnessOnA, Vector3d wWitnessOnB,
            IDebugDraw debugDraw) {
        double radialMargin = 0.0;

        // JAVA NOTE: 2.70b1: update when GjkEpaSolver2 is ported

        GjkEpaSolver.Results results = new GjkEpaSolver.Results();
        if (gjkEpaSolver.collide(pConvexA, transformA,
                pConvexB, transformB,
                radialMargin, results)) {
            wWitnessOnA.set(results.witnesses[0]);
            wWitnessOnB.set(results.witnesses[1]);
            return true;
        }

        return false;
    }

}
