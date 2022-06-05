package com.bulletphysics.collision.narrowphase;

import com.bulletphysics.linearmath.IDebugDraw;
import com.bulletphysics.linearmath.Transform;
import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 * ConvexCast is an interface for casting.
 *
 * @author jezek2
 */
public abstract class ConvexCast {

    /**
     * Cast a convex against another convex object.
     */
    public abstract boolean calcTimeOfImpact(Transform fromA, Transform toA, Transform fromB, Transform toB, CastResult result);

    ////////////////////////////////////////////////////////////////////////////

    /**
     * RayResult stores the closest result. Alternatively, add a callback method
     * to decide about closest/all results.
     */
    @SuppressWarnings("unused")
    public static class CastResult {
        public final Transform hitTransformA = new Transform();
        public final Transform hitTransformB = new Transform();

        public final Vector3d normal = new Vector3d();
        public final Vector3d hitPoint = new Vector3d();
        public double fraction = Double.POSITIVE_INFINITY; // input and output
        public double allowedPenetration = 0.0;

        public IDebugDraw debugDrawer;

        public void debugDraw(double fraction) {
        }

        public void drawCoordSystem(Transform trans) {
        }
    }

}
