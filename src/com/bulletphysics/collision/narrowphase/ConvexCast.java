package com.bulletphysics.collision.narrowphase;

import com.bulletphysics.linearmath.Transform;

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
    public static class CastResult {
        public final Vector3d normal = new Vector3d();
        public final Vector3d hitPoint = new Vector3d();
        public double fraction = 1e30; // input and output
        public double allowedPenetration = 0.0;

        public void debugDraw(double fraction) {
        }
    }

}
