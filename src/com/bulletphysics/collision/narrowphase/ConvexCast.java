package com.bulletphysics.collision.narrowphase;

import com.bulletphysics.linearmath.Transform;

import javax.vecmath.Vector3d;

/**
 * ConvexCast is an interface for casting.
 *
 * @author jezek2
 */
public interface ConvexCast {

    /**
     * Cast a convex against another convex object.
     */
    boolean calcTimeOfImpact(Transform fromA, Transform toA, Transform fromB, Transform toB, CastResult result);

}
