package com.bulletphysics.collision.narrowphase;

import javax.vecmath.Vector3d;

/**
 * RayResult stores the closest result. Alternatively, add a callback method
 * to decide about closest/all results.
 */
public class CastResult {
    public final Vector3d normal = new Vector3d();
    public final Vector3d hitPoint = new Vector3d();
    public double fraction = 1e30; // input and output
    public double allowedPenetration = 0.0;

    public void debugDraw(double fraction) {
    }

    public void init() {
        fraction = 1e30;
        allowedPenetration = 0.0;
    }
}