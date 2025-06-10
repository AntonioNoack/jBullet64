package com.bulletphysics.collision.narrowphase;

import javax.vecmath.Vector3d;

/**
 * @author jezek2
 */
public class PointCollector implements DiscreteCollisionDetectorInterface.Result {

    public final Vector3d normalOnBInWorld = new Vector3d();
    public final Vector3d pointInWorld = new Vector3d();
    public double distance = 1e308; // negative means penetration

    public boolean hasResult = false;

    public void init() {
        distance = 1e308;
        hasResult = false;
    }

    public void setShapeIdentifiers(int partId0, int index0, int partId1, int index1) {
        // ??
    }

    public void addContactPoint(Vector3d normalOnBInWorld, Vector3d pointInWorld, double depth) {
        if (depth < distance) {
            hasResult = true;
            this.normalOnBInWorld.set(normalOnBInWorld);
            this.pointInWorld.set(pointInWorld);
            // negative means penetration
            distance = depth;
        }
    }

}
