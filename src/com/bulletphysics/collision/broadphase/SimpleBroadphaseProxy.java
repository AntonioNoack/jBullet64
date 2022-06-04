package com.bulletphysics.collision.broadphase;

import javax.vecmath.Vector3d;

/**
 * @author jezek2
 */
class SimpleBroadphaseProxy extends BroadphaseProxy {

    protected final Vector3d min = new Vector3d();
    protected final Vector3d max = new Vector3d();

    public SimpleBroadphaseProxy() {
    }

    public SimpleBroadphaseProxy(
            Vector3d minPoint, Vector3d maxPoint,
            Object userPtr,
            short collisionFilterGroup,
            short collisionFilterMask,
            Object multiSapProxy) {
        super(userPtr, collisionFilterGroup, collisionFilterMask, multiSapProxy);
        this.min.set(minPoint);
        this.max.set(maxPoint);
    }

}
