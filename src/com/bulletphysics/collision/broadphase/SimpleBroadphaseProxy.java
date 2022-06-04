package com.bulletphysics.collision.broadphase;

import cz.advel.stack.Stack;

import javax.vecmath.Vector3d;

/**
 *
 * @author jezek2
 */
class SimpleBroadphaseProxy extends BroadphaseProxy {

	protected final Vector3d min = new Vector3d();
	protected final Vector3d max = new Vector3d();
	
	public SimpleBroadphaseProxy() {
	}

	public SimpleBroadphaseProxy(Vector3d minpt, Vector3d maxpt, BroadphaseNativeType shapeType, Object userPtr, short collisionFilterGroup, short collisionFilterMask, Object multiSapProxy) {
		super(userPtr, collisionFilterGroup, collisionFilterMask, multiSapProxy);
		this.min.set(minpt);
		this.max.set(maxpt);
	}
	
}
