package com.bulletphysics.collision.broadphase;

import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.dynamics.RigidBody;

/**
 * BroadphaseProxy is the main class that can be used with the Bullet broadphases.
 * It stores collision shape type information, collision filter information and
 * a client object, typically a {@link CollisionObject} or {@link RigidBody}.
 * 
 * @author jezek2
 */
public class BroadphaseProxy {

	/**
	 * Usually the client CollisionObject or Rigidbody class
	 * */
	public Object clientObject;
	
	// TODO: mask
	public short collisionFilterGroup;
	public short collisionFilterMask;
	
	public Object multiSapParentProxy;

	/**
	 * uniqueId is introduced for paircache. could get rid of this, by calculating the address offset etc.
	 * */
	public int uniqueId;

	public BroadphaseProxy() {
	}
	
	public BroadphaseProxy(Object userPtr, short collisionFilterGroup, short collisionFilterMask) {
		this(userPtr, collisionFilterGroup, collisionFilterMask, null);
	}
	
	public BroadphaseProxy(Object userPtr, short collisionFilterGroup, short collisionFilterMask, Object multiSapParentProxy) {
		this.clientObject = userPtr;
		this.collisionFilterGroup = collisionFilterGroup;
		this.collisionFilterMask = collisionFilterMask;
		this.multiSapParentProxy = multiSapParentProxy;
	}

	public int getUid() {
		return uniqueId;
	}
	
}
