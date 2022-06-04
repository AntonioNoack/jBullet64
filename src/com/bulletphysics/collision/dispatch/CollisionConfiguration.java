package com.bulletphysics.collision.dispatch;

import com.bulletphysics.collision.broadphase.BroadphaseNativeType;

/**
 * CollisionConfiguration allows to configure Bullet default collision algorithms.
 * 
 * @author jezek2
 */
public abstract class CollisionConfiguration {

	public abstract CollisionAlgorithmCreateFunc getCollisionAlgorithmCreateFunc(BroadphaseNativeType proxyType0, BroadphaseNativeType proxyType1);
	
}
