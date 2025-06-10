package com.bulletphysics.collision.dispatch;

import com.bulletphysics.collision.broadphase.BroadphasePair;
import com.bulletphysics.collision.broadphase.DispatcherInfo;

/**
 * Callback for overriding collision filtering and more fine-grained control over
 * collision detection.
 * 
 * @see CollisionDispatcher#setNearCallback
 * @see CollisionDispatcher#getNearCallback
 * @author jezek2
 */
public interface NearCallback {

	void handleCollision(BroadphasePair collisionPair, CollisionDispatcher dispatcher, DispatcherInfo dispatchInfo);
	
}
