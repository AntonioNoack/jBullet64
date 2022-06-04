package com.bulletphysics.collision.broadphase;

/**
 * Callback for filtering broadphase collisions.
 * 
 * @see OverlappingPairCache#setOverlapFilterCallback
 * @author jezek2
 */
public abstract class OverlapFilterCallback {

	/**
	 * Checks if given pair of collision objects needs collision.
	 * 
	 * @param proxy0 first object
	 * @param proxy1 second object
	 * @return true when pairs need collision
	 */
	public abstract boolean needBroadphaseCollision(BroadphaseProxy proxy0, BroadphaseProxy proxy1);
	
}
