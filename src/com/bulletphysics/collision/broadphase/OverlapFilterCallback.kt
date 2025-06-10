package com.bulletphysics.collision.broadphase;

/**
 * Callback for filtering broadphase collisions.
 * 
 * @see OverlappingPairCache#setOverlapFilterCallback
 * @author jezek2
 */
public interface OverlapFilterCallback {

	/**
	 * Checks if given a pair of collision objects needs collision.
	 * 
	 * @param proxy0 first object
	 * @param proxy1 second object
	 * @return true when pairs need collision
	 */
	boolean needBroadphaseCollision(BroadphaseProxy proxy0, BroadphaseProxy proxy1);
	
}
