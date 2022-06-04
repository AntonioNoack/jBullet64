package com.bulletphysics.collision.broadphase;

import javax.vecmath.Vector3d;

/**
 * BroadphaseInterface for AABB overlapping object pairs.
 * 
 * @author jezek2
 */
public abstract class BroadphaseInterface {

	public abstract BroadphaseProxy createProxy(Vector3d aabbMin, Vector3d aabbMax, BroadphaseNativeType shapeType, Object userPtr, short collisionFilterGroup, short collisionFilterMask, Dispatcher dispatcher, Object multiSapProxy);

	public abstract void destroyProxy(BroadphaseProxy proxy, Dispatcher dispatcher);

	public abstract void setAabb(BroadphaseProxy proxy, Vector3d aabbMin, Vector3d aabbMax, Dispatcher dispatcher);

	// calculateOverlappingPairs is optional: incremental algorithms (sweep and prune) might do it during the set aabb
	public abstract void calculateOverlappingPairs(Dispatcher dispatcher);

	public abstract OverlappingPairCache getOverlappingPairCache();
	
	// getAabb returns the axis aligned bounding box in the 'global' coordinate frame
	// will add some transform later
	public abstract void getBroadphaseAabb(Vector3d aabbMin, Vector3d aabbMax);
	
}
