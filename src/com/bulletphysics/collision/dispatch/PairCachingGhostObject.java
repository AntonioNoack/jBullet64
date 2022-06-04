package com.bulletphysics.collision.dispatch;

import com.bulletphysics.collision.broadphase.BroadphaseProxy;
import com.bulletphysics.collision.broadphase.Dispatcher;
import com.bulletphysics.collision.broadphase.HashedOverlappingPairCache;

/**
 *
 * @author tomrbryn
 */
public class PairCachingGhostObject extends GhostObject {
	
	HashedOverlappingPairCache hashPairCache = new HashedOverlappingPairCache();

	/**
	 * This method is mainly for expert/internal use only.
	 */
	@Override
	public void addOverlappingObjectInternal(BroadphaseProxy otherProxy, BroadphaseProxy thisProxy) {
		BroadphaseProxy actualThisProxy = thisProxy != null? thisProxy : getBroadphaseHandle();
		assert(actualThisProxy != null);

		CollisionObject otherObject = (CollisionObject) otherProxy.clientObject;
		assert (otherObject != null);

		// if this linearSearch becomes too slow (too many overlapping objects) we should add a more appropriate data structure
		int index = overlappingObjects.indexOf(otherObject);
		if (index == -1) {
			overlappingObjects.add(otherObject);
			hashPairCache.addOverlappingPair(actualThisProxy, otherProxy);
		}
	}

	@Override
	public void removeOverlappingObjectInternal(BroadphaseProxy otherProxy, Dispatcher dispatcher, BroadphaseProxy thisProxy1) {
		CollisionObject otherObject = (CollisionObject)otherProxy.clientObject;
		BroadphaseProxy actualThisProxy = thisProxy1 != null? thisProxy1 : getBroadphaseHandle();
		assert(actualThisProxy != null);

		assert (otherObject != null);
		int index = overlappingObjects.indexOf(otherObject);
		if (index != -1) {
			overlappingObjects.set(index, overlappingObjects.get(overlappingObjects.size()-1));
			overlappingObjects.remove(overlappingObjects.size()-1);
			hashPairCache.removeOverlappingPair(actualThisProxy, otherProxy, dispatcher);
		}
	}

	public HashedOverlappingPairCache getOverlappingPairCache() {
		return hashPairCache;
	}
	
}
