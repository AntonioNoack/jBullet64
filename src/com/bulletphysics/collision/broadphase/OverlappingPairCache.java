package com.bulletphysics.collision.broadphase;

import com.bulletphysics.util.ObjectArrayList;

/**
 * OverlappingPairCache provides an interface for overlapping pair management (add,
 * remove, storage), used by the {@link BroadphaseInterface} broadphases.
 *
 * @author jezek2
 */
public abstract class OverlappingPairCache extends OverlappingPairCallback {

	public abstract ObjectArrayList<BroadphasePair> getOverlappingPairArray();
	
	public abstract void cleanOverlappingPair(BroadphasePair pair, Dispatcher dispatcher);
	
	public abstract int getNumOverlappingPairs();
	
	public abstract void cleanProxyFromPairs(BroadphaseProxy proxy, Dispatcher dispatcher);
	
	public abstract void setOverlapFilterCallback(OverlapFilterCallback overlapFilterCallback);
	
	public abstract void processAllOverlappingPairs(OverlapCallback callback, Dispatcher dispatcher);
	
	public abstract BroadphasePair findPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1);
	
	public abstract boolean hasDeferredRemoval();

	public abstract void setInternalGhostPairCallback(OverlappingPairCallback ghostPairCallback);
	
}
