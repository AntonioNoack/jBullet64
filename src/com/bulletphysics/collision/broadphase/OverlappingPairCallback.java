package com.bulletphysics.collision.broadphase;

/**
 * OverlappingPairCallback class is an additional optional broadphase user callback
 * for adding/removing overlapping pairs, similar interface to {@link OverlappingPairCache}.
 *
 * @author jezek2
 */
public abstract class OverlappingPairCallback {

	public abstract BroadphasePair addOverlappingPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1);

	public abstract Object removeOverlappingPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1, Dispatcher dispatcher);

	public abstract void removeOverlappingPairsContainingProxy(BroadphaseProxy proxy0, Dispatcher dispatcher);
	
}
