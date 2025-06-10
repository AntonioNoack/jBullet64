package com.bulletphysics.collision.broadphase;

/**
 * OverlappingPairCallback is an additional optional broadphase user callback
 * for adding/removing overlapping pairs, similar interface to {@link OverlappingPairCache}.
 *
 * @author jezek2
 */
@SuppressWarnings("UnusedReturnValue")
public interface OverlappingPairCallback {

	BroadphasePair addOverlappingPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1);

	Object removeOverlappingPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1, Dispatcher dispatcher);

	void removeOverlappingPairsContainingProxy(BroadphaseProxy proxy0, Dispatcher dispatcher);
	
}
