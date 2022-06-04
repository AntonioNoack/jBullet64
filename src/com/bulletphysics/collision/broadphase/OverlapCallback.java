package com.bulletphysics.collision.broadphase;

/**
 * OverlapCallback is used when processing all overlapping pairs in broadphase.
 * 
 * @see OverlappingPairCache#processAllOverlappingPairs
 * @author jezek2
 */
public abstract class OverlapCallback {

	//return true for deletion of the pair
	public abstract boolean processOverlap(BroadphasePair pair);
	
}
