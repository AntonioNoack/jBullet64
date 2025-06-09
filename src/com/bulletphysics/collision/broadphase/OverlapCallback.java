package com.bulletphysics.collision.broadphase;

/**
 * OverlapCallback is used when processing all overlapping pairs in broadphase.
 *
 * @author jezek2
 * @see OverlappingPairCache#processAllOverlappingPairs
 */
public abstract class OverlapCallback {

    /**
     * return true for deletion of the pair
     */
    public abstract boolean processOverlap(BroadphasePair pair);

}
