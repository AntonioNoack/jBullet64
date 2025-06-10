package com.bulletphysics.collision.broadphase;

/**
 * OverlapCallback is used when processing all overlapping pairs in broadphase.
 *
 * @author jezek2
 * @see OverlappingPairCache#processAllOverlappingPairs
 */
public interface OverlapCallback {

    /**
     * return true for deletion of the pair
     */
    boolean processOverlap(BroadphasePair pair);

}
