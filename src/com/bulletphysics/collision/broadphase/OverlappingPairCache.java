package com.bulletphysics.collision.broadphase;

import com.bulletphysics.util.ObjectArrayList;

/**
 * OverlappingPairCache provides an interface for overlapping pair management (add,
 * remove, storage), used by the {@link BroadphaseInterface} broadphases.
 *
 * @author jezek2
 */
public interface OverlappingPairCache extends OverlappingPairCallback {

    ObjectArrayList<BroadphasePair> getOverlappingPairArray();

    void cleanOverlappingPair(BroadphasePair pair, Dispatcher dispatcher);

    int getNumOverlappingPairs();

    void cleanProxyFromPairs(BroadphaseProxy proxy, Dispatcher dispatcher);

    void setOverlapFilterCallback(OverlapFilterCallback overlapFilterCallback);

    void processAllOverlappingPairs(OverlapCallback callback, Dispatcher dispatcher);

    BroadphasePair findPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1);

    boolean hasDeferredRemoval();

    void setInternalGhostPairCallback(OverlappingPairCallback ghostPairCallback);

}
