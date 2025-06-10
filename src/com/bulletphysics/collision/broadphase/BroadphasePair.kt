package com.bulletphysics.collision.broadphase;

import java.util.Comparator;

/**
 * BroadphasePair class contains a pair of AABB-overlapping objects.
 * {@link Dispatcher} can search a {@link CollisionAlgorithm} that performs
 * exact/narrowphase collision detection on the actual collision shapes.
 *
 * @author jezek2
 */
public class BroadphasePair {

    public BroadphaseProxy proxy0;
    public BroadphaseProxy proxy1;
    public CollisionAlgorithm algorithm;
    public Object userInfo;

    public BroadphasePair() {
    }

    public BroadphasePair(BroadphaseProxy proxy0, BroadphaseProxy proxy1) {
        this.proxy0 = proxy0;
        this.proxy1 = proxy1;
        this.algorithm = null;
        this.userInfo = null;
    }

    public void set(BroadphasePair p) {
        proxy0 = p.proxy0;
        proxy1 = p.proxy1;
        algorithm = p.algorithm;
        userInfo = p.userInfo;
    }

    public boolean equals(BroadphasePair p) {
        return proxy0 == p.proxy0 && proxy1 == p.proxy1;
    }

    public static final Comparator<BroadphasePair> broadphasePairSortPredicate =
            Comparator.<BroadphasePair>comparingInt(it -> it.proxy0.getUid())
                    .thenComparing(it -> it.proxy1.getUid())
                    .reversed();

}
