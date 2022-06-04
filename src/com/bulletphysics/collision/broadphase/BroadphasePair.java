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

	public BroadphaseProxy pProxy0;
	public BroadphaseProxy pProxy1;
	public CollisionAlgorithm algorithm;
	public Object userInfo;

	public BroadphasePair() {
	}

	public BroadphasePair(BroadphaseProxy pProxy0, BroadphaseProxy pProxy1) {
		this.pProxy0 = pProxy0;
		this.pProxy1 = pProxy1;
		this.algorithm = null;
		this.userInfo = null;
	}
	
	public void set(BroadphasePair p) {
		pProxy0 = p.pProxy0;
		pProxy1 = p.pProxy1;
		algorithm = p.algorithm;
		userInfo = p.userInfo;
	}
	
	public boolean equals(BroadphasePair p) {
		return pProxy0 == p.pProxy0 && pProxy1 == p.pProxy1;
	}
	
	public static final Comparator<BroadphasePair> broadphasePairSortPredicate = new Comparator<BroadphasePair>() {
		public int compare(BroadphasePair a, BroadphasePair b) {
			// JAVA TODO:
			boolean result = a.pProxy0.getUid() > b.pProxy0.getUid() ||
					(a.pProxy0.getUid() == b.pProxy0.getUid() && a.pProxy1.getUid() > b.pProxy1.getUid()) ||
					(a.pProxy0.getUid() == b.pProxy0.getUid() && a.pProxy1.getUid() == b.pProxy1.getUid() /*&& a.algorithm > b.m_algorithm*/);
			return result? -1 : 1;
		}
	};
	
}
