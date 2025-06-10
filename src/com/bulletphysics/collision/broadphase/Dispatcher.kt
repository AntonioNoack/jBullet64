package com.bulletphysics.collision.broadphase;

import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.util.ObjectArrayList;

/**
 * Dispatcher can be used in combination with broadphase to dispatch
 * calculations for overlapping pairs. For example for pairwise collision detection,
 * calculating contact points stored in {@link PersistentManifold} or user callbacks
 * (game logic).
 * 
 * @author jezek2
 */
public interface Dispatcher {

	default CollisionAlgorithm findAlgorithm(CollisionObject body0, CollisionObject body1) {
		return findAlgorithm(body0, body1, null);
	}

	CollisionAlgorithm findAlgorithm(CollisionObject body0, CollisionObject body1, PersistentManifold sharedManifold);

	PersistentManifold getNewManifold(Object body0, Object body1);

	void releaseManifold(PersistentManifold manifold);

	void clearManifold(PersistentManifold manifold);

	boolean needsCollision(CollisionObject body0, CollisionObject body1);

	boolean needsResponse(CollisionObject body0, CollisionObject body1);

	void dispatchAllCollisionPairs(OverlappingPairCache pairCache, DispatcherInfo dispatchInfo, Dispatcher dispatcher);

	int getNumManifolds();

	PersistentManifold getManifoldByIndexInternal(int index);

	ObjectArrayList<PersistentManifold> getInternalManifoldPointer();

	void freeCollisionAlgorithm(CollisionAlgorithm algo);
	
}
