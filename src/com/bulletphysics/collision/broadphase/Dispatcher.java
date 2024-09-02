package com.bulletphysics.collision.broadphase;

import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.util.ObjectArrayList;

/**
 * Dispatcher abstract class can be used in combination with broadphase to dispatch
 * calculations for overlapping pairs. For example for pairwise collision detection,
 * calculating contact points stored in {@link PersistentManifold} or user callbacks
 * (game logic).
 * 
 * @author jezek2
 */
public abstract class Dispatcher {

	public final CollisionAlgorithm findAlgorithm(CollisionObject body0, CollisionObject body1) {
		return findAlgorithm(body0, body1, null);
	}

	public abstract CollisionAlgorithm findAlgorithm(CollisionObject body0, CollisionObject body1, PersistentManifold sharedManifold);

	public abstract PersistentManifold getNewManifold(Object body0, Object body1);

	public abstract void releaseManifold(PersistentManifold manifold);

	public abstract void clearManifold(PersistentManifold manifold);

	public abstract boolean needsCollision(CollisionObject body0, CollisionObject body1);

	public abstract boolean needsResponse(CollisionObject body0, CollisionObject body1);

	public abstract void dispatchAllCollisionPairs(OverlappingPairCache pairCache, DispatcherInfo dispatchInfo, Dispatcher dispatcher);

	public abstract int getNumManifolds();

	public abstract PersistentManifold getManifoldByIndexInternal(int index);

	public abstract ObjectArrayList<PersistentManifold> getInternalManifoldPointer();

	public abstract void freeCollisionAlgorithm(CollisionAlgorithm algo);
	
}
