package com.bulletphysics.collision.dispatch;

import com.bulletphysics.collision.broadphase.CollisionAlgorithm;
import com.bulletphysics.collision.broadphase.CollisionAlgorithmConstructionInfo;

/**
 * Used by the CollisionDispatcher to register and create instances for CollisionAlgorithm.
 * 
 * @author jezek2
 */
public abstract class CollisionAlgorithmCreateFunc {

	public boolean swapped;
	
	public abstract CollisionAlgorithm createCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1);
	
	// JAVA NOTE: added
	public abstract void releaseCollisionAlgorithm(CollisionAlgorithm algo);
	
}
