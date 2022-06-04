package com.bulletphysics.collision.broadphase;

import com.bulletphysics.collision.dispatch.CollisionAlgorithmCreateFunc;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.ManifoldResult;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import com.bulletphysics.util.ObjectArrayList;

/**
 * Collision algorithm for handling narrowphase or midphase collision detection
 * between two collision object types.
 * 
 * @author jezek2
 */
public abstract class CollisionAlgorithm {

	//protected final BulletStack stack = BulletStack.get();
	
	// JAVA NOTE: added
	private CollisionAlgorithmCreateFunc createFunc;
	
	protected Dispatcher dispatcher;

	public void init() {
	}

	public void init(CollisionAlgorithmConstructionInfo ci) {
		dispatcher = ci.dispatcher1;
	}
	
	public abstract void destroy();

	public abstract void processCollision(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut);

	public abstract double calculateTimeOfImpact(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut);
	
	public abstract void getAllContactManifolds(ObjectArrayList<PersistentManifold> manifoldArray);
	
	public final void internalSetCreateFunc(CollisionAlgorithmCreateFunc func) {
		createFunc = func;
	}

	public final CollisionAlgorithmCreateFunc internalGetCreateFunc() {
		return createFunc;
	}
	
}
