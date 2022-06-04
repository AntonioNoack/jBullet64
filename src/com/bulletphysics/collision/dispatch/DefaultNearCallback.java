package com.bulletphysics.collision.dispatch;

import com.bulletphysics.collision.broadphase.BroadphasePair;
import com.bulletphysics.collision.broadphase.DispatchFunc;
import com.bulletphysics.collision.broadphase.DispatcherInfo;

/**
 * Default implementation of {@link NearCallback}.
 * 
 * @author jezek2
 */
public class DefaultNearCallback extends NearCallback {

	private final ManifoldResult contactPointResult = new ManifoldResult();

	public void handleCollision(BroadphasePair collisionPair, CollisionDispatcher dispatcher, DispatcherInfo dispatchInfo) {
		CollisionObject colObj0 = (CollisionObject) collisionPair.pProxy0.clientObject;
		CollisionObject colObj1 = (CollisionObject) collisionPair.pProxy1.clientObject;

		if (dispatcher.needsCollision(colObj0, colObj1)) {
			// dispatcher will keep algorithms persistent in the collision pair
			if (collisionPair.algorithm == null) {
				collisionPair.algorithm = dispatcher.findAlgorithm(colObj0, colObj1);
			}

			if (collisionPair.algorithm != null) {
				//ManifoldResult contactPointResult = new ManifoldResult(colObj0, colObj1);
				contactPointResult.init(colObj0, colObj1);

				if (dispatchInfo.dispatchFunc == DispatchFunc.DISPATCH_DISCRETE) {
					// discrete collision detection query
					collisionPair.algorithm.processCollision(colObj0, colObj1, dispatchInfo, contactPointResult);
				}
				else {
					// continuous collision detection query, time of impact (toi)
					double toi = collisionPair.algorithm.calculateTimeOfImpact(colObj0, colObj1, dispatchInfo, contactPointResult);
					if (dispatchInfo.timeOfImpact > toi) {
						dispatchInfo.timeOfImpact = toi;
					}
				}
			}
		}
	}
	
}
