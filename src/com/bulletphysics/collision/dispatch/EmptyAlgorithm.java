package com.bulletphysics.collision.dispatch;

import com.bulletphysics.collision.broadphase.CollisionAlgorithm;
import com.bulletphysics.collision.broadphase.CollisionAlgorithmConstructionInfo;
import com.bulletphysics.collision.broadphase.DispatcherInfo;
import com.bulletphysics.collision.narrowphase.PersistentManifold;
import java.util.ArrayList;

/**
 * Empty algorithm, used as fallback when no collision algorithm is found for given
 * shape type pair.
 * 
 * @author jezek2
 */
public class EmptyAlgorithm extends CollisionAlgorithm {
	
	private static final EmptyAlgorithm INSTANCE = new EmptyAlgorithm();

	@Override
	public void destroy() {
	}

	@Override
	public void processCollision(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut) {
	}

	@Override
	public double calculateTimeOfImpact(CollisionObject body0, CollisionObject body1, DispatcherInfo dispatchInfo, ManifoldResult resultOut) {
		return 1.0;
	}

	@Override
	public void getAllContactManifolds(ArrayList<PersistentManifold> manifoldArray) {
	}
	
	////////////////////////////////////////////////////////////////////////////
	
	public static class CreateFunc extends CollisionAlgorithmCreateFunc {
		@Override
		public CollisionAlgorithm createCollisionAlgorithm(CollisionAlgorithmConstructionInfo ci, CollisionObject body0, CollisionObject body1) {
			return INSTANCE;
		}

		@Override
		public void releaseCollisionAlgorithm(CollisionAlgorithm algo) {
		}
	};

}
