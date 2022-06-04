package com.bulletphysics.collision.dispatch;

import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.BroadphasePair;
import com.bulletphysics.collision.broadphase.BroadphaseProxy;
import com.bulletphysics.collision.broadphase.Dispatcher;
import com.bulletphysics.collision.broadphase.OverlappingPairCallback;

/**
 * GhostPairCallback interfaces and forwards adding and removal of overlapping
 * pairs from the {@link BroadphaseInterface} to {@link GhostObject}.
 *
 * @author tomrbryn
 */
public class GhostPairCallback extends OverlappingPairCallback {

	public BroadphasePair addOverlappingPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1) {
		CollisionObject colObj0 = (CollisionObject)proxy0.clientObject;
		CollisionObject colObj1 = (CollisionObject)proxy1.clientObject;
		GhostObject ghost0 = GhostObject.upcast(colObj0);
		GhostObject ghost1 = GhostObject.upcast(colObj1);

		if (ghost0 != null) {
			ghost0.addOverlappingObjectInternal(proxy1, proxy0);
		}
		if (ghost1 != null) {
			ghost1.addOverlappingObjectInternal(proxy0, proxy1);
		}
		return null;
	}

	public Object removeOverlappingPair(BroadphaseProxy proxy0, BroadphaseProxy proxy1, Dispatcher dispatcher) {
		CollisionObject colObj0 = (CollisionObject)proxy0.clientObject;
		CollisionObject colObj1 = (CollisionObject)proxy1.clientObject;
		GhostObject ghost0 = GhostObject.upcast(colObj0);
		GhostObject ghost1 = GhostObject.upcast(colObj1);
		
		if (ghost0 != null) {
			ghost0.removeOverlappingObjectInternal(proxy1, dispatcher, proxy0);
		}
		if (ghost1 != null) {
			ghost1.removeOverlappingObjectInternal(proxy0, dispatcher, proxy1);
		}
		return null;
	}

	public void removeOverlappingPairsContainingProxy(BroadphaseProxy proxy0, Dispatcher dispatcher) {
		assert (false);

		// need to keep track of all ghost objects and call them here
		// hashPairCache.removeOverlappingPairsContainingProxy(proxy0, dispatcher);
	}
	
}
