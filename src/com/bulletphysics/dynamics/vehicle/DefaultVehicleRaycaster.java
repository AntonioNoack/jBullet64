package com.bulletphysics.dynamics.vehicle;

import com.bulletphysics.collision.dispatch.CollisionWorld.ClosestRayResultCallback;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import javax.vecmath.Vector3d;

/**
 * Default implementation of {@link VehicleRaycaster}.
 * 
 * @author jezek2
 */
@SuppressWarnings("unused")
public class DefaultVehicleRaycaster extends VehicleRaycaster {

	protected DynamicsWorld dynamicsWorld;

	public DefaultVehicleRaycaster(DynamicsWorld world) {
		this.dynamicsWorld = world;
	}

	public Object castRay(Vector3d from, Vector3d to, VehicleRaycasterResult result) {
		ClosestRayResultCallback rayCallback = new ClosestRayResultCallback(from, to);
		dynamicsWorld.rayTest(from, to, rayCallback);

		if (rayCallback.hasHit()) {
			RigidBody body = RigidBody.upcast(rayCallback.collisionObject);
			if (body != null && body.hasContactResponse()) {
				result.hitPointInWorld.set(rayCallback.hitPointWorld);
				result.hitNormalInWorld.set(rayCallback.hitNormalWorld);
				result.hitNormalInWorld.normalize();
				result.distFraction = rayCallback.closestHitFraction;
				return body;
			}
		}
		return null;
	}
}
