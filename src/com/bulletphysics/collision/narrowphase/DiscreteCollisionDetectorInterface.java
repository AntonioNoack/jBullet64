package com.bulletphysics.collision.narrowphase;

import com.bulletphysics.linearmath.IDebugDraw;
import com.bulletphysics.linearmath.Transform;
import javax.vecmath.Vector3d;

/**
 * This interface is made to be used by an iterative approach to do TimeOfImpact calculations.<p>
 * 
 * This interface allows to query for closest points and penetration depth between two (convex) objects
 * the closest point is on the second object (B), and the normal points from the surface on B towards A.
 * distance is between closest points on B and closest point on A. So you can calculate closest point on A
 * by taking <code>closestPointInA = closestPointInB + distance * normalOnSurfaceB</code>.
 * 
 * @author jezek2
 */
public abstract class DiscreteCollisionDetectorInterface {

	public static abstract class Result {
		///setShapeIdentifiers provides experimental support for per-triangle material / custom material combiner
		public abstract void setShapeIdentifiers(int partId0, int index0, int partId1, int index1);

		public abstract void addContactPoint(Vector3d normalOnBInWorld, Vector3d pointInWorld, double depth);
	}
	
	public static class ClosestPointInput {
		public final Transform transformA = new Transform();
		public final Transform transformB = new Transform();
		public double maximumDistanceSquared;

		public ClosestPointInput() {
			init();
		}
		
		public void init() {
			maximumDistanceSquared = Float.MAX_VALUE;
		}
	}

	/**
	 * Give either closest points (distance > 0) or penetration (distance)
	 * the normal always points from B towards A.
	 */
	public final void getClosestPoints(ClosestPointInput input,Result output, IDebugDraw debugDraw) {
		getClosestPoints(input, output, debugDraw, false);
	}
	
	/**
	 * Give either closest points (distance > 0) or penetration (distance)
	 * the normal always points from B towards A.
	 */
	public abstract void getClosestPoints(ClosestPointInput input,Result output, IDebugDraw debugDraw, boolean swapResults);
	
}
