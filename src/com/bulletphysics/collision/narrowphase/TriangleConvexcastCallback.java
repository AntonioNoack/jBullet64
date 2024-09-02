package com.bulletphysics.collision.narrowphase;

import com.bulletphysics.collision.narrowphase.ConvexCast.CastResult;
import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.collision.shapes.TriangleCallback;
import com.bulletphysics.collision.shapes.TriangleShape;
import com.bulletphysics.linearmath.Transform;
import javax.vecmath.Vector3d;

/**
 *
 * @author jezek2
 */
public abstract class TriangleConvexcastCallback extends TriangleCallback {

	public ConvexShape convexShape;
	public final Transform convexShapeFrom = new Transform();
	public final Transform convexShapeTo = new Transform();
	public final Transform triangleToWorld = new Transform();
	public double hitFraction;
	public double triangleCollisionMargin;

	public TriangleConvexcastCallback(ConvexShape convexShape, Transform convexShapeFrom, Transform convexShapeTo, Transform triangleToWorld, double triangleCollisionMargin) {
		this.convexShape = convexShape;
		this.convexShapeFrom.set(convexShapeFrom);
		this.convexShapeTo.set(convexShapeTo);
		this.triangleToWorld.set(triangleToWorld);
		this.hitFraction = 1.0;
		this.triangleCollisionMargin = triangleCollisionMargin;
	}
	
	public void processTriangle(Vector3d[] triangle, int partId, int triangleIndex) {
		TriangleShape triangleShape = new TriangleShape(triangle[0], triangle[1], triangle[2]);
		triangleShape.setMargin(triangleCollisionMargin);

		VoronoiSimplexSolver simplexSolver = new VoronoiSimplexSolver();

		// TODO: implement ContinuousConvexCollision
		SubsimplexConvexCast convexCaster = new SubsimplexConvexCast(convexShape, triangleShape, simplexSolver);

		CastResult castResult = new CastResult();
		castResult.fraction = 1.0;
		if (convexCaster.calcTimeOfImpact(convexShapeFrom, convexShapeTo, triangleToWorld, triangleToWorld, castResult)) {
			// add hit
			if (castResult.normal.lengthSquared() > 0.0001f) {
				if (castResult.fraction < hitFraction) {

					/* btContinuousConvexCast's normal is already in world space */
					/*
					// rotate normal into worldspace
					convexShapeFrom.basis.transform(castResult.normal);
					*/
					castResult.normal.normalize();

					reportHit(castResult.normal,
							castResult.hitPoint,
							castResult.fraction,
							partId,
							triangleIndex);
				}
			}
		}
	}

	public abstract double reportHit(Vector3d hitNormalLocal, Vector3d hitPointLocal, double hitFraction, int partId, int triangleIndex);
	
}
