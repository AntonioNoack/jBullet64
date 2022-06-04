package com.bulletphysics.extras.gimpact;

import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.shapes.TriangleCallback;
import javax.vecmath.Vector3d;

/**
 *
 * @author jezek2
 */
class GImpactTriangleCallback extends TriangleCallback {

	public GImpactCollisionAlgorithm algorithm;
	public CollisionObject body0;
	public CollisionObject body1;
	public GImpactShapeInterface gimpactShape0;
	public boolean swapped;
	public double margin;
	
	public void processTriangle(Vector3d[] triangle, int partId, int triangleIndex) {
		TriangleShapeEx tri1 = new TriangleShapeEx(triangle[0], triangle[1], triangle[2]);
		tri1.setMargin(margin);
		if (swapped) {
			algorithm.setPart0(partId);
			algorithm.setFace0(triangleIndex);
		}
		else {
			algorithm.setPart1(partId);
			algorithm.setFace1(triangleIndex);
		}
		algorithm.gimpact_vs_shape(body0, body1, gimpactShape0, tri1, swapped);
	}

}
