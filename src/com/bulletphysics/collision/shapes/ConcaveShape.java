package com.bulletphysics.collision.shapes;

import javax.vecmath.Vector3d;

/**
 * ConcaveShape class provides an interface for non-moving (static) concave shapes.
 * 
 * @author jezek2
 */
public abstract class ConcaveShape extends CollisionShape {

	protected double collisionMargin = 0.0;

	public abstract void processAllTriangles(TriangleCallback callback, Vector3d aabbMin, Vector3d aabbMax);

	public double getMargin() {
		return collisionMargin;
	}

	public void setMargin(double margin) {
		this.collisionMargin = margin;
	}
	
}
