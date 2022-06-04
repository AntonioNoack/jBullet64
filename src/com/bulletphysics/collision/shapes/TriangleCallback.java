package com.bulletphysics.collision.shapes;

import javax.vecmath.Vector3d;

/**
 * TriangleCallback provides a callback for each overlapping triangle when calling
 * processAllTriangles.<p>
 * 
 * This callback is called by processAllTriangles for all {@link ConcaveShape} derived
 * classes, such as {@link BvhTriangleMeshShape}, {@link StaticPlaneShape} and
 * {@link HeightfieldTerrainShape}.
 * 
 * @author jezek2
 */
public abstract class TriangleCallback {

	public abstract void processTriangle(Vector3d[] triangle, int partId, int triangleIndex);
	
}
