package com.bulletphysics.collision.shapes;

import javax.vecmath.Vector3d;

/**
 * Callback for internal processing of triangles.
 * 
 * @see StridingMeshInterface#internalProcessAllTriangles
 * @author jezek2
 */
public abstract class InternalTriangleIndexCallback {

	public abstract void internalProcessTriangleIndex(Vector3d[] triangle, int partId, int triangleIndex);
	
}
