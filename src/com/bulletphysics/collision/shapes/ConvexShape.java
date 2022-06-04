package com.bulletphysics.collision.shapes;

import com.bulletphysics.linearmath.Transform;
import javax.vecmath.Vector3d;

/**
 * ConvexShape is an abstract shape class. It describes general convex shapes
 * using the {@link #localGetSupportingVertex localGetSupportingVertex} interface
 * used in combination with GJK or ConvexCast.
 * 
 * @author jezek2
 */
public abstract class ConvexShape extends CollisionShape {

	public static final int MAX_PREFERRED_PENETRATION_DIRECTIONS = 10;
	
	public abstract Vector3d localGetSupportingVertex(Vector3d dir, Vector3d out);

	//#ifndef __SPU__
	public abstract Vector3d localGetSupportingVertexWithoutMargin(Vector3d dir, Vector3d out);

	//notice that the vectors should be unit length
	public abstract void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3d[] vectors, Vector3d[] supportVerticesOut, int numVectors);
	//#endif
	
	public abstract void getAabbSlow(Transform t, Vector3d aabbMin, Vector3d aabbMax);

	public abstract void setLocalScaling(Vector3d scaling);

	public abstract Vector3d getLocalScaling(Vector3d out);

	public abstract void setMargin(double margin);

	public abstract double getMargin();

	public abstract int getNumPreferredPenetrationDirections();

	public abstract void getPreferredPenetrationDirection(int index, Vector3d penetrationVector);
	
}
