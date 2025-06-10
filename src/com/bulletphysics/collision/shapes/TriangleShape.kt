package com.bulletphysics.collision.shapes;

import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;
import org.jetbrains.annotations.NotNull;

import javax.vecmath.Vector3d;

/**
 * Single triangle shape.
 * 
 * @author jezek2
 */
public class TriangleShape extends PolyhedralConvexShape {
	
	public final Vector3d[] vertices = new Vector3d[] { new Vector3d(), new Vector3d(), new Vector3d() };

	// JAVA NOTE: added
	public TriangleShape() {
	}
	
	public TriangleShape(Vector3d p0, Vector3d p1, Vector3d p2) {
		vertices[0].set(p0);
		vertices[1].set(p1);
		vertices[2].set(p2);
	}
	
	// JAVA NOTE: added
	public void init(Vector3d p0, Vector3d p1, Vector3d p2) {
		vertices[0].set(p0);
		vertices[1].set(p1);
		vertices[2].set(p2);
	}

	@Override
	public int getNumVertices() {
		return 3;
	}

	public Vector3d getVertexPtr(int index) {
		return vertices[index];
	}
	
	@Override
	public void getVertex(int index, Vector3d vert) {
		vert.set(vertices[index]);
	}

	@Override
	public BroadphaseNativeType getShapeType() {
		return BroadphaseNativeType.TRIANGLE_SHAPE_PROXYTYPE;
	}

	@Override
	public int getNumEdges() {
		return 3;
	}

	@Override
	public void getEdge(int i, Vector3d pa, Vector3d pb) {
		getVertex(i, pa);
		getVertex((i + 1) % 3, pb);
	}

	@Override
	public void getAabb(Transform t, Vector3d aabbMin, Vector3d aabbMax) {
//		btAssert(0);
		getAabbSlow(t, aabbMin, aabbMax);
	}

	@NotNull
	@Override
	public Vector3d localGetSupportingVertexWithoutMargin(Vector3d dir, Vector3d out) {
		Vector3d dots = Stack.newVec();
		dots.set(dir.dot(vertices[0]), dir.dot(vertices[1]), dir.dot(vertices[2]));
		out.set(vertices[VectorUtil.maxAxis(dots)]);
		return out;
	}

	@Override
	public void batchedUnitVectorGetSupportingVertexWithoutMargin(Vector3d[] dirs, Vector3d[] outs, int numVectors) {
		Vector3d dots = Stack.newVec();

		for (int i = 0; i < numVectors; i++) {
			Vector3d dir = dirs[i];
			dots.set(dir.dot(vertices[0]), dir.dot(vertices[1]), dir.dot(vertices[2]));
			outs[i].set(vertices[VectorUtil.maxAxis(dots)]);
		}
	}

	@Override
	public void getPlane(Vector3d planeNormal, Vector3d planeSupport, int i) {
		getPlaneEquation(i,planeNormal,planeSupport);
	}

	@Override
	public int getNumPlanes() {
		return 1;
	}

	public void calcNormal(Vector3d normal) {
		Vector3d tmp1 = Stack.newVec();
		Vector3d tmp2 = Stack.newVec();

		tmp1.sub(vertices[1], vertices[0]);
		tmp2.sub(vertices[2], vertices[0]);

		normal.cross(tmp1, tmp2);
		normal.normalize();
	}

	public void getPlaneEquation(int i, Vector3d planeNormal, Vector3d planeSupport) {
		calcNormal(planeNormal);
		planeSupport.set(vertices[0]);
	}

	@Override
	public void calculateLocalInertia(double mass, Vector3d inertia) {
		assert (false);
		inertia.set(0.0, 0.0, 0.0);
	}
	
	@Override
	public boolean isInside(Vector3d pt, double tolerance) {
		Vector3d normal = Stack.newVec();
		calcNormal(normal);
		// distance to plane
		double dist = pt.dot(normal);
		double planeConst = vertices[0].dot(normal);
		dist -= planeConst;
		if (dist >= -tolerance && dist <= tolerance) {
			// inside check on edge-planes
			int i;
			for (i = 0; i < 3; i++) {
				Vector3d pa = Stack.newVec(), pb = Stack.newVec();
				getEdge(i, pa, pb);
				Vector3d edge = Stack.newVec();
				edge.sub(pb, pa);
				Vector3d edgeNormal = Stack.newVec();
				edgeNormal.cross(edge, normal);
				edgeNormal.normalize();
				/*double*/ dist = pt.dot(edgeNormal);
				double edgeConst = pa.dot(edgeNormal);
				dist -= edgeConst;
				if (dist < -tolerance) {
					return false;
				}
			}

			return true;
		}

		return false;
	}

	@Override
	public int getNumPreferredPenetrationDirections() {
		return 2;
	}

	@Override
	public void getPreferredPenetrationDirection(int index, Vector3d penetrationVector) {
		calcNormal(penetrationVector);
		if (index != 0) {
			penetrationVector.scale(-1.0);
		}
	}

}
