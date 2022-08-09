package com.bulletphysics.collision.shapes;

import com.bulletphysics.linearmath.AabbUtil2;
import com.bulletphysics.linearmath.MatrixUtil;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.linearmath.VectorUtil;
import cz.advel.stack.Stack;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

/**
 * Concave triangle mesh abstract class. Use {@link BvhTriangleMeshShape} as concrete
 * implementation.
 * 
 * @author jezek2
 */
public abstract class TriangleMeshShape extends ConcaveShape {

	protected final Vector3d localAabbMin = new Vector3d();
	protected final Vector3d localAabbMax = new Vector3d();
	protected StridingMeshInterface meshInterface;

	/**
	 * TriangleMeshShape constructor has been disabled/protected, so that users will not mistakenly use this class.
	 * Don't use btTriangleMeshShape but use btBvhTriangleMeshShape instead!
	 */
	protected TriangleMeshShape(StridingMeshInterface meshInterface) {
		this.meshInterface = meshInterface;
		
		// JAVA NOTE: moved to BvhTriangleMeshShape
		// recalcLocalAabb();
	}
	
	public Vector3d localGetSupportingVertex(Vector3d vec, Vector3d out) {
		Vector3d tmp = Stack.newVec();

		Transform identity = Stack.newTrans();
		identity.setIdentity();

		SupportVertexCallback supportCallback = new SupportVertexCallback(vec, identity);

		Vector3d aabbMax = Stack.newVec();
		aabbMax.set(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
		tmp.negate(aabbMax);

		processAllTriangles(supportCallback, tmp, aabbMax);

		supportCallback.getSupportVertexLocal(out);

		return out;
	}

	public Vector3d localGetSupportingVertexWithoutMargin(Vector3d vec, Vector3d out) {
		return localGetSupportingVertex(vec, out);
	}

	public void recalculateLocalAabb() {
		Vector3d vec = Stack.newVec();
		Vector3d tmp = Stack.newVec();
		vec.set(1.0, 0.0, 0.0);

		localGetSupportingVertex(vec, tmp);
		localAabbMax.x = tmp.x;
		vec.x = -1.0;
		localGetSupportingVertex(vec, tmp);
		localAabbMin.x = tmp.x;
		vec.x = 0.0;

		vec.y = 1.0;
		localGetSupportingVertex(vec, tmp);
		localAabbMax.y = tmp.y;
		vec.y = -1.0;
		localGetSupportingVertex(vec, tmp);
		localAabbMin.y = tmp.y;
		vec.y = 0.0;

		vec.z = 1.0;
		localGetSupportingVertex(vec, tmp);
		localAabbMax.z = tmp.z;
		vec.z = -1.0;
		localGetSupportingVertex(vec, tmp);
		localAabbMin.z = tmp.z;

		localAabbMax.x += collisionMargin;
		localAabbMax.y += collisionMargin;
		localAabbMax.z += collisionMargin;
		localAabbMin.x -= collisionMargin;
		localAabbMin.y -= collisionMargin;
		localAabbMin.z -= collisionMargin;

		Stack.subVec(2);
	}

	@Override
	public void getAabb(Transform trans, Vector3d aabbMin, Vector3d aabbMax) {
		Vector3d tmp = Stack.newVec();

		Vector3d localHalfExtents = Stack.newVec();
		localHalfExtents.sub(localAabbMax, localAabbMin);
		localHalfExtents.scale(0.5);

		Vector3d localCenter = Stack.newVec();
		localCenter.add(localAabbMax, localAabbMin);
		localCenter.scale(0.5);

		Matrix3d absB = Stack.newMat(trans.basis);
		MatrixUtil.absolute(absB);

		Vector3d center = Stack.newVec(localCenter);
		trans.transform(center);

		Vector3d extent = Stack.newVec();
		absB.getRow(0, tmp);
		extent.x = tmp.dot(localHalfExtents);
		absB.getRow(1, tmp);
		extent.y = tmp.dot(localHalfExtents);
		absB.getRow(2, tmp);
		extent.z = tmp.dot(localHalfExtents);

		Vector3d margin = Stack.newVec();
		margin.set(getMargin(), getMargin(), getMargin());
		extent.add(margin);

		aabbMin.sub(center, extent);
		aabbMax.add(center, extent);
	}

	@Override
	public void processAllTriangles(TriangleCallback callback, Vector3d aabbMin, Vector3d aabbMax) {
		FilteredCallback filterCallback = new FilteredCallback(callback, aabbMin, aabbMax);

		meshInterface.internalProcessAllTriangles(filterCallback, aabbMin, aabbMax);
	}

	@Override
	public void calculateLocalInertia(double mass, Vector3d inertia) {
		// moving concave objects not supported
		assert (false);
		inertia.set(0.0, 0.0, 0.0);
	}


	@Override
	public void setLocalScaling(Vector3d scaling) {
		meshInterface.setScaling(scaling);
		recalculateLocalAabb();
	}

	@Override
	public Vector3d getLocalScaling(Vector3d out) {
		return meshInterface.getScaling(out);
	}
	
	public StridingMeshInterface getMeshInterface() {
		return meshInterface;
	}

	public Vector3d getLocalAabbMin(Vector3d out) {
		out.set(localAabbMin);
		return out;
	}

	public Vector3d getLocalAabbMax(Vector3d out) {
		out.set(localAabbMax);
		return out;
	}

	@Override
	public String getName() {
		return "TRIANGLE_MESH";
	}
	
	////////////////////////////////////////////////////////////////////////////
	
	private static class SupportVertexCallback extends TriangleCallback {
		private final Vector3d supportVertexLocal = new Vector3d(0.0, 0.0, 0.0);
		public final Transform worldTrans = new Transform();
		public double maxDot = Double.NEGATIVE_INFINITY;
		public final Vector3d supportVecLocal = new Vector3d();

		public SupportVertexCallback(Vector3d supportVecWorld,Transform trans) {
			this.worldTrans.set(trans);
			MatrixUtil.transposeTransform(supportVecLocal, supportVecWorld, worldTrans.basis);
		}
		
		public void processTriangle(Vector3d[] triangle, int partId, int triangleIndex) {
			for (int i = 0; i < 3; i++) {
				double dot = supportVecLocal.dot(triangle[i]);
				if (dot > maxDot) {
					maxDot = dot;
					supportVertexLocal.set(triangle[i]);
				}
			}
		}

		public Vector3d getSupportVertexWorldSpace(Vector3d out) {
			out.set(supportVertexLocal);
			worldTrans.transform(out);
			return out;
		}

		public Vector3d getSupportVertexLocal(Vector3d out) {
			out.set(supportVertexLocal);
			return out;
		}
	}
	
	private static class FilteredCallback extends InternalTriangleIndexCallback {
		public TriangleCallback callback;
		public final Vector3d aabbMin = new Vector3d();
		public final Vector3d aabbMax = new Vector3d();

		public FilteredCallback(TriangleCallback callback, Vector3d aabbMin, Vector3d aabbMax) {
			this.callback = callback;
			this.aabbMin.set(aabbMin);
			this.aabbMax.set(aabbMax);
		}

		public void internalProcessTriangleIndex(Vector3d[] triangle, int partId, int triangleIndex) {
			if (AabbUtil2.testTriangleAgainstAabb2(triangle, aabbMin, aabbMax)) {
				// check aabb in triangle-space, before doing this
				callback.processTriangle(triangle, partId, triangleIndex);
			}
		}
	}

}
